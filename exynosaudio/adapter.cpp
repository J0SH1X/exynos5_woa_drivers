/** @file
  exynosaudio adapter: device start (resources, board, codec), the WaveRT
  and topology subdevices, and the render start / stop the stream uses.

  ACPI (_HID EXYN500B), in _CRS order:
    Memory32Fixed x3   I2S0, audio subsystem clocks, GPZ
    Interrupt          I2S0 (GSI 147; not used yet: polling mode)
    SpiSerialBus       the WM5102 on \_SB.SPI2
    GpioIo x3          LDOENA (pin 41), /RESET (pin 48), MAINMIC_BIAS (pin 62)
  FixedDMA           I2S0 RX capture: request line 2, channel 0 (ADMA0,
                     system DMA through the halextpl330 HAL extension)
    GpioInt            codec IRQ (pin 21, level high): headphone jack, handled
                     at PASSIVE_LEVEL (the handler talks SPI to the codec)
  The PMU (MCLK1 on CLKOUT, MAU power domain) is mapped unclaimed.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include "exynosaudio.h"

EXYNOS_AUDIO  gAudio;

#define PMU_MAP_BYTES  0x5000           // up to MAU_STATUS (0x40E4)

extern "C" DRIVER_INITIALIZE  DriverEntry;
static DRIVER_ADD_DEVICE      AddDevice;
static DRIVER_DISPATCH        DispatchPnp;
static PDRIVER_DISPATCH       mPcDispatchPnp;

extern "C" int __cdecl
_purecall (void)
{
  KeBugCheckEx (0x41554449, 0, 0, 0, 0);     // "AUDI": pure virtual call
}

//
// ---------------------------------------------------------------------------
// Connections through the resource hub (SPB, GPIO).
// ---------------------------------------------------------------------------
//

/**
  Access: GENERIC_READ | GENERIC_WRITE for SPB; GENERIC_WRITE alone for
  GpioIo lines declared IoRestrictionOutputOnly (GpioClx refuses read access
  to those with STATUS_INVALID_PARAMETER).
**/
static NTSTATUS
OpenConnection (ULONG IdLow, ULONG IdHigh, ACCESS_MASK Access, HANDLE *Handle, PFILE_OBJECT *File)
{
  DECLARE_UNICODE_STRING_SIZE (Path, RESOURCE_HUB_PATH_SIZE);
  OBJECT_ATTRIBUTES  Attributes;
  IO_STATUS_BLOCK    Iosb;
  NTSTATUS           Status;

  Status = RESOURCE_HUB_CREATE_PATH_FROM_ID (&Path, IdLow, IdHigh);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  InitializeObjectAttributes (&Attributes, &Path, OBJ_KERNEL_HANDLE | OBJ_CASE_INSENSITIVE, NULL, NULL);
  Status = ZwCreateFile (Handle, Access, &Attributes, &Iosb, NULL, FILE_ATTRIBUTE_NORMAL, 0,
                         FILE_OPEN, FILE_NON_DIRECTORY_FILE, NULL, 0);
  if (!NT_SUCCESS (Status)) {
    *Handle = NULL;
    return Status;
  }
  Status = ObReferenceObjectByHandle (*Handle, 0, *IoFileObjectType, KernelMode, (PVOID *)File, NULL);
  if (!NT_SUCCESS (Status)) {
    ZwClose (*Handle);
    *Handle = NULL;
    *File   = NULL;
  }
  return Status;
}

static VOID
CloseConnection (HANDLE *Handle, PFILE_OBJECT *File)
{
  if (*File != NULL) {
    ObDereferenceObject (*File);
    *File = NULL;
  }
  if (*Handle != NULL) {
    ZwClose (*Handle);
    *Handle = NULL;
  }
}

/**
  One synchronous request to the connection's target: a write (Ioctl 0)
  or an IOCTL with input / output buffers.
**/
static NTSTATUS
SendRequest (PFILE_OBJECT File, ULONG Ioctl, PVOID In, ULONG InLength, PVOID Out, ULONG OutLength)
{
  PDEVICE_OBJECT   Target;
  KEVENT           Event;
  IO_STATUS_BLOCK  Iosb;
  LARGE_INTEGER    Offset;
  PIRP             Irp;
  NTSTATUS         Status;

  if (File == NULL) {
    return STATUS_DEVICE_NOT_CONNECTED;
  }
  Target = IoGetRelatedDeviceObject (File);
  KeInitializeEvent (&Event, NotificationEvent, FALSE);
  if (Ioctl == 0) {
    Offset.QuadPart = 0;
    Irp = IoBuildSynchronousFsdRequest (IRP_MJ_WRITE, Target, In, InLength, &Offset, &Event, &Iosb);
  } else {
    Irp = IoBuildDeviceIoControlRequest (Ioctl, Target, In, InLength, Out, OutLength, FALSE, &Event, &Iosb);
  }
  if (Irp == NULL) {
    return STATUS_INSUFFICIENT_RESOURCES;
  }
  IoGetNextIrpStackLocation (Irp)->FileObject = File;
  Status = IoCallDriver (Target, Irp);
  if (Status == STATUS_PENDING) {
    KeWaitForSingleObject (&Event, Executive, KernelMode, FALSE, NULL);
    Status = Iosb.Status;
  }
  return Status;
}

static NTSTATUS
GpioSet (ULONG Index, BOOLEAN High)
{
  UCHAR  Value;

  Value = High ? 1 : 0;
  return SendRequest (gAudio.GpioFile[Index], IOCTL_GPIO_WRITE_PINS, &Value, sizeof (Value), NULL, 0);
}

/**
  Wm5102Core's bus: one chip-select cycle, a write, or a write-then-read
  sequence (exynosspi keeps the chip selected across a sequence).
**/
static INTN
CodecXfer (VOID *Context, CONST UINT8 *Tx, UINTN TxLength, UINT8 *Rx, UINTN RxLength)
{
  SPB_TRANSFER_LIST_AND_ENTRIES (2)  Sequence;
  NTSTATUS                           Status;

  UNREFERENCED_PARAMETER (Context);

  if (RxLength == 0) {
    Status = SendRequest (gAudio.SpiFile, 0, (PVOID)Tx, (ULONG)TxLength, NULL, 0);
  } else {
    SPB_TRANSFER_LIST_INIT (&Sequence.List, 2);
    Sequence.List.Transfers[0] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE (SpbTransferDirectionToDevice, 0, (PVOID)Tx,
                                                                      (ULONG)TxLength);
    Sequence.List.Transfers[1] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE (SpbTransferDirectionFromDevice, 0, Rx,
                                                                      (ULONG)RxLength);
    Status = SendRequest (gAudio.SpiFile, IOCTL_SPB_EXECUTE_SEQUENCE, &Sequence, sizeof (Sequence), NULL, 0);
  }
  if (!NT_SUCCESS (Status)) {
    if (gAudio.SpiErrors++ < 10) {
      OsLog ("exynosaudio: SPI request failed: 0x%08lx", Status);
    }
    return -1;
  }
  return 0;
}

//
// ---------------------------------------------------------------------------
// Render path (called by the stream at PASSIVE_LEVEL).
// ---------------------------------------------------------------------------
//

static UINT8
DacValue (LONG Level)
{
  Level = max (min (Level, (LONG)AUDIO_VOLUME_MAX), (LONG)AUDIO_VOLUME_MIN);
  return (UINT8)(AUDIO_DAC_VOLUME + Level / AUDIO_VOLUME_STEP);
}

/**
  Topology node changes: remembered, and sent to the codec while playing
  (the codec is reset between streams, AudioRenderStart() applies them).
**/
VOID
AudioSetVolume (ULONG Channel, LONG Level)
{
  ULONG  Index;

  KeWaitForSingleObject (&gAudio.Lock, Executive, KernelMode, FALSE, NULL);
  for (Index = 0; Index < AUDIO_CHANNELS; Index++) {
    if ((Channel == (ULONG)~0) || (Channel == Index)) {
      gAudio.Volume[Index] = max (min (Level, (LONG)AUDIO_VOLUME_MAX), (LONG)AUDIO_VOLUME_MIN);
    }
  }
  if (gAudio.Rendering) {
    Wm5102SetVolumeLR (&gAudio.Codec, DacValue (gAudio.Volume[0]), DacValue (gAudio.Volume[1]), gAudio.Mute);
  }
  KeReleaseMutex (&gAudio.Lock, FALSE);
}

VOID
AudioSetMute (BOOLEAN Mute)
{
  KeWaitForSingleObject (&gAudio.Lock, Executive, KernelMode, FALSE, NULL);
  gAudio.Mute = Mute;
  if (gAudio.Rendering) {
    Wm5102SetVolumeLR (&gAudio.Codec, DacValue (gAudio.Volume[0]), DacValue (gAudio.Volume[1]), gAudio.Mute);
  }
  KeReleaseMutex (&gAudio.Lock, FALSE);
}

NTSTATUS
AudioRenderStart (ULONG Physical, ULONG Bytes)
{
  NTSTATUS  Status;

  KeWaitForSingleObject (&gAudio.Lock, Executive, KernelMode, FALSE, NULL);
  Status = STATUS_DEVICE_NOT_READY;
  if (gAudio.CodecReady && !gAudio.Rendering) {
    if (Wm5102StartPlayback (&gAudio.Codec, AUDIO_DAC_VOLUME) == 0) {
      Wm5102SetVolumeLR (&gAudio.Codec, DacValue (gAudio.Volume[0]), DacValue (gAudio.Volume[1]), gAudio.Mute);
      ExynosI2sStart (&gAudio.I2s, Physical, Bytes, Bytes / 4, FALSE);
      gAudio.Rendering = TRUE;
      Status = STATUS_SUCCESS;
    } else {
      Wm5102LogStatus (&gAudio.Codec);
    }
  }
  KeReleaseMutex (&gAudio.Lock, FALSE);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosaudio: render start failed: 0x%08lx", Status);
  }
  return Status;
}

VOID
AudioRenderStop (VOID)
{
  KeWaitForSingleObject (&gAudio.Lock, Executive, KernelMode, FALSE, NULL);
  if (gAudio.Rendering) {
    ExynosI2sStop (&gAudio.I2s);
    Wm5102StopPlayback (&gAudio.Codec);
    gAudio.Rendering = FALSE;
  }
  KeReleaseMutex (&gAudio.Lock, FALSE);
}

ULONG
AudioRenderPosition (VOID)
{
  return gAudio.Rendering ? ExynosI2sPosition (&gAudio.I2s) : 0;
}

//
// ---------------------------------------------------------------------------
// Device start / stop.
// ---------------------------------------------------------------------------
//

static PUCHAR
MapPhysical (ULONG Address, ULONG Bytes)
{
  PHYSICAL_ADDRESS  Physical;

  Physical.QuadPart = Address;
  return (PUCHAR)MmMapIoSpace (Physical, Bytes, MmNonCached);
}

//
// ---------------------------------------------------------------------------
// Codec interrupt: the headphone jack.  A GPIO interrupt connected at
// PASSIVE_LEVEL (CONNECT_FULLY_SPECIFIED with Irql / SynchronizeIrql =
// PASSIVE_LEVEL, as KMDF does it on Windows 8.x), so the handler can read
// and acknowledge the codec over SPI before the level-triggered line drops.
// The output follows the plug, also in the middle of a stream.
// ---------------------------------------------------------------------------
//

static BOOLEAN
CodecIsr (PKINTERRUPT Interrupt, PVOID Context)
{
  UINT16   Pending;
  BOOLEAN  In;

  UNREFERENCED_PARAMETER (Interrupt);
  UNREFERENCED_PARAMETER (Context);

  KeWaitForSingleObject (&gAudio.Lock, Executive, KernelMode, FALSE, NULL);
  gAudio.CodecIrqs++;
  if (gAudio.CodecReady) {
    Pending = Wm5102IrqAcknowledge (&gAudio.Codec);
    In      = Wm5102JackPresent (&gAudio.Codec);
    if (In != gAudio.JackIn) {
      gAudio.JackIn = In;
      Wm5102SelectOutput (&gAudio.Codec, In);
      OsLog ("exynosaudio: headphones %s (codec IRQ 0x%04x), playing through the %s", In ? "plugged in" : "unplugged",
             Pending, In ? "headphones" : "speakers");
    }
  }
  KeReleaseMutex (&gAudio.Lock, FALSE);
  return TRUE;                          // Exclusive line: always ours
}

static NTSTATUS
CodecIrqConnect (PRESOURCELIST Resources)
{
  IO_CONNECT_INTERRUPT_PARAMETERS  Params;
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  PDEVICE_OBJECT                   Pdo;
  NTSTATUS                         Status;

  //
  // Interrupt resources in _CRS order: I2S0 (GSI 147), then the GpioInt.
  //
  Desc = Resources->FindTranslatedEntry (CmResourceTypeInterrupt, 1);
  if (Desc == NULL) {
    return STATUS_DEVICE_CONFIGURATION_ERROR;
  }
  Status = PcGetPhysicalDeviceObject (gAudio.Fdo, &Pdo);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  RtlZeroMemory (&Params, sizeof (Params));
  Params.Version                             = CONNECT_FULLY_SPECIFIED;
  Params.FullySpecified.PhysicalDeviceObject = Pdo;
  Params.FullySpecified.InterruptObject      = &gAudio.CodecInterrupt;
  Params.FullySpecified.ServiceRoutine       = CodecIsr;
  Params.FullySpecified.ServiceContext       = NULL;
  Params.FullySpecified.SpinLock             = NULL;
  Params.FullySpecified.SynchronizeIrql      = PASSIVE_LEVEL;
  Params.FullySpecified.FloatingSave         = FALSE;
  Params.FullySpecified.ShareVector          = (Desc->ShareDisposition == CmResourceShareShared);
  Params.FullySpecified.Vector               = Desc->u.Interrupt.Vector;
  Params.FullySpecified.Irql                 = PASSIVE_LEVEL;
  Params.FullySpecified.InterruptMode        = (Desc->Flags & CM_RESOURCE_INTERRUPT_LATCHED) ? Latched : LevelSensitive;
  Params.FullySpecified.ProcessorEnableMask  = Desc->u.Interrupt.Affinity;
  Status = IoConnectInterruptEx (&Params);
  OsLog ("exynosaudio: codec IRQ (vector %lu, %s) connected at PASSIVE_LEVEL - 0x%08lx", Desc->u.Interrupt.Vector,
         (Desc->Flags & CM_RESOURCE_INTERRUPT_LATCHED) ? "edge" : "level", Status);
  if (!NT_SUCCESS (Status)) {
    gAudio.CodecInterrupt = NULL;
  }
  return Status;
}

static VOID
CodecIrqDisconnect (VOID)
{
  IO_DISCONNECT_INTERRUPT_PARAMETERS  Params;

  if (gAudio.CodecInterrupt != NULL) {
    RtlZeroMemory (&Params, sizeof (Params));
    Params.Version                           = CONNECT_FULLY_SPECIFIED;
    Params.ConnectionContext.InterruptObject = gAudio.CodecInterrupt;
    IoDisconnectInterruptEx (&Params);
    gAudio.CodecInterrupt = NULL;
  }
}

//
// ---------------------------------------------------------------------------
// Capture through system DMA.
// ---------------------------------------------------------------------------
//

static VOID
LogResources (PCSTR Which, PCM_RESOURCE_LIST List)
{
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  ULONG                            Index;

  if ((List == NULL) || (List->Count == 0)) {
    OsLog ("exynosaudio: %s resources: none", Which);
    return;
  }
  for (Index = 0; Index < List->List[0].PartialResourceList.Count; Index++) {
    Desc = &List->List[0].PartialResourceList.PartialDescriptors[Index];
    OsLog ("exynosaudio: %s resource %lu: type %u flags 0x%04x data %08lx %08lx %08lx", Which, Index,
           (ULONG)Desc->Type, (ULONG)Desc->Flags, Desc->u.Generic.Start.LowPart,
           Desc->u.Generic.Start.HighPart, Desc->u.Generic.Length);
  }
}

static NTSTATUS
CaptureDmaOpen (PRESOURCELIST Resources)
{
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  DEVICE_DESCRIPTION               Device;
  PDEVICE_OBJECT                   Pdo;
  NTSTATUS                         Status;

  Desc = Resources->FindTranslatedEntry (CmResourceTypeDma, 0);
  if ((Desc == NULL) || ((Desc->Flags & CM_RESOURCE_DMA_V3) == 0)) {
    OsLog ("exynosaudio: no FixedDMA resource: no capture");
    LogResources ("raw", Resources->UntranslatedList ());
    LogResources ("translated", Resources->TranslatedList ());
    return STATUS_DEVICE_CONFIGURATION_ERROR;
  }
  gAudio.DmaRequestLine = Desc->u.DmaV3.RequestLine;
  gAudio.DmaChannel     = Desc->u.DmaV3.Channel;
  Status = PcGetPhysicalDeviceObject (gAudio.Fdo, &Pdo);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  RtlZeroMemory (&Device, sizeof (Device));
  Device.Version                 = DEVICE_DESCRIPTION_VERSION3;
  Device.Master                  = FALSE;           // system DMA
  Device.AutoInitialize          = TRUE;            // loop over the buffer
  Device.Dma32BitAddresses       = TRUE;
  Device.InterfaceType           = ACPIBus;
  Device.DmaChannel              = gAudio.DmaChannel;
  Device.DmaRequestLine          = gAudio.DmaRequestLine;
  Device.DmaWidth                = Width32Bits;
  Device.DmaAddressWidth         = 32;
  Device.MaximumLength           = AUDIO_MAX_BUFFER + PAGE_SIZE;
  Device.DeviceAddress.QuadPart  = EXYNOS_I2S0_RXD_PHYS;
  gAudio.DmaAdapter = IoGetDmaAdapter (Pdo, &Device, &gAudio.DmaMapRegisters);
  OsLog ("exynosaudio: capture DMA request line %lu channel %lu: adapter %p, %lu map registers",
         gAudio.DmaRequestLine, gAudio.DmaChannel, gAudio.DmaAdapter, gAudio.DmaMapRegisters);
  return (gAudio.DmaAdapter != NULL) ? STATUS_SUCCESS : STATUS_INSUFFICIENT_RESOURCES;
}

/**
  Record into the WaveRT buffer: the headset mic if a plug is in, else the
  main mic (its external bias on); DMA looping over Mdl first, then the
  I2S0 RX request.
**/
NTSTATUS
AudioCaptureStart (PMDL Mdl, ULONG Bytes)
{
  PDMA_OPERATIONS  Ops;
  ULONG            Length;
  NTSTATUS         Status;

  KeWaitForSingleObject (&gAudio.Lock, Executive, KernelMode, FALSE, NULL);
  if (!gAudio.CodecReady || gAudio.Capturing || (gAudio.DmaAdapter == NULL)) {
    KeReleaseMutex (&gAudio.Lock, FALSE);
    return STATUS_DEVICE_NOT_READY;
  }
  if (!gAudio.JackIn && (gAudio.GpioFile[2] != NULL)) {
    GpioSet (2, TRUE);
  }
  if (Wm5102StartCapture (&gAudio.Codec, gAudio.JackIn, AUDIO_MIC_GAIN_DB) != 0) {
    Status = STATUS_DEVICE_NOT_READY;
    goto Fail;
  }

  Ops    = gAudio.DmaAdapter->DmaOperations;
  Status = Ops->InitializeDmaTransferContext (gAudio.DmaAdapter, gAudio.DmaContext);
  if (NT_SUCCESS (Status)) {
    Status = Ops->AllocateAdapterChannelEx (gAudio.DmaAdapter, gAudio.Fdo, gAudio.DmaContext,
                                            min ((ULONG)BYTES_TO_PAGES (Bytes), gAudio.DmaMapRegisters),
                                            DMA_SYNCHRONOUS_CALLBACK, NULL, NULL, &gAudio.DmaMapRegisterBase);
  }
  if (NT_SUCCESS (Status)) {
    Length = Bytes;
    Status = Ops->MapTransferEx (gAudio.DmaAdapter, Mdl, gAudio.DmaMapRegisterBase, 0, 0, &Length, FALSE, NULL, 0,
                                 NULL, NULL);
    if (!NT_SUCCESS (Status)) {
      Ops->FreeAdapterChannel (gAudio.DmaAdapter);
    } else {
      gAudio.CaptureBytes = Length;
    }
  }
  if (!NT_SUCCESS (Status)) {
    Wm5102StopCapture (&gAudio.Codec);
    goto Fail;
  }

  ExynosI2sRxStart (&gAudio.I2s, TRUE);
  gAudio.CaptureMdl = Mdl;
  gAudio.Capturing  = TRUE;
  KeReleaseMutex (&gAudio.Lock, FALSE);
  return STATUS_SUCCESS;

Fail:
  if (gAudio.GpioFile[2] != NULL) {
    GpioSet (2, FALSE);
  }
  KeReleaseMutex (&gAudio.Lock, FALSE);
  OsLog ("exynosaudio: capture start failed: 0x%08lx", Status);
  return Status;
}

VOID
AudioCaptureStop (VOID)
{
  PDMA_OPERATIONS  Ops;

  KeWaitForSingleObject (&gAudio.Lock, Executive, KernelMode, FALSE, NULL);
  if (gAudio.Capturing) {
    ExynosI2sRxStop (&gAudio.I2s);
    Ops = gAudio.DmaAdapter->DmaOperations;
    Ops->CancelMappedTransfer (gAudio.DmaAdapter, gAudio.DmaContext);
    Ops->FlushAdapterBuffersEx (gAudio.DmaAdapter, gAudio.CaptureMdl, gAudio.DmaMapRegisterBase, 0,
                                gAudio.CaptureBytes, FALSE);
    Ops->FreeAdapterChannel (gAudio.DmaAdapter);
    Wm5102StopCapture (&gAudio.Codec);
    if (gAudio.GpioFile[2] != NULL) {
      GpioSet (2, FALSE);
    }
    gAudio.Capturing = FALSE;
  }
  KeReleaseMutex (&gAudio.Lock, FALSE);
}

ULONG
AudioCapturePosition (VOID)
{
  ULONG  Remaining;

  if (!gAudio.Capturing || (gAudio.CaptureBytes == 0)) {
    return 0;
  }
  Remaining = gAudio.DmaAdapter->DmaOperations->ReadDmaCounter (gAudio.DmaAdapter);
  if ((Remaining == 0) || (Remaining > gAudio.CaptureBytes)) {
    return 0;
  }
  return gAudio.CaptureBytes - Remaining;
}

static VOID
BoardDown (VOID)
{
  CodecIrqDisconnect ();
  AudioCaptureStop ();
  AudioRenderStop ();
  if (gAudio.GpioFile[1] != NULL) {
    GpioSet (1, FALSE);                 // codec in reset
  }
  if (gAudio.GpioFile[0] != NULL) {
    GpioSet (0, FALSE);                 // LDO off
  }
  gAudio.CodecReady = FALSE;
  CloseConnection (&gAudio.SpiHandle, &gAudio.SpiFile);
  CloseConnection (&gAudio.GpioHandle[0], &gAudio.GpioFile[0]);
  CloseConnection (&gAudio.GpioHandle[1], &gAudio.GpioFile[1]);
  CloseConnection (&gAudio.GpioHandle[2], &gAudio.GpioFile[2]);
  if (gAudio.DmaAdapter != NULL) {
    gAudio.DmaAdapter->DmaOperations->PutDmaAdapter (gAudio.DmaAdapter);
    gAudio.DmaAdapter = NULL;
  }
}

/**
  Resources, connections, codec power-up and identification.
**/
static NTSTATUS
BoardUp (PRESOURCELIST Resources)
{
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  ULONG                            Count;
  ULONG                            Index;
  ULONG                            Gpios;
  PUCHAR                           *Map[3];
  NTSTATUS                         Status;

  Map[0] = &gAudio.I2sVa;
  Map[1] = &gAudio.AudssVa;
  Map[2] = &gAudio.GpzVa;
  Count  = Resources->NumberOfEntriesOfType (CmResourceTypeMemory);
  for (Index = 0; (Index < Count) && (Index < 3); Index++) {
    Desc = Resources->FindTranslatedEntry (CmResourceTypeMemory, Index);
    if ((Desc != NULL) && (*Map[Index] == NULL)) {
      *Map[Index] = (PUCHAR)MmMapIoSpace (Desc->u.Memory.Start, Desc->u.Memory.Length, MmNonCached);
    }
  }
  if (gAudio.PmuVa == NULL) {
    gAudio.PmuVa = MapPhysical (EXYNOS_PMU_PHYS, PMU_MAP_BYTES);
  }
  if ((gAudio.I2sVa == NULL) || (gAudio.AudssVa == NULL) || (gAudio.GpzVa == NULL) || (gAudio.PmuVa == NULL)) {
    OsLog ("exynosaudio: register blocks missing (%lu memory resources)", Count);
    return STATUS_DEVICE_CONFIGURATION_ERROR;
  }
  gAudio.I2s.I2s   = (UINTN)gAudio.I2sVa;
  gAudio.I2s.Audss = (UINTN)gAudio.AudssVa;
  gAudio.I2s.Gpz   = (UINTN)gAudio.GpzVa;
  gAudio.I2s.Pmu   = (UINTN)gAudio.PmuVa;

  //
  // The SPI connection and the two GPIO lines, in _CRS order.
  //
  Gpios  = 0;
  Status = STATUS_SUCCESS;
  Count  = Resources->NumberOfEntriesOfType (CmResourceTypeConnection);
  for (Index = 0; (Index < Count) && NT_SUCCESS (Status); Index++) {
    Desc = Resources->FindTranslatedEntry (CmResourceTypeConnection, Index);
    if (Desc == NULL) {
      continue;
    }
    if ((Desc->u.Connection.Class == CM_RESOURCE_CONNECTION_CLASS_SERIAL) && (gAudio.SpiFile == NULL)) {
      Status = OpenConnection (Desc->u.Connection.IdLowPart, Desc->u.Connection.IdHighPart, GENERIC_READ | GENERIC_WRITE,
                               &gAudio.SpiHandle, &gAudio.SpiFile);
    } else if ((Desc->u.Connection.Class == CM_RESOURCE_CONNECTION_CLASS_GPIO) && (Gpios < 3)) {
      Status = OpenConnection (Desc->u.Connection.IdLowPart, Desc->u.Connection.IdHighPart, GENERIC_WRITE,
                               &gAudio.GpioHandle[Gpios], &gAudio.GpioFile[Gpios]);
      if (!NT_SUCCESS (Status)) {
        OsLog ("exynosaudio: GPIO line %lu open failed: 0x%08lx", Gpios, Status);
      }
      Gpios++;
    }
  }
  if (!NT_SUCCESS (Status) || (gAudio.SpiFile == NULL) || (Gpios < 2)) {
    OsLog ("exynosaudio: connections: SPI %s, %lu GPIO lines - 0x%08lx", (gAudio.SpiFile != NULL) ? "open" : "missing",
           Gpios, Status);
    return NT_SUCCESS (Status) ? STATUS_DEVICE_CONFIGURATION_ERROR : Status;
  }

  //
  // Codec in reset, LDO on, audio block and MCLK1 up, out of reset.
  //
  GpioSet (1, FALSE);
  GpioSet (0, TRUE);
  if (ExynosI2sPowerUp (&gAudio.I2s) != 0) {
    return STATUS_DEVICE_POWER_FAILURE;
  }
  EnvSleepMs (10);
  GpioSet (1, TRUE);
  EnvSleepMs (5);

  RtlZeroMemory (&gAudio.Codec, sizeof (gAudio.Codec));
  gAudio.Codec.Xfer = CodecXfer;
  if (Wm5102Init (&gAudio.Codec) != 0) {
    OsLog ("exynosaudio: WM5102 not found or did not boot");
    return STATUS_DEVICE_NOT_READY;
  }
  //
  // Headphone jack: detection on, the codec's IRQ pin for it, the current
  // state, then the interrupt (a plug change after this point raises it).
  //
  Wm5102JackInit (&gAudio.Codec);
  Wm5102IrqInit (&gAudio.Codec);
  EnvSleepMs (100);                     // clamp detection debounce
  Wm5102IrqAcknowledge (&gAudio.Codec);
  gAudio.JackIn           = Wm5102JackPresent (&gAudio.Codec);
  gAudio.Codec.Headphones = gAudio.JackIn;
  Wm5102LogStatus (&gAudio.Codec);
  OsLog ("exynosaudio: headphones %s", gAudio.JackIn ? "plugged in" : "not plugged in");
  ExynosI2sConfigure (&gAudio.I2s);
  gAudio.CodecReady = TRUE;
  if (!NT_SUCCESS (CodecIrqConnect (Resources))) {
    OsLog ("exynosaudio: no codec interrupt: the output stays on the %s", gAudio.JackIn ? "headphones" : "speakers");
  }
  CaptureDmaOpen (Resources);           // optional: playback works without it
  return STATUS_SUCCESS;
}

static NTSTATUS
InstallSubdevice (PDEVICE_OBJECT Fdo, PIRP Irp, PWSTR Name, REFGUID PortClass, PFNCREATEINSTANCE Create,
                  PRESOURCELIST Resources, PUNKNOWN *PortUnknown)
{
  PPORT     Port;
  PUNKNOWN  Miniport;
  NTSTATUS  Status;

  Port     = NULL;
  Miniport = NULL;
  Status   = PcNewPort (&Port, PortClass);
  if (NT_SUCCESS (Status)) {
    Status = Create (&Miniport, PortClass, NULL, NonPagedPool);
  }
  if (NT_SUCCESS (Status)) {
    Status = Port->Init (Fdo, Irp, Miniport, NULL, Resources);
  }
  if (NT_SUCCESS (Status)) {
    Status = PcRegisterSubdevice (Fdo, Name, Port);
  }
  if (Miniport != NULL) {
    Miniport->Release ();
  }
  if (NT_SUCCESS (Status)) {
    *PortUnknown = Port;
  } else {
    OsLog ("exynosaudio: subdevice %ws failed: 0x%08lx", Name, Status);
    if (Port != NULL) {
      Port->Release ();
    }
  }
  return Status;
}

static NTSTATUS
StartDevice (PDEVICE_OBJECT Fdo, PIRP Irp, PRESOURCELIST Resources)
{
  PUNKNOWN  WavePort;
  PUNKNOWN  TopologyPort;
  NTSTATUS  Status;

  gAudio.Fdo = Fdo;
  Status = BoardUp (Resources);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosaudio: board bring-up failed: 0x%08lx", Status);
    BoardDown ();
    return Status;
  }

  WavePort     = NULL;
  TopologyPort = NULL;
  Status = InstallSubdevice (Fdo, Irp, (PWSTR)L"Wave", CLSID_PortWaveRT, CreateWaveMiniport, Resources, &WavePort);
  if (NT_SUCCESS (Status)) {
    Status = InstallSubdevice (Fdo, Irp, (PWSTR)L"Topology", CLSID_PortTopology, CreateTopologyMiniport, Resources,
                               &TopologyPort);
  }
  if (NT_SUCCESS (Status)) {
    Status = PcRegisterPhysicalConnection (Fdo, WavePort, 1, TopologyPort, 0);       // render
  }
  if (NT_SUCCESS (Status)) {
    Status = PcRegisterPhysicalConnection (Fdo, TopologyPort, 3, WavePort, 2);       // capture
  }
  if (WavePort != NULL) {
    WavePort->Release ();
  }
  if (TopologyPort != NULL) {
    TopologyPort->Release ();
  }
  OsLog ("exynosaudio: started - 0x%08lx", Status);
  return Status;
}

/**
  Power the codec down on stop / remove, then let PortCls handle the IRP.
**/
static NTSTATUS
DispatchPnp (PDEVICE_OBJECT DeviceObject, PIRP Irp)
{
  PIO_STACK_LOCATION  Stack;

  Stack = IoGetCurrentIrpStackLocation (Irp);
  if ((Stack->MinorFunction == IRP_MN_STOP_DEVICE) || (Stack->MinorFunction == IRP_MN_REMOVE_DEVICE) ||
      (Stack->MinorFunction == IRP_MN_SURPRISE_REMOVAL))
  {
    OsLog ("exynosaudio: PnP minor 0x%x, codec off (%lu streams, %lu SPI errors, %lu codec IRQs)", Stack->MinorFunction,
           gAudio.Streams, gAudio.SpiErrors, gAudio.CodecIrqs);
    BoardDown ();
  }
  return mPcDispatchPnp (DeviceObject, Irp);
}

static NTSTATUS
AddDevice (PDRIVER_OBJECT DriverObject, PDEVICE_OBJECT PhysicalDeviceObject)
{
  return PcAddAdapterDevice (DriverObject, PhysicalDeviceObject, StartDevice, 2, 0);
}

extern "C" NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  NTSTATUS  Status;

  OsLogInit ();
  OsLog ("exynosaudio: DriverEntry (built " __DATE__ " " __TIME__ ")");
  KeInitializeMutex (&gAudio.Lock, 0);
  gAudio.Volume[0] = -12 * 65536;      // until the endpoint sets its own
  gAudio.Volume[1] = -12 * 65536;
  Status = PcInitializeAdapterDriver (DriverObject, RegistryPath, AddDevice);
  if (NT_SUCCESS (Status)) {
    mPcDispatchPnp                          = DriverObject->MajorFunction[IRP_MJ_PNP];
    DriverObject->MajorFunction[IRP_MJ_PNP] = DispatchPnp;
  }
  return Status;
}
