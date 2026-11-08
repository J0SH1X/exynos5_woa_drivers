/** @file
  eMMC (Exynos5420 dw_mmc channel 0) as a Storport disk for Windows RT 8.1
  on v1awifi: 29.8 GB, big enough for Windows itself.  Windows 8.1 has no
  sdport (that is Windows 10), so this is a Storport miniport presenting the
  eMMC as one SCSI disk, with the MMC side done by the very files UEFI uses
  (Exynos5420Pkg Drivers/SdCardDxe: MmcCore.c, Dwmci.c, built unchanged
  against MmcEnv.h here).  Modelled on the GT-N7000's microSD exynosstor.

  It registers as a *virtual* miniport (see exynosemmc: a physical one has
  to claim bus-master DMA and Storport then builds HAL DMA mappings that
  fail for this ACPI device).  The controller's own DMA (IDMAC) moves the
  data through an uncached, physically contiguous bounce buffer that the
  MMC layer allocates; Storport only hands over the mapped DataBuffer.

  - HwFindAdapter (PASSIVE): map dw_mmc0 (its access range if Storport
    hands one to a virtual miniport, otherwise the known address), reset
    the host and identify the eMMC (CMD0 .. user area, 8-bit, 20 MHz), then
    switch it to the fastest bus mode it passes (HS200, else HS52).
  - HwStartIo (DISPATCH) only queues requests; a Storport worker runs them
    at PASSIVE level, polling the controller like UEFI does, and completes
    them.  No interrupt.

  UEFI leaves clocks (sclk_dwmci0 = CPLL / 1, CIU = 666 MHz / 4), pins and
  power set up.

  ACPI: _HID EXYN5007 (EMMC), dw_mmc0 at 0x12200000 (+ the eMMCP security
  unit at +0x1000).

  With LOAD_TEST it bugchecks 0x454D4D43 ("EMMC") in HwFindAdapter with
  P1 = register base, P2 = CIU clock (Hz), P3 = access range count.
  Bring-up failures (step STOR_STEP_*, status) are logged and fail the
  adapter.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#define OSLOG_GLOBAL
#include "../../uefi/Exynos5420Pkg/Drivers/SdCardDxe/MmcHost.h"
#include <storport.h>

#ifndef LOAD_TEST
#define LOAD_TEST 0
#endif
#define STOR_BUGCHECK  0x454D4D43

enum {
  STOR_STEP_INITIALIZE = 1,
  STOR_STEP_NO_RANGE,
  STOR_STEP_MAP,
  STOR_STEP_HOST,
  STOR_STEP_CARD,
  STOR_STEP_WORKER,
};

#define DWMCI0_BASE          0x12200000u
#define DWMCI0_SIZE          0x2000u          // registers + eMMCP (SMU) at +0x1000
#define DWMCI_CIU_HZ         166500000u       // sclk_dwmci0 666 MHz / CLKSEL divider 4
#define DWMCI_SDR_TIMING     0x03040000u      // pdata sdr_timing, as UEFI
#define MAX_TRANSFER_BLOCKS  256              // 128 KiB per request

typedef struct {
  LIST_ENTRY           Link;
  PSCSI_REQUEST_BLOCK  Srb;
} SRB_EXT, *PSRB_EXT;

typedef struct {
  PUCHAR      Base;
  ULONG       Size;
  MMC_HOST    Host;
  PVOID       Worker;
  KSPIN_LOCK  Lock;
  LIST_ENTRY  Pending;
  BOOLEAN     WorkerQueued;
  ULONG       Commands;       // SCSI commands seen (logging)
  ULONG       Transfers;      // reads/writes done (logging)
  ULONG       Recoveries;     // card re-initialisations after a dropout
  LONGLONG    LastRecovery;   // KeQueryInterruptTime() of the last one
} DEVICE_EXTENSION, *PDEVICE_EXTENSION;

static VOID
BringUpFailed (ULONG Step, ULONG_PTR Status)
{
  OsLog ("exynosemmc: bring-up step %lu failed: 0x%08Ix", Step, Status);
}

//
// ---------------------------------------------------------------------------
// SCSI commands, at PASSIVE level in the worker.
// ---------------------------------------------------------------------------
//

static UCHAR
CopyOut (PSCSI_REQUEST_BLOCK Srb, const VOID *Data, ULONG Length)
{
  Length = min (Length, Srb->DataTransferLength);
  if ((Length != 0) && (Srb->DataBuffer == NULL)) {
    return SRB_STATUS_ERROR;
  }
  RtlCopyMemory (Srb->DataBuffer, Data, Length);
  Srb->DataTransferLength = Length;
  return SRB_STATUS_SUCCESS;
}

static UCHAR
ScsiInquiry (PDEVICE_EXTENSION Dev, PSCSI_REQUEST_BLOCK Srb)
{
  INQUIRYDATA  Inquiry;
  UCHAR        Pages[6];

  UNREFERENCED_PARAMETER (Dev);

  if ((Srb->Cdb[1] & 1) != 0) {
    // EVPD: only the supported-pages page.
    if (Srb->Cdb[2] != 0) {
      return SRB_STATUS_INVALID_REQUEST;
    }
    RtlZeroMemory (Pages, sizeof (Pages));
    Pages[3] = 1;                    // one page: 0x00
    return CopyOut (Srb, Pages, 5);
  }

  RtlZeroMemory (&Inquiry, sizeof (Inquiry));
  Inquiry.DeviceType          = DIRECT_ACCESS_DEVICE;
  Inquiry.Versions            = 5;
  Inquiry.ResponseDataFormat  = 2;
  Inquiry.AdditionalLength    = sizeof (Inquiry) - 5;
  RtlCopyMemory (Inquiry.VendorId, "Samsung ", 8);
  RtlCopyMemory (Inquiry.ProductId, "SM-P900 eMMC    ", 16);
  RtlCopyMemory (Inquiry.ProductRevisionLevel, "1.0 ", 4);
  return CopyOut (Srb, &Inquiry, sizeof (Inquiry));
}

static UCHAR
ScsiReadCapacity (PDEVICE_EXTENSION Dev, PSCSI_REQUEST_BLOCK Srb, BOOLEAN Long)
{
  UCHAR   Data[32];
  UINT64  Last;
  ULONG   Index;

  Last = Dev->Host.Card.BlockCount - 1;
  RtlZeroMemory (Data, sizeof (Data));
  if (Long) {
    for (Index = 0; Index < 8; Index++) {
      Data[Index] = (UCHAR)(Last >> (56 - 8 * Index));
    }
    Data[10] = BLOCK_SIZE >> 8;        // bytes 8-11: block length, big endian
    return CopyOut (Srb, Data, 32);
  }

  if (Last > 0xFFFFFFFF) {
    Last = 0xFFFFFFFF;                 // tells the host to use READ CAPACITY 16
  }
  Data[0] = (UCHAR)(Last >> 24);
  Data[1] = (UCHAR)(Last >> 16);
  Data[2] = (UCHAR)(Last >> 8);
  Data[3] = (UCHAR)Last;
  Data[6] = BLOCK_SIZE >> 8;
  return CopyOut (Srb, Data, 8);
}

/**
  Mode parameter header only (no pages): not write-protected, no cache page.
**/
static UCHAR
ScsiModeSense (PSCSI_REQUEST_BLOCK Srb, BOOLEAN Ten)
{
  UCHAR  Header[8];

  RtlZeroMemory (Header, sizeof (Header));
  if (Ten) {
    Header[1] = 6;                     // mode data length after this field
    return CopyOut (Srb, Header, 8);
  }
  Header[0] = 3;
  return CopyOut (Srb, Header, 4);
}

/**
  The controller's clocks stay as UEFI left them (sclk_dwmci0 = CPLL / 1 and
  CLKSEL / 4 = 166.5 MHz CIU): HS52 and HS200 need no CMU change, the MMC
  core only wants the CIU rate back.  HS400 would (not supported here).
**/
static EFI_STATUS
EFIAPI
SourceClock (MMC_HOST *Host, MMC_SPEED Speed, UINT32 ClkSel, UINT32 *CiuHz)
{
  UNREFERENCED_PARAMETER (Host);
  UNREFERENCED_PARAMETER (ClkSel);

  if ((Speed != MmcSpeedDefault) && (Speed != MmcSpeedHs) && (Speed != MmcSpeedHs200)) {
    return EFI_UNSUPPORTED;
  }
  *CiuHz = DWMCI_CIU_HZ;
  return EFI_SUCCESS;
}

#define HS200_SAMPLE_PHASE  3        // middle of the passing window 0..6 found in UEFI (EmmcTest, 8 phases x 8 drives)

/**
  Fastest bus mode the card passes: HS200 (8-bit SDR at 166.5 MHz, sample
  phase from the UEFI scan), else HS52.  The shared MmcCore proves each by
  reading LBA 0 and EXT_CSD and resets the card back to the default speed
  when a mode fails.  Measured in UEFI (EmmcTest): default 14 / 11 MB/s
  read / write, HS52 31 / 18.6, HS200 80 / 26; 3000 random read/write
  operations verified in HS52 and HS200.
**/
static VOID
SelectHighSpeed (PDEVICE_EXTENSION Dev)
{
  EFI_STATUS  Status;

#ifndef EMMC_HIGH_SPEED
  UNREFERENCED_PARAMETER (Dev);
  UNREFERENCED_PARAMETER (Status);
  return;                           // slow mode: stay at the default 20 MHz
#endif
  Dev->Host.SetSourceClock = SourceClock;
  Dev->Host.SamplePhase    = HS200_SAMPLE_PHASE;
  Status = MmcSetSpeed (&Dev->Host, MmcSpeedHs200);
  if (EFI_ERROR (Status)) {
    OsLog ("exynosemmc: HS200 failed - 0x%08Ix, trying HS52", Status);
    Status = MmcSetSpeed (&Dev->Host, MmcSpeedHs);
  }
  OsLog ("exynosemmc: bus speed mode %lu (0 default, 1 HS52, 4 HS200) - 0x%08Ix, card clock %lu Hz, CIU %lu Hz",
         (ULONG)Dev->Host.Speed, Status, Dev->Host.ClockHz, Dev->Host.MaxClockHz);
}

/**
  The card sometimes stops answering mid-session (every command, CMD13
  included, times out: it has lost its RCA, as after a reset).  Bring it
  back like at boot: controller FIFO / DMA reset, 400 kHz 1-bit, then the
  whole identification sequence.  At most every 2 s.
**/
static BOOLEAN
RecoverCard (PDEVICE_EXTENSION Dev)
{
  LONGLONG    Now;
  EFI_STATUS  Status;

  Now = (LONGLONG)KeQueryInterruptTime ();
  if ((Dev->LastRecovery != 0) && (Now - Dev->LastRecovery < 20000000LL)) {
    return FALSE;
  }
  Dev->LastRecovery = Now;
  Dev->Recoveries++;

  WRITE_REGISTER_ULONG ((PULONG)(Dev->Base + 0x00), READ_REGISTER_ULONG ((PULONG)(Dev->Base + 0x00)) | 0x6);
  KeStallExecutionProcessor (100);
  Dev->Host.SetBusWidth (&Dev->Host, 1);
  Dev->Host.SetClock (&Dev->Host, 400000);
  Status = MmcInitCard (&Dev->Host, TRUE);
  OsLog ("exynosemmc: card stopped answering, re-init #%lu - 0x%08Ix (%lu-bit)", Dev->Recoveries, Status,
         Dev->Host.Card.BusWidth);
  if (!EFI_ERROR (Status)) {
    SelectHighSpeed (Dev);          // the reset put the card back to the default speed
  }
  return !EFI_ERROR (Status);
}

static UCHAR
ScsiReadWrite (PDEVICE_EXTENSION Dev, PSCSI_REQUEST_BLOCK Srb, BOOLEAN Write, UINT64 Lba, ULONG Blocks)
{
  EFI_STATUS  Status;

  if (Blocks == 0) {
    return SRB_STATUS_SUCCESS;
  }
  if ((Lba + Blocks > Dev->Host.Card.BlockCount) ||
      (Srb->DataTransferLength < Blocks * BLOCK_SIZE) || (Srb->DataBuffer == NULL)) {
    return SRB_STATUS_INVALID_REQUEST;
  }

  Status = Write ? MmcWriteBlocks (&Dev->Host, Lba, Blocks, Srb->DataBuffer)
                 : MmcReadBlocks (&Dev->Host, Lba, Blocks, Srb->DataBuffer);
  if ((Status == EFI_TIMEOUT) && RecoverCard (Dev)) {
    Status = Write ? MmcWriteBlocks (&Dev->Host, Lba, Blocks, Srb->DataBuffer)
                   : MmcReadBlocks (&Dev->Host, Lba, Blocks, Srb->DataBuffer);
    OsLog ("exynosemmc: retry after re-init: %s LBA %I64u x%lu - 0x%08Ix", Write ? "write" : "read", Lba, Blocks, Status);
  }
  if (EFI_ERROR (Status)) {
    OsLog ("exynosemmc: %s LBA %I64u x%lu failed 0x%08Ix", Write ? "write" : "read", Lba, Blocks, Status);
    return SRB_STATUS_ERROR;
  }
  Srb->DataTransferLength = Blocks * BLOCK_SIZE;
  return SRB_STATUS_SUCCESS;
}

static UINT64
Be (const UCHAR *P, ULONG Bytes)
{
  UINT64  Value;

  for (Value = 0; Bytes > 0; Bytes--, P++) {
    Value = (Value << 8) | *P;
  }
  return Value;
}

static UCHAR
ExecuteScsi (PDEVICE_EXTENSION Dev, PSCSI_REQUEST_BLOCK Srb)
{
  const UCHAR  *Cdb = Srb->Cdb;

  switch (Cdb[0]) {
    case SCSIOP_TEST_UNIT_READY:
    case SCSIOP_VERIFY:
    case SCSIOP_START_STOP_UNIT:
    case SCSIOP_MEDIUM_REMOVAL:
    case SCSIOP_SYNCHRONIZE_CACHE:
    case SCSIOP_SYNCHRONIZE_CACHE16:
      Srb->DataTransferLength = 0;
      return SRB_STATUS_SUCCESS;

    case SCSIOP_REQUEST_SENSE:
    {
      SENSE_DATA  Sense;

      RtlZeroMemory (&Sense, sizeof (Sense));
      Sense.ErrorCode = SCSI_SENSE_ERRORCODE_FIXED_CURRENT;
      Sense.AdditionalSenseLength = sizeof (Sense) - 8;
      return CopyOut (Srb, &Sense, sizeof (Sense));
    }

    case SCSIOP_INQUIRY:
      return ScsiInquiry (Dev, Srb);

    case SCSIOP_READ_CAPACITY:
      return ScsiReadCapacity (Dev, Srb, FALSE);

    case SCSIOP_SERVICE_ACTION_IN16:
      if ((Cdb[1] & 0x1F) == SERVICE_ACTION_READ_CAPACITY16) {
        return ScsiReadCapacity (Dev, Srb, TRUE);
      }
      return SRB_STATUS_INVALID_REQUEST;

    case SCSIOP_MODE_SENSE:
      return ScsiModeSense (Srb, FALSE);

    case SCSIOP_MODE_SENSE10:
      return ScsiModeSense (Srb, TRUE);

    case SCSIOP_READ:
    case SCSIOP_WRITE:
      return ScsiReadWrite (Dev, Srb, Cdb[0] == SCSIOP_WRITE, Be (&Cdb[2], 4), (ULONG)Be (&Cdb[7], 2));

    case SCSIOP_READ16:
    case SCSIOP_WRITE16:
      return ScsiReadWrite (Dev, Srb, Cdb[0] == SCSIOP_WRITE16, Be (&Cdb[2], 8), (ULONG)Be (&Cdb[10], 4));

    default:
      return SRB_STATUS_INVALID_REQUEST;
  }
}

/**
  Storport worker (PASSIVE): run queued requests until the queue is empty.
**/
static VOID
Worker (PVOID DeviceExtension, PVOID Context, PVOID WorkerHandle)
{
  PDEVICE_EXTENSION    Dev;
  PLIST_ENTRY          Entry;
  PSCSI_REQUEST_BLOCK  Srb;
  KIRQL                Irql;

  UNREFERENCED_PARAMETER (Context);
  UNREFERENCED_PARAMETER (WorkerHandle);

  Dev = DeviceExtension;
  for ( ; ; ) {
    KeAcquireSpinLock (&Dev->Lock, &Irql);
    if (IsListEmpty (&Dev->Pending)) {
      Dev->WorkerQueued = FALSE;
      KeReleaseSpinLock (&Dev->Lock, Irql);
      return;
    }
    Entry = RemoveHeadList (&Dev->Pending);
    KeReleaseSpinLock (&Dev->Lock, Irql);

    Srb = CONTAINING_RECORD (Entry, SRB_EXT, Link)->Srb;
    Srb->SrbStatus = ExecuteScsi (Dev, Srb);
    Srb->ScsiStatus = (Srb->SrbStatus == SRB_STATUS_SUCCESS) ? SCSISTAT_GOOD : SCSISTAT_CHECK_CONDITION;

    //
    // The first commands (the disk class bring-up), every failure, and a
    // sign of life every 500 reads/writes.
    //
    Dev->Commands++;
    if ((Dev->Commands <= 60) || (Srb->SrbStatus != SRB_STATUS_SUCCESS)) {
      OsLog ("exynosemmc: SCSI 0x%02x len %lu -> srb 0x%02x (cmd %lu)", Srb->Cdb[0],
             Srb->DataTransferLength, Srb->SrbStatus, Dev->Commands);
    }
    if ((Srb->Cdb[0] == SCSIOP_READ) || (Srb->Cdb[0] == SCSIOP_WRITE) ||
        (Srb->Cdb[0] == SCSIOP_READ16) || (Srb->Cdb[0] == SCSIOP_WRITE16)) {
      if ((Dev->Transfers++ % 500) == 0) {
        OsLog ("exynosemmc: %lu transfers, last 0x%02x LBA %I64u", Dev->Transfers, Srb->Cdb[0],
               (Srb->Cdb[0] == SCSIOP_READ || Srb->Cdb[0] == SCSIOP_WRITE) ? Be (&Srb->Cdb[2], 4) : Be (&Srb->Cdb[2], 8));
      }
    }
    StorPortNotification (RequestComplete, Dev, Srb);
  }
}

//
// ---------------------------------------------------------------------------
// Storport callbacks.
// ---------------------------------------------------------------------------
//

static BOOLEAN
HwStartIo (PVOID DeviceExtension, PSCSI_REQUEST_BLOCK Srb)
{
  PDEVICE_EXTENSION  Dev;
  PSRB_EXT           Ext;
  KIRQL              Irql;
  BOOLEAN            Kick;

  Dev = DeviceExtension;

  switch (Srb->Function) {
    case SRB_FUNCTION_EXECUTE_SCSI:
      if ((Srb->PathId != 0) || (Srb->TargetId != 0) || (Srb->Lun != 0)) {
        Srb->SrbStatus = SRB_STATUS_SELECTION_TIMEOUT;
        break;
      }
      Ext      = Srb->SrbExtension;
      Ext->Srb = Srb;
      KeAcquireSpinLock (&Dev->Lock, &Irql);
      InsertTailList (&Dev->Pending, &Ext->Link);
      Kick = !Dev->WorkerQueued;
      Dev->WorkerQueued = TRUE;
      KeReleaseSpinLock (&Dev->Lock, Irql);
      if (Kick) {
        ULONG  QueueStatus = STOR_STATUS_UNSUCCESSFUL;

        if (Dev->Worker != NULL) {
          QueueStatus = StorPortQueueWorkItem (Dev, Worker, Dev->Worker, NULL);
        }
        if (QueueStatus != STOR_STATUS_SUCCESS) {
          //
          // No worker (yet): run the request here, at DISPATCH level.
          //
          if (Dev->Commands < 60) {
            OsLog ("exynosemmc: worker %p unavailable (0x%08lx), running inline", Dev->Worker, QueueStatus);
          }
          Worker (Dev, NULL, NULL);
        }
      }
      return TRUE;

    case SRB_FUNCTION_FLUSH:
    case SRB_FUNCTION_SHUTDOWN:
    case SRB_FUNCTION_RESET_BUS:
    case SRB_FUNCTION_RESET_DEVICE:
    case SRB_FUNCTION_RESET_LOGICAL_UNIT:
    case SRB_FUNCTION_PNP:
    case SRB_FUNCTION_POWER:
      Srb->SrbStatus = SRB_STATUS_SUCCESS;
      break;

    default:
      Srb->SrbStatus = SRB_STATUS_INVALID_REQUEST;
      break;
  }

  if (Srb->Function == SRB_FUNCTION_PNP) {
    OsLog ("exynosemmc: PnP action %d", ((PSCSI_PNP_REQUEST_BLOCK)Srb)->PnPAction);
  } else if (Dev->Commands < 60) {
    OsLog ("exynosemmc: SRB function 0x%02x -> 0x%02x", Srb->Function, Srb->SrbStatus);
  }
  StorPortNotification (RequestComplete, Dev, Srb);
  return TRUE;
}

static BOOLEAN
HwResetBus (PVOID DeviceExtension, ULONG PathId)
{
  UNREFERENCED_PARAMETER (DeviceExtension);
  UNREFERENCED_PARAMETER (PathId);
  return TRUE;
}

static BOOLEAN
HwPassiveInitialize (PVOID DeviceExtension)
{
  PDEVICE_EXTENSION  Dev;
  ULONG              Status;

  Dev    = DeviceExtension;
  Status = StorPortInitializeWorker (Dev, &Dev->Worker);
  OsLog ("exynosemmc: passive init, worker %p (0x%08lx)", Dev->Worker, Status);
  if (Status != STOR_STATUS_SUCCESS) {
    BringUpFailed (STOR_STEP_WORKER, Status);
    Dev->Worker = NULL;
  }
  return TRUE;
}

static BOOLEAN
HwInitialize (PVOID DeviceExtension)
{
  BOOLEAN  Enabled;

  Enabled = StorPortEnablePassiveInitialization (DeviceExtension, HwPassiveInitialize);
  OsLog ("exynosemmc: HwInitialize, passive init %s", Enabled ? "enabled" : "not available");
  return TRUE;
}

static SCSI_ADAPTER_CONTROL_STATUS
HwAdapterControl (PVOID DeviceExtension, SCSI_ADAPTER_CONTROL_TYPE ControlType, PVOID Parameters)
{
  PSCSI_SUPPORTED_CONTROL_TYPE_LIST  List;

  UNREFERENCED_PARAMETER (DeviceExtension);

  switch (ControlType) {
    case ScsiQuerySupportedControlTypes:
      List = Parameters;
      if (List->MaxControlType > ScsiStopAdapter) {
        List->SupportedTypeList[ScsiStopAdapter] = TRUE;
      }
      if (List->MaxControlType > ScsiRestartAdapter) {
        List->SupportedTypeList[ScsiRestartAdapter] = TRUE;
      }
      List->SupportedTypeList[ScsiQuerySupportedControlTypes] = TRUE;
      return ScsiAdapterControlSuccess;
    case ScsiStopAdapter:
    case ScsiRestartAdapter:
      return ScsiAdapterControlSuccess;
    default:
      return ScsiAdapterControlUnsuccessful;
  }
}

static ULONG
HwFindAdapter (PVOID DeviceExtension, PVOID HwContext, PVOID BusInformation, PVOID LowerDevice,
               PCHAR ArgumentString, PPORT_CONFIGURATION_INFORMATION ConfigInfo, PBOOLEAN Again)
{
  PDEVICE_EXTENSION       Dev;
  EFI_STATUS              Status;

  UNREFERENCED_PARAMETER (HwContext);
  UNREFERENCED_PARAMETER (BusInformation);
  UNREFERENCED_PARAMETER (ArgumentString);
  UNREFERENCED_PARAMETER (LowerDevice);
  UNREFERENCED_PARAMETER (Again);

  Dev = DeviceExtension;
  OsLog ("exynosemmc: HwFindAdapter, interface %d, %lu access ranges",
         ConfigInfo->AdapterInterfaceType, ConfigInfo->NumberOfAccessRanges);
  KeInitializeSpinLock (&Dev->Lock);
  InitializeListHead (&Dev->Pending);

  //
  // A virtual miniport may or may not see the ACPI memory resource; the
  // register block is known either way.
  //
  {
    PHYSICAL_ADDRESS  Phys;
    ULONG             Size;

    Phys.QuadPart = DWMCI0_BASE;
    Size          = DWMCI0_SIZE;
    if ((ConfigInfo->NumberOfAccessRanges >= 1) && (ConfigInfo->AccessRanges != NULL) &&
        ((*ConfigInfo->AccessRanges)[0].RangeLength >= DWMCI0_SIZE)) {
      Phys = (*ConfigInfo->AccessRanges)[0].RangeStart;
      Size = (*ConfigInfo->AccessRanges)[0].RangeLength;
    }
    Dev->Base = MmMapIoSpace (Phys, Size, MmNonCached);
    Dev->Size = Size;
    if (Dev->Base == NULL) {
      BringUpFailed (STOR_STEP_MAP, Phys.LowPart);
      return SP_RETURN_ERROR;
    }
  }
  OsLog ("exynosemmc: dw_mmc0 mapped at %p, CIU %lu Hz", Dev->Base, DWMCI_CIU_HZ);

#if LOAD_TEST
  KeBugCheckEx (STOR_BUGCHECK, DWMCI0_BASE, DWMCI_CIU_HZ, ConfigInfo->NumberOfAccessRanges, 0);
#endif

  Status = DwmciInit (&Dev->Host, (UINT32)(ULONG_PTR)Dev->Base, DWMCI_CIU_HZ, 8,
                      DWMCI_SDR_TIMING, TRUE);
  if (EFI_ERROR (Status)) {
    BringUpFailed (STOR_STEP_HOST, Status);
    return SP_RETURN_ERROR;
  }
  Status = MmcInitCard (&Dev->Host, TRUE);
  if (EFI_ERROR (Status)) {
    BringUpFailed (STOR_STEP_CARD, Status);
    return SP_RETURN_ERROR;
  }
  OsLog ("exynosemmc: eMMC ready, %I64u blocks (%I64u MiB), %lu-bit",
         Dev->Host.Card.BlockCount, Dev->Host.Card.BlockCount >> 11, Dev->Host.Card.BusWidth);
  SelectHighSpeed (Dev);

  ConfigInfo->VirtualDevice               = TRUE;
  ConfigInfo->MaximumTransferLength       = MAX_TRANSFER_BLOCKS * BLOCK_SIZE;
  ConfigInfo->NumberOfPhysicalBreaks      = MAX_TRANSFER_BLOCKS + 1;
  ConfigInfo->AlignmentMask               = 3;
  ConfigInfo->NumberOfBuses               = 1;
  ConfigInfo->MaximumNumberOfTargets      = 1;
  ConfigInfo->MaximumNumberOfLogicalUnits = 1;
  ConfigInfo->InitiatorBusId[0]           = 1;
  //
  // Storport only starts adapters that claim scatter/gather bus-master DMA
  // (without it: StorRemoveDevice right after HwFindAdapter).  For a virtual
  // miniport it builds the S/G list itself from the request's MDL, with no
  // HAL DMA involved (that is what failed as a physical miniport); the data
  // still moves by PIO through the mapped DataBuffer.
  //
  ConfigInfo->ScatterGather               = TRUE;
  ConfigInfo->Master                      = TRUE;
  ConfigInfo->Dma32BitAddresses           = TRUE;
  ConfigInfo->CachesData                  = FALSE;
  ConfigInfo->MapBuffers                  = STOR_MAP_ALL_BUFFERS_INCLUDING_READ_WRITE;
  return SP_RETURN_FOUND;
}

//
// Virtual miniport callbacks with nothing to do here.
//
static VOID
HwFreeAdapterResources (PVOID DeviceExtension)
{
  PDEVICE_EXTENSION  Dev;

  Dev = DeviceExtension;
  OsLog ("exynosemmc: free adapter resources");
  if (Dev->Base != NULL) {
    MmUnmapIoSpace (Dev->Base, Dev->Size);
    Dev->Base = NULL;
  }
}

static VOID
HwProcessServiceRequest (PVOID DeviceExtension, PVOID Irp)
{
  ((PIRP)Irp)->IoStatus.Status      = STATUS_NOT_SUPPORTED;
  ((PIRP)Irp)->IoStatus.Information = 0;
  StorPortExtendedFunction (ExtFunctionCompleteServiceIrp, DeviceExtension, Irp);
}

static VOID
HwCompleteServiceIrp (PVOID DeviceExtension)
{
  UNREFERENCED_PARAMETER (DeviceExtension);
}

ULONG
DriverEntry (PVOID DriverObject, PVOID RegistryPath)
{
  static const INTERFACE_TYPE     Types[] = { Internal, PNPBus, ACPIBus };
  VIRTUAL_HW_INITIALIZATION_DATA  Init;
  ULONG                           Status;
  ULONG                           Result;
  ULONG                           Index;

  OsLogInit ();
  OsLog ("exynosemmc: DriverEntry");

  RtlZeroMemory (&Init, sizeof (Init));
  Init.HwInitializationDataSize = sizeof (Init);
  Init.HwInitialize             = HwInitialize;
  Init.HwStartIo                = HwStartIo;
  Init.HwFindAdapter            = HwFindAdapter;
  Init.HwResetBus               = HwResetBus;
  Init.HwAdapterControl         = HwAdapterControl;
  Init.HwFreeAdapterResources   = HwFreeAdapterResources;
  Init.HwProcessServiceRequest  = HwProcessServiceRequest;
  Init.HwCompleteServiceIrp     = HwCompleteServiceIrp;
  Init.DeviceExtensionSize      = sizeof (DEVICE_EXTENSION);
  Init.SrbExtensionSize         = sizeof (SRB_EXT);
  Init.MapBuffers               = STOR_MAP_ALL_BUFFERS_INCLUDING_READ_WRITE;
  Init.NeedPhysicalAddresses    = TRUE;
  Init.AutoRequestSense         = TRUE;
  Init.TaggedQueuing            = FALSE;
  Init.MultipleRequestPerLu     = FALSE;

  //
  // The ACPI device shows up as ACPIBus here; register for the other bus
  // types an ACPI device can present too (INF Parameters\PnpInterface).
  //
  Result = (ULONG)STATUS_UNSUCCESSFUL;
  for (Index = 0; Index < ARRAYSIZE (Types); Index++) {
    Init.AdapterInterfaceType = Types[Index];
    Status = StorPortInitialize (DriverObject, RegistryPath, (PHW_INITIALIZATION_DATA)&Init, NULL);
    OsLog ("exynosemmc: StorPortInitialize (virtual, interface %d) 0x%08lx", Types[Index], Status);
    if (Status == STATUS_SUCCESS) {
      Result = STATUS_SUCCESS;
    }
  }
  if (Result != STATUS_SUCCESS) {
    BringUpFailed (STOR_STEP_INITIALIZE, Status);
  }
  return Result;
}
