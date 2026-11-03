/** @file
  Broadcom UART Bluetooth (v1awifi: BCM4350-family controller on UART0) as
  a serial HCI bus driver for Windows RT 8.1 (KMDF 1.13), the shape of
  Microsoft's Serial HCI bus sample: this driver owns the transport and
  enumerates one child, MS_BTHX_BTHMINI, which the inbox bth.inf installs
  BthMini (and with it the Windows Bluetooth stack) on.  BthMini talks
  BthX (bthxddi.h) to the child: GET/SET_VERSION, QUERY_CAPABILITIES,
  WRITE_HCI, READ_HCI.

  Transport: exynosuart (SerCx2) through the resource hub (UartSerialBus
  in the DSDT, 115200 8N1 RTS/CTS).  H4 framing: one packet-type byte
  (1 command, 2 ACL, 4 event) before each HCI packet.

  Bring-up, as Broadcom's libbt-vendor (hw_config_cback) does it:
    BT_EN low / high (100 ms, board-bluetooth-bcm4339.c), BT_WAKE high,
    HCI_Reset, Read_Local_Name (picks the patch file), baud rate up
    (vendor 0xFC18), Download_Minidriver (0xFC2E), the .hcd records (each
    an HCI command, the last one Launch_RAM 0xFC4E), 200 ms, back at
    115200, HCI_Reset, baud rate up again.  Patch files (Samsung's
    /vendor/firmware, not in the source tree): bcm4350_V0301.0591.hcd,
    bcm4335_V0105.0541.hcd in System32\drivers.

  The read side is a chain of exact-length UART reads (type, header,
  payload), each started from the previous one's completion: no polling.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#define OSLOG_GLOBAL
#include <ExynosEnv.h>
#include <wdf.h>
#include <ntddser.h>
#include <gpio.h>
#define RESHUB_USE_HELPER_ROUTINES
#include <reshub.h>
#include <initguid.h>
#include <bthxddi.h>

#define BT_TAG             'tBxE'
#define TARGET_BAUD        3000000u
#define INIT_BAUD          115200u
#define CMD_TIMEOUT_MS     2000
#define MAX_ACL            1021
#define MAX_PACKET         (4 + MAX_ACL)
#define RING_PACKETS       32
#define H4_COMMAND         0x01
#define H4_ACL             0x02
#define H4_EVENT           0x04
#define EVT_COMMAND_COMPLETE 0x0E
#define EVT_COMMAND_STATUS   0x0F

//
// The child: device ID in ENUMERATOR\DEVICE form (PnP builds the instance
// path from it), hardware / compatible ID what bth.inf's BthMini matches.
//
#define CHILD_DEVICE_ID    L"EXYNOSBT\\MS_BTHX_BTHMINI"
#define BTHMINI_ID         L"MS_BTHX_BTHMINI"

typedef struct {
  UCHAR   Type;
  USHORT  Length;
  UCHAR   Data[MAX_PACKET];
} PACKET;

typedef enum {
  RxType,
  RxHeader,
  RxPayload
} RX_STATE;

typedef struct {
  WDFDEVICE      Device;
  WDFDEVICE      Pdo;
  WDFIOTARGET    Uart;
  WDFIOTARGET    GpioEn;
  WDFIOTARGET    GpioWake;
  WDFIOTARGET    GpioHostWake;     // input, held open for its PullNone
  LARGE_INTEGER  UartId;
  LARGE_INTEGER  GpioId[3];
  ULONG          GpioCount;
  BOOLEAN        Ready;            // firmware up, child enumerated
  CHAR           Name[32];         // controller name (Read_Local_Name)
  //
  // Read pump.
  //
  WDFREQUEST     RxRequest;
  WDFMEMORY      RxMemory;
  RX_STATE       RxState;
  PACKET         RxPacket;
  ULONG          RxWant;
  BOOLEAN        RxStop;
  KEVENT         RxStopped;
  //
  // Packets waiting for a BthMini read, per kind (events / ACL).
  //
  WDFSPINLOCK    Lock;
  WDFQUEUE       ReadEvents;       // pended READ_HCI, on the child
  WDFQUEUE       ReadAcl;
  PACKET        *Ring;
  ULONG          RingHead;
  ULONG          RingCount;
  ULONG          Dropped;
  ULONG          RxPackets;
  ULONG          TxPackets;
  ULONG          Logs;
  ULONG          TxDoneLogs;
  ULONG          RxStepLogs;
} BT_CONTEXT, *PBT_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (BT_CONTEXT, GetBt)

typedef struct {
  PBT_CONTEXT  Bt;
} PDO_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (PDO_CONTEXT, GetPdo)

typedef struct {
  WDFREQUEST  Original;            // BthMini's WRITE_HCI
  ULONG       DataLen;             // packet bytes (without the H4 type byte)
} WRITE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (WRITE_CONTEXT, GetWrite)

DRIVER_INITIALIZE                         DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD                 EvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE           EvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE           EvtReleaseHardware;
EVT_WDF_DEVICE_D0_EXIT                    EvtD0Exit;
EVT_WDF_DEVICE_D0_ENTRY                   EvtD0Entry;
EVT_WDF_OBJECT_CONTEXT_CLEANUP            EvtCleanup;
EVT_WDF_IO_QUEUE_IO_INTERNAL_DEVICE_CONTROL EvtPdoInternalIoctl;
EVT_WDF_REQUEST_COMPLETION_ROUTINE        RxComplete;
EVT_WDF_REQUEST_COMPLETION_ROUTINE        TxComplete;

static VOID
SleepMs (ULONG Ms)
{
  LARGE_INTEGER  Delay;

  Delay.QuadPart = -10000LL * Ms;
  KeDelayExecutionThread (KernelMode, FALSE, &Delay);
}

//
// ---------------------------------------------------------------------------
// Targets: UART and GPIO through the resource hub.
// ---------------------------------------------------------------------------
//

static NTSTATUS
OpenTarget (WDFDEVICE Device, LARGE_INTEGER Id, ACCESS_MASK Access, WDFIOTARGET *Target)
{
  WDF_IO_TARGET_OPEN_PARAMS  Open;
  NTSTATUS                   Status;

  DECLARE_UNICODE_STRING_SIZE (Path, RESOURCE_HUB_PATH_SIZE);

  Status = RESOURCE_HUB_CREATE_PATH_FROM_ID (&Path, Id.LowPart, Id.HighPart);
  if (NT_SUCCESS (Status)) {
    Status = WdfIoTargetCreate (Device, WDF_NO_OBJECT_ATTRIBUTES, Target);
  }
  if (NT_SUCCESS (Status)) {
    WDF_IO_TARGET_OPEN_PARAMS_INIT_OPEN_BY_NAME (&Open, &Path, Access);
    Status = WdfIoTargetOpen (*Target, &Open);
    if (!NT_SUCCESS (Status)) {
      WdfObjectDelete (*Target);
      *Target = NULL;
    }
  }
  return Status;
}

static NTSTATUS
GpioSet (WDFIOTARGET Target, UCHAR Value)
{
  WDF_MEMORY_DESCRIPTOR  In;

  if (Target == NULL) {
    return STATUS_DEVICE_NOT_CONNECTED;
  }
  WDF_MEMORY_DESCRIPTOR_INIT_BUFFER (&In, &Value, sizeof (Value));
  return WdfIoTargetSendIoctlSynchronously (Target, NULL, IOCTL_GPIO_WRITE_PINS, &In, NULL, NULL, NULL);
}

static NTSTATUS
UartIoctl (PBT_CONTEXT Bt, ULONG Code, PVOID In, ULONG InSize)
{
  WDF_MEMORY_DESCRIPTOR  Desc;

  WDF_MEMORY_DESCRIPTOR_INIT_BUFFER (&Desc, In, InSize);
  return WdfIoTargetSendIoctlSynchronously (Bt->Uart, NULL, Code, (In != NULL) ? &Desc : NULL, NULL, NULL, NULL);
}

static NTSTATUS
UartQuery (PBT_CONTEXT Bt, ULONG Code, PVOID Out, ULONG OutSize)
{
  WDF_MEMORY_DESCRIPTOR  Desc;

  WDF_MEMORY_DESCRIPTOR_INIT_BUFFER (&Desc, Out, OutSize);
  return WdfIoTargetSendIoctlSynchronously (Bt->Uart, NULL, Code, NULL, &Desc, NULL, NULL);
}

/**
  After a missing answer: CTS (the controller's flow control: inactive
  means it is not up) and the UART's queues (bytes still waiting to go out
  / received but unread).
**/
static VOID
LogUartState (PBT_CONTEXT Bt)
{
  ULONG          Modem = 0;
  SERIAL_STATUS  Comm;

  RtlZeroMemory (&Comm, sizeof (Comm));
  UartQuery (Bt, IOCTL_SERIAL_GET_MODEMSTATUS, &Modem, sizeof (Modem));
  UartQuery (Bt, IOCTL_SERIAL_GET_COMMSTATUS, &Comm, sizeof (Comm));
  OsLog ("exynosbt: UART: CTS %s, %lu bytes to send, %lu received", (Modem & SERIAL_CTS_STATE) ? "active" : "inactive",
         Comm.AmountInOutQueue, Comm.AmountInInQueue);
}

/**
  Diagnostics: the pins as the SoC sees them (GPG0 BT_EN, GPX3 BT_WAKE,
  GPX2 HOST_WAKE, GPA0 UART0: CON / DAT / PUD), read directly.
**/
static VOID
LogPins (VOID)
{
  static CONST struct { ULONG Phys; PCSTR Name; } Banks[] = {
    { 0x14000080, "GPG0 (BT_EN 7)" }, { 0x13400C60, "GPX3 (BT_WAKE 2)" }, { 0x13400C40, "GPX2 (HOST_WAKE 1)" },
    { 0x14010000, "GPA0 (UART0 RX TX CTS RTS)" },
  };
  PHYSICAL_ADDRESS  Pa;
  PULONG            Va;
  ULONG             Index;

  for (Index = 0; Index < ARRAYSIZE (Banks); Index++) {
    Pa.QuadPart = Banks[Index].Phys & ~0xFFFu;
    Va = MmMapIoSpace (Pa, 0x1000, MmNonCached);
    if (Va != NULL) {
      PULONG  R = (PULONG)((PUCHAR)Va + (Banks[Index].Phys & 0xFFF));

      OsLog ("exynosbt: %s CON 0x%08lx DAT 0x%02lx PUD 0x%04lx", Banks[Index].Name, READ_REGISTER_ULONG (R),
             READ_REGISTER_ULONG (R + 1), READ_REGISTER_ULONG (R + 2));
      MmUnmapIoSpace (Va, 0x1000);
    }
  }
}

static NTSTATUS
UartBaud (PBT_CONTEXT Bt, ULONG Baud)
{
  SERIAL_BAUD_RATE  Rate;

  Rate.BaudRate = Baud;
  return UartIoctl (Bt, IOCTL_SERIAL_SET_BAUD_RATE, &Rate, sizeof (Rate));
}

static NTSTATUS
UartWrite (PBT_CONTEXT Bt, PVOID Data, ULONG Length)
{
  WDF_MEMORY_DESCRIPTOR       Desc;
  WDF_REQUEST_SEND_OPTIONS    Options;
  ULONG_PTR                   Bytes = 0;
  NTSTATUS                    Status;

  WDF_MEMORY_DESCRIPTOR_INIT_BUFFER (&Desc, Data, Length);
  WDF_REQUEST_SEND_OPTIONS_INIT (&Options, WDF_REQUEST_SEND_OPTION_TIMEOUT);
  Options.Timeout = WDF_REL_TIMEOUT_IN_MS (CMD_TIMEOUT_MS);
  Status = WdfIoTargetSendWriteSynchronously (Bt->Uart, NULL, &Desc, NULL, &Options, &Bytes);
  if (NT_SUCCESS (Status) && (Bytes != Length)) {
    Status = STATUS_IO_TIMEOUT;
  }
  return Status;
}

static NTSTATUS
UartRead (PBT_CONTEXT Bt, PVOID Data, ULONG Length, ULONG TimeoutMs)
{
  WDF_MEMORY_DESCRIPTOR       Desc;
  WDF_REQUEST_SEND_OPTIONS    Options;
  ULONG_PTR                   Bytes = 0;
  NTSTATUS                    Status;

  WDF_MEMORY_DESCRIPTOR_INIT_BUFFER (&Desc, Data, Length);
  WDF_REQUEST_SEND_OPTIONS_INIT (&Options, WDF_REQUEST_SEND_OPTION_TIMEOUT);
  Options.Timeout = WDF_REL_TIMEOUT_IN_MS (TimeoutMs);
  Status = WdfIoTargetSendReadSynchronously (Bt->Uart, NULL, &Desc, NULL, &Options, &Bytes);
  if (NT_SUCCESS (Status) && (Bytes != Length)) {
    Status = STATUS_IO_TIMEOUT;
  }
  return Status;
}

//
// ---------------------------------------------------------------------------
// Bring-up: synchronous HCI commands.
// ---------------------------------------------------------------------------
//

/**
  Send an HCI command and wait for its Command Complete (other events are
  skipped).  Returns the event's parameters after the opcode (status
  first) in Out.
**/
static NTSTATUS
HciCommand (PBT_CONTEXT Bt, USHORT Opcode, CONST UCHAR *Params, UCHAR ParamLength, PUCHAR Out, ULONG OutSize,
            PULONG OutLength)
{
  UCHAR     Cmd[4 + 255];
  UCHAR     Hdr[3];
  UCHAR     Evt[255];
  ULONG     Tries;
  NTSTATUS  Status;

  Cmd[0] = H4_COMMAND;
  Cmd[1] = (UCHAR)Opcode;
  Cmd[2] = (UCHAR)(Opcode >> 8);
  Cmd[3] = ParamLength;
  if (ParamLength != 0) {
    RtlCopyMemory (Cmd + 4, Params, ParamLength);
  }
  Status = UartWrite (Bt, Cmd, 4 + ParamLength);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosbt: command 0x%04x: write failed 0x%08lx", Opcode, Status);
    return Status;
  }
  for (Tries = 0; Tries < 8; Tries++) {
    Status = UartRead (Bt, Hdr, 3, CMD_TIMEOUT_MS);
    if (!NT_SUCCESS (Status)) {
      OsLog ("exynosbt: command 0x%04x: no event (0x%08lx)", Opcode, Status);
      LogUartState (Bt);
      return Status;
    }
    if (Hdr[0] != H4_EVENT) {
      OsLog ("exynosbt: command 0x%04x: packet type 0x%02x, not an event", Opcode, Hdr[0]);
      return STATUS_DEVICE_PROTOCOL_ERROR;
    }
    if ((Hdr[2] != 0) && !NT_SUCCESS (Status = UartRead (Bt, Evt, Hdr[2], CMD_TIMEOUT_MS))) {
      return Status;
    }
    if ((Hdr[1] == EVT_COMMAND_COMPLETE) && (Hdr[2] >= 3) && ((Evt[1] | (Evt[2] << 8)) == Opcode)) {
      if (Out != NULL) {
        *OutLength = min ((ULONG)Hdr[2] - 3, OutSize);
        RtlCopyMemory (Out, Evt + 3, *OutLength);
      }
      return ((Hdr[2] > 3) && (Evt[3] != 0)) ? STATUS_DEVICE_PROTOCOL_ERROR : STATUS_SUCCESS;
    }
    if ((Hdr[1] == EVT_COMMAND_STATUS) && (Hdr[2] >= 4) && ((Evt[2] | (Evt[3] << 8)) == Opcode) && (Evt[0] != 0)) {
      OsLog ("exynosbt: command 0x%04x: status 0x%02x", Opcode, Evt[0]);
      return STATUS_DEVICE_PROTOCOL_ERROR;
    }
  }
  return STATUS_IO_TIMEOUT;
}

static NTSTATUS
HciReset (PBT_CONTEXT Bt)
{
  return HciCommand (Bt, 0x0C03, NULL, 0, NULL, 0, NULL);
}

/**
  Broadcom Update_Baudrate (0xFC18: two zero bytes, the rate), then the
  host side.
**/
static NTSTATUS
SetBaud (PBT_CONTEXT Bt, ULONG Baud)
{
  UCHAR     P[6] = { 0, 0, (UCHAR)Baud, (UCHAR)(Baud >> 8), (UCHAR)(Baud >> 16), (UCHAR)(Baud >> 24) };
  NTSTATUS  Status;

  Status = HciCommand (Bt, 0xFC18, P, sizeof (P), NULL, 0, NULL);
  if (NT_SUCCESS (Status)) {
    Status = UartBaud (Bt, Baud);
    SleepMs (20);
  }
  OsLog ("exynosbt: baud rate %lu - 0x%08lx", Baud, Status);
  return Status;
}

static NTSTATUS
LoadFile (PCWSTR Path, PUCHAR *Data, ULONG *Size)
{
  UNICODE_STRING             Name;
  OBJECT_ATTRIBUTES          Attr;
  IO_STATUS_BLOCK            Io;
  FILE_STANDARD_INFORMATION  Info;
  HANDLE                     File;
  NTSTATUS                   Status;

  *Data = NULL;
  RtlInitUnicodeString (&Name, Path);
  InitializeObjectAttributes (&Attr, &Name, OBJ_CASE_INSENSITIVE | OBJ_KERNEL_HANDLE, NULL, NULL);
  Status = ZwCreateFile (&File, GENERIC_READ | SYNCHRONIZE, &Attr, &Io, NULL, FILE_ATTRIBUTE_NORMAL, FILE_SHARE_READ,
                         FILE_OPEN, FILE_SYNCHRONOUS_IO_NONALERT | FILE_NON_DIRECTORY_FILE, NULL, 0);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  Status = ZwQueryInformationFile (File, &Io, &Info, sizeof (Info), FileStandardInformation);
  if (NT_SUCCESS (Status) && ((Info.EndOfFile.HighPart != 0) || (Info.EndOfFile.LowPart > 1024 * 1024))) {
    Status = STATUS_FILE_TOO_LARGE;
  }
  if (NT_SUCCESS (Status)) {
    *Size = Info.EndOfFile.LowPart;
    *Data = ExAllocatePoolWithTag (PagedPool, *Size, BT_TAG);
    Status = (*Data != NULL) ? ZwReadFile (File, NULL, NULL, NULL, &Io, *Data, *Size, NULL, NULL)
                             : STATUS_INSUFFICIENT_RESOURCES;
  }
  ZwClose (File);
  if (!NT_SUCCESS (Status) && (*Data != NULL)) {
    ExFreePoolWithTag (*Data, BT_TAG);
    *Data = NULL;
  }
  return Status;
}

/**
  Download_Minidriver, then every record of the .hcd (opcode, length,
  parameters: HCI commands, Launch_RAM last), then 200 ms for the patch to
  start (libbt's settlement delay).
**/
static NTSTATUS
DownloadPatch (PBT_CONTEXT Bt, PCWSTR Path)
{
  PUCHAR    Hcd;
  ULONG     Size;
  ULONG     Offset;
  ULONG     Records = 0;
  NTSTATUS  Status;

  Status = LoadFile (Path, &Hcd, &Size);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosbt: cannot read %ws: 0x%08lx", Path, Status);
    return Status;
  }
  Status = HciCommand (Bt, 0xFC2E, NULL, 0, NULL, 0, NULL);
  if (NT_SUCCESS (Status)) {
    SleepMs (50);
    for (Offset = 0; Offset + 3 <= Size; Offset += 3 + Hcd[Offset + 2]) {
      USHORT  Opcode = Hcd[Offset] | (Hcd[Offset + 1] << 8);
      UCHAR   Length = Hcd[Offset + 2];

      if (Offset + 3 + Length > Size) {
        break;
      }
      Status = HciCommand (Bt, Opcode, Hcd + Offset + 3, Length, NULL, 0, NULL);
      if (!NT_SUCCESS (Status)) {
        OsLog ("exynosbt: patch record %lu (opcode 0x%04x at %lu) failed 0x%08lx", Records, Opcode, Offset, Status);
        break;
      }
      Records++;
    }
  }
  ExFreePoolWithTag (Hcd, BT_TAG);
  OsLog ("exynosbt: patch %ws: %lu records - 0x%08lx", Path, Records, Status);
  SleepMs (200);
  return Status;
}

static NTSTATUS
BringUp (PBT_CONTEXT Bt)
{
  SERIAL_TIMEOUTS  Timeouts;
  UCHAR            Out[255];
  ULONG            OutLength;
  PCWSTR           Patch;
  NTSTATUS         Status;
  ULONG            Index;

  //
  // UART: the DSDT's settings (115200 8N1, RTS/CTS), reads wait for their
  // full length.
  //
  UartIoctl (Bt, IOCTL_SERIAL_APPLY_DEFAULT_CONFIGURATION, NULL, 0);
  RtlZeroMemory (&Timeouts, sizeof (Timeouts));
  UartIoctl (Bt, IOCTL_SERIAL_SET_TIMEOUTS, &Timeouts, sizeof (Timeouts));

  //
  // Power: BT_EN low / high, BT_WAKE high (the controller stays awake: no
  // low-power mode is configured); HCI_Reset.  Up to three power cycles,
  // the wait after BT_EN growing (100 ms as the board driver, then longer).
  //
  for (Index = 0; Index < 3; Index++) {
    GpioSet (Bt->GpioWake, 0);
    GpioSet (Bt->GpioEn, 0);
    SleepMs (100);
    UartBaud (Bt, INIT_BAUD);
    UartIoctl (Bt, IOCTL_SERIAL_PURGE, &(ULONG){ SERIAL_PURGE_RXCLEAR | SERIAL_PURGE_TXCLEAR }, sizeof (ULONG));
    Status = GpioSet (Bt->GpioEn, 1);
    GpioSet (Bt->GpioWake, 1);
    SleepMs (100 + Index * 200);
    OsLog ("exynosbt: BT_EN on - 0x%08lx (try %lu)", Status, Index + 1);
    LogPins ();
    LogUartState (Bt);
    Status = HciReset (Bt);
    OsLog ("exynosbt: HCI_Reset - 0x%08lx", Status);
    if (NT_SUCCESS (Status)) {
      break;
    }
  }
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  Status = HciCommand (Bt, 0x0C14, NULL, 0, Out, sizeof (Out), &OutLength);   // Read_Local_Name
  if (NT_SUCCESS (Status) && (OutLength > 1)) {
    for (Index = 0; (Index < sizeof (Bt->Name) - 1) && (Index + 1 < OutLength) && (Out[1 + Index] != 0); Index++) {
      Bt->Name[Index] = (CHAR)Out[1 + Index];
    }
    Bt->Name[Index] = 0;
  }
  OsLog ("exynosbt: controller \"%s\" - 0x%08lx", Bt->Name, Status);

  Patch = (strstr (Bt->Name, "4335") != NULL) || (strstr (Bt->Name, "4339") != NULL) ?
          L"\\SystemRoot\\System32\\drivers\\bcm4335_V0105.0541.hcd" :
          L"\\SystemRoot\\System32\\drivers\\bcm4350_V0301.0591.hcd";

  // Patch download at the high rate when the controller takes it.
  SetBaud (Bt, TARGET_BAUD);
  Status = DownloadPatch (Bt, Patch);

  // The patch restarts the controller at 115200.
  UartBaud (Bt, INIT_BAUD);
  UartIoctl (Bt, IOCTL_SERIAL_PURGE, &(ULONG){ SERIAL_PURGE_RXCLEAR }, sizeof (ULONG));
  Status = HciReset (Bt);
  OsLog ("exynosbt: HCI_Reset after the patch - 0x%08lx", Status);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  if (NT_SUCCESS (HciCommand (Bt, 0x1001, NULL, 0, Out, sizeof (Out), &OutLength)) && (OutLength >= 9)) {
    OsLog ("exynosbt: HCI version %u rev 0x%04x, LMP %u subversion 0x%04x, manufacturer %u", Out[1],
           Out[2] | (Out[3] << 8), Out[4], Out[7] | (Out[8] << 8), Out[5] | (Out[6] << 8));
  }
  if (NT_SUCCESS (HciCommand (Bt, 0x1009, NULL, 0, Out, sizeof (Out), &OutLength)) && (OutLength >= 7)) {
    OsLog ("exynosbt: BD_ADDR %02x:%02x:%02x:%02x:%02x:%02x", Out[6], Out[5], Out[4], Out[3], Out[2], Out[1]);
  }
  SetBaud (Bt, TARGET_BAUD);
  return STATUS_SUCCESS;
}

//
// ---------------------------------------------------------------------------
// Runtime: the read pump and BthMini's requests.
// ---------------------------------------------------------------------------
//

/**
  Complete a pended READ_HCI with a packet.  Caller holds Bt->Lock.
**/
static BOOLEAN
DeliverLocked (PBT_CONTEXT Bt, PACKET *Packet)
{
  WDFREQUEST                    Request;
  PIRP                          Irp;
  PBTHX_HCI_READ_WRITE_CONTEXT  Ctx;
  ULONG                         Size;

  if (!NT_SUCCESS (WdfIoQueueRetrieveNextRequest ((Packet->Type == H4_EVENT) ? Bt->ReadEvents : Bt->ReadAcl,
                                                  &Request))) {
    return FALSE;
  }
  Irp  = WdfRequestWdmGetIrp (Request);
  Size = IoGetCurrentIrpStackLocation (Irp)->Parameters.DeviceIoControl.OutputBufferLength;
  Ctx  = (PBTHX_HCI_READ_WRITE_CONTEXT)Irp->UserBuffer;
  if ((Ctx == NULL) || (Size < FIELD_OFFSET (BTHX_HCI_READ_WRITE_CONTEXT, Data) + Packet->Length)) {
    OsLog ("exynosbt: READ_HCI buffer %lu too small for %u bytes", Size, Packet->Length);
    WdfRequestComplete (Request, STATUS_BUFFER_TOO_SMALL);
    return TRUE;
  }
  Ctx->DataLen = Packet->Length;
  Ctx->Type    = Packet->Type;
  RtlCopyMemory (Ctx->Data, Packet->Data, Packet->Length);
  WdfRequestCompleteWithInformation (Request, STATUS_SUCCESS, FIELD_OFFSET (BTHX_HCI_READ_WRITE_CONTEXT, Data) +
                                     Packet->Length);
  return TRUE;
}

/**
  A packet from the controller: to a waiting read, else into the ring.
**/
static VOID
PacketIn (PBT_CONTEXT Bt)
{
  if (Bt->RxPackets < 24) {
    OsLog ("exynosbt: in type %u, %u bytes: %02x %02x %02x %02x %02x %02x", Bt->RxPacket.Type, Bt->RxPacket.Length,
           Bt->RxPacket.Data[0], Bt->RxPacket.Data[1], Bt->RxPacket.Data[2], Bt->RxPacket.Data[3],
           Bt->RxPacket.Data[4], Bt->RxPacket.Data[5]);
  }
  Bt->RxPackets++;
  WdfSpinLockAcquire (Bt->Lock);
  if ((Bt->Pdo == NULL) || !DeliverLocked (Bt, &Bt->RxPacket)) {
    if (Bt->RingCount < RING_PACKETS) {
      Bt->Ring[(Bt->RingHead + Bt->RingCount) % RING_PACKETS] = Bt->RxPacket;
      Bt->RingCount++;
    } else if (Bt->Dropped++ % 16 == 0) {
      OsLog ("exynosbt: no reader, %lu packets dropped", Bt->Dropped);
    }
  }
  WdfSpinLockRelease (Bt->Lock);
}

static VOID
RxNext (PBT_CONTEXT Bt)
{
  WDF_REQUEST_REUSE_PARAMS  Reuse;
  WDFMEMORY_OFFSET          Offset;
  ULONG                     At;
  NTSTATUS                  Status;

  if (Bt->RxStop) {
    KeSetEvent (&Bt->RxStopped, IO_NO_INCREMENT, FALSE);
    return;
  }
  switch (Bt->RxState) {
    case RxType:    At = 0; Bt->RxWant = 1; break;
    case RxHeader:  At = 1; Bt->RxWant = (Bt->RxPacket.Type == H4_EVENT) ? 2 : 4; break;
    default:        At = 1 + ((Bt->RxPacket.Type == H4_EVENT) ? 2 : 4); break;   // RxWant set by the header
  }
  WDF_REQUEST_REUSE_PARAMS_INIT (&Reuse, WDF_REQUEST_REUSE_NO_FLAGS, STATUS_SUCCESS);
  WdfRequestReuse (Bt->RxRequest, &Reuse);
  Offset.BufferOffset = At;
  Offset.BufferLength = Bt->RxWant;
  Status = WdfIoTargetFormatRequestForRead (Bt->Uart, Bt->RxRequest, Bt->RxMemory, &Offset, NULL);
  if (NT_SUCCESS (Status)) {
    WdfRequestSetCompletionRoutine (Bt->RxRequest, RxComplete, Bt);
    if (!WdfRequestSend (Bt->RxRequest, Bt->Uart, WDF_NO_SEND_OPTIONS)) {
      Status = WdfRequestGetStatus (Bt->RxRequest);
    }
  }
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosbt: read pump stopped: 0x%08lx", Status);
    KeSetEvent (&Bt->RxStopped, IO_NO_INCREMENT, FALSE);
  }
}

/**
  The receive buffer (RxMemory) holds [type][header][payload] of the
  packet being read; RxPacket gets the packet without the type byte.
**/
VOID
RxComplete (WDFREQUEST Request, WDFIOTARGET Target, PWDF_REQUEST_COMPLETION_PARAMS Params, WDFCONTEXT Context)
{
  PBT_CONTEXT  Bt = Context;
  PUCHAR       Buf = WdfMemoryGetBuffer (Bt->RxMemory, NULL);
  ULONG        HdrLen;
  ULONG        Payload;

  UNREFERENCED_PARAMETER (Request);
  UNREFERENCED_PARAMETER (Target);

  if (Bt->Ready && (Bt->RxStepLogs++ < 60)) {
    OsLog ("exynosbt: rx step %u: 0x%08lx, %Iu of %lu bytes, first 0x%02x", Bt->RxState, Params->IoStatus.Status,
           Params->IoStatus.Information, Bt->RxWant, Buf[(Bt->RxState == RxType) ? 0 : 1]);
  }
  if (!NT_SUCCESS (Params->IoStatus.Status) || (Params->IoStatus.Information != Bt->RxWant)) {
    if (!Bt->RxStop) {
      OsLog ("exynosbt: UART read 0x%08lx (%Iu of %lu bytes)", Params->IoStatus.Status, Params->IoStatus.Information,
             Bt->RxWant);
    }
    if (Bt->RxStop || (Params->IoStatus.Status == STATUS_CANCELLED)) {
      KeSetEvent (&Bt->RxStopped, IO_NO_INCREMENT, FALSE);
      return;
    }
    Bt->RxState = RxType;
    RxNext (Bt);
    return;
  }

  switch (Bt->RxState) {
    case RxType:
      if ((Buf[0] == H4_EVENT) || (Buf[0] == H4_ACL)) {
        Bt->RxPacket.Type = Buf[0];
        Bt->RxState       = RxHeader;
      } else if (Bt->Logs++ < 20) {
        OsLog ("exynosbt: unexpected packet type 0x%02x, resyncing", Buf[0]);
      }
      break;
    case RxHeader:
      HdrLen  = (Bt->RxPacket.Type == H4_EVENT) ? 2 : 4;
      Payload = (Bt->RxPacket.Type == H4_EVENT) ? Buf[2] : (Buf[3] | (Buf[4] << 8));
      if (HdrLen + Payload > MAX_PACKET) {
        OsLog ("exynosbt: packet of %lu bytes too long, resyncing", HdrLen + Payload);
        Bt->RxState = RxType;
        break;
      }
      RtlCopyMemory (Bt->RxPacket.Data, Buf + 1, HdrLen);
      Bt->RxPacket.Length = (USHORT)HdrLen;
      if (Payload == 0) {
        PacketIn (Bt);
        Bt->RxState = RxType;
      } else {
        Bt->RxWant  = Payload;           // RxNext reads it after the header
        Bt->RxState = RxPayload;
      }
      break;
    default:
      HdrLen = (Bt->RxPacket.Type == H4_EVENT) ? 2 : 4;
      RtlCopyMemory (Bt->RxPacket.Data + HdrLen, Buf + 1 + HdrLen, Bt->RxWant);
      Bt->RxPacket.Length = (USHORT)(HdrLen + Bt->RxWant);
      PacketIn (Bt);
      Bt->RxState = RxType;
      break;
  }
  RxNext (Bt);
}

VOID
TxComplete (WDFREQUEST Request, WDFIOTARGET Target, PWDF_REQUEST_COMPLETION_PARAMS Params, WDFCONTEXT Context)
{
  WDFREQUEST  Original = GetWrite (Request)->Original;
  PIRP        Irp = WdfRequestWdmGetIrp (Original);
  ULONG       OutSize = IoGetCurrentIrpStackLocation (Irp)->Parameters.DeviceIoControl.OutputBufferLength;
  ULONG_PTR   Info = 0;

  UNREFERENCED_PARAMETER (Target);

  if (((PBT_CONTEXT)Context)->TxDoneLogs++ < 24) {
    OsLog ("exynosbt: write done 0x%08lx (%Iu bytes), output %p size %lu", Params->IoStatus.Status,
           Params->IoStatus.Information, Irp->UserBuffer, OutSize);
  }

  //
  // BthMini's WRITE_HCI has a ULONG output: the bytes written (without
  // it, BthMini took the write as failed and stopped: code 43).
  //
  if (NT_SUCCESS (Params->IoStatus.Status) && (Irp->UserBuffer != NULL) && (OutSize >= sizeof (ULONG))) {
    *(PULONG)Irp->UserBuffer = GetWrite (Request)->DataLen;
    Info = sizeof (ULONG);
  }
  WdfRequestCompleteWithInformation (Original, Params->IoStatus.Status, Info);
  WdfObjectDelete (Request);
}

/**
  WRITE_HCI: the packet with its H4 type byte in front, sent to the UART;
  BthMini's request completes with the UART write.
**/
static NTSTATUS
WriteHci (PBT_CONTEXT Bt, WDFREQUEST Request, PBTHX_HCI_READ_WRITE_CONTEXT In, ULONG InSize)
{
  WDF_OBJECT_ATTRIBUTES  Attributes;
  WDFREQUEST             Write;
  WDFMEMORY              Memory;
  PUCHAR                 Buf;
  NTSTATUS               Status;

  if ((In == NULL) || (InSize < FIELD_OFFSET (BTHX_HCI_READ_WRITE_CONTEXT, Data)) ||
      (InSize < FIELD_OFFSET (BTHX_HCI_READ_WRITE_CONTEXT, Data) + In->DataLen) || (In->DataLen > MAX_PACKET)) {
    return STATUS_INVALID_PARAMETER;
  }
  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, WRITE_CONTEXT);
  Attributes.ParentObject = Bt->Device;
  Status = WdfRequestCreate (&Attributes, Bt->Uart, &Write);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  WDF_OBJECT_ATTRIBUTES_INIT (&Attributes);
  Attributes.ParentObject = Write;
  Status = WdfMemoryCreate (&Attributes, NonPagedPool, BT_TAG, 1 + In->DataLen, &Memory, (PVOID *)&Buf);
  if (NT_SUCCESS (Status)) {
    Buf[0] = In->Type;
    RtlCopyMemory (Buf + 1, In->Data, In->DataLen);
    Status = WdfIoTargetFormatRequestForWrite (Bt->Uart, Write, Memory, NULL, NULL);
  }
  if (NT_SUCCESS (Status)) {
    GetWrite (Write)->Original = Request;
    GetWrite (Write)->DataLen  = In->DataLen;
    if (Bt->TxPackets < 24) {
      OsLog ("exynosbt: out type %u, %lu bytes: %02x %02x %02x %02x", In->Type, In->DataLen, In->Data[0],
             (In->DataLen > 1) ? In->Data[1] : 0, (In->DataLen > 2) ? In->Data[2] : 0, (In->DataLen > 3) ? In->Data[3] : 0);
    }
    WdfRequestSetCompletionRoutine (Write, TxComplete, Bt);
    Bt->TxPackets++;
    if (WdfRequestSend (Write, Bt->Uart, WDF_NO_SEND_OPTIONS)) {
      return STATUS_PENDING;
    }
    Status = WdfRequestGetStatus (Write);
  }
  WdfObjectDelete (Write);
  return Status;
}

/**
  READ_HCI: the packet kind wanted is the input's first byte (2 ACL,
  4 event; Serial HCI bus sample); a buffered packet of that kind
  completes it at once, else it waits in that kind's queue.
**/
static NTSTATUS
ReadHci (PBT_CONTEXT Bt, WDFREQUEST Request, PUCHAR In, ULONG InSize)
{
  UCHAR     Kind = ((In != NULL) && (InSize >= 1)) ? In[0] : H4_EVENT;
  ULONG     Index;
  NTSTATUS  Status;

  if ((Kind != H4_EVENT) && (Kind != H4_ACL) && (InSize >= 5)) {
    Kind = In[4];                        // a BTHX_HCI_READ_WRITE_CONTEXT
  }
  WdfSpinLockAcquire (Bt->Lock);
  Status = WdfRequestForwardToIoQueue (Request, (Kind == H4_ACL) ? Bt->ReadAcl : Bt->ReadEvents);
  if (NT_SUCCESS (Status)) {
    // The oldest buffered packet of that kind, if any, goes to it now.
    for (Index = 0; Index < Bt->RingCount; Index++) {
      PACKET  *P = &Bt->Ring[(Bt->RingHead + Index) % RING_PACKETS];

      if (((P->Type == H4_ACL) == (Kind == H4_ACL)) && DeliverLocked (Bt, P)) {
        // close the gap
        for (; Index + 1 < Bt->RingCount; Index++) {
          Bt->Ring[(Bt->RingHead + Index) % RING_PACKETS] = Bt->Ring[(Bt->RingHead + Index + 1) % RING_PACKETS];
        }
        Bt->RingCount--;
        break;
      }
    }
  }
  WdfSpinLockRelease (Bt->Lock);
  return NT_SUCCESS (Status) ? STATUS_PENDING : Status;
}

VOID
EvtPdoInternalIoctl (WDFQUEUE Queue, WDFREQUEST Request, size_t OutputBufferLength, size_t InputBufferLength,
                     ULONG IoControlCode)
{
  PBT_CONTEXT          Bt = GetPdo (WdfIoQueueGetDevice (Queue))->Bt;
  PIRP                 Irp = WdfRequestWdmGetIrp (Request);
  PVOID                In = IoGetCurrentIrpStackLocation (Irp)->Parameters.DeviceIoControl.Type3InputBuffer;
  PVOID                Out = Irp->UserBuffer;
  ULONG_PTR            Info = 0;
  NTSTATUS             Status;

  if ((Bt->Logs < 60) && (((IoControlCode != IOCTL_BTHX_READ_HCI) && (IoControlCode != IOCTL_BTHX_WRITE_HCI)) ||
                           (Bt->Logs < 20))) {
    Bt->Logs++;
    OsLog ("exynosbt: BthX %s 0x%08lx (in %Iu, out %Iu, first byte %d)",
           (IoGetCurrentIrpStackLocation (Irp)->MajorFunction == IRP_MJ_INTERNAL_DEVICE_CONTROL) ? "internal ioctl" : "ioctl",
           IoControlCode, InputBufferLength, OutputBufferLength,
           ((In != NULL) && (InputBufferLength != 0)) ? ((PUCHAR)In)[0] : -1);
  }
  switch (IoControlCode) {
    case IOCTL_BTHX_GET_VERSION:
      if ((Out == NULL) || (OutputBufferLength < sizeof (BTHX_VERSION))) {
        Status = STATUS_BUFFER_TOO_SMALL;
      } else {
        ((PBTHX_VERSION)Out)->Version = BTHX_DDI_VERSION_1;
        Info   = sizeof (BTHX_VERSION);
        Status = STATUS_SUCCESS;
      }
      break;
    case IOCTL_BTHX_SET_VERSION:
      Status = ((In != NULL) && (InputBufferLength >= sizeof (BTHX_VERSION)) &&
                (((PBTHX_VERSION)In)->Version == BTHX_DDI_VERSION_1)) ? STATUS_SUCCESS : STATUS_NOT_SUPPORTED;
      break;
    case IOCTL_BTHX_QUERY_CAPABILITIES:
      if ((Out == NULL) || (OutputBufferLength < sizeof (BTHX_CAPABILITIES))) {
        Status = STATUS_BUFFER_TOO_SMALL;
      } else {
        PBTHX_CAPABILITIES  Caps = Out;

        RtlZeroMemory (Caps, sizeof (*Caps));
        Caps->MaxAclTransferInSize = MAX_PACKET;
        //
        // As the Serial HCI bus sample reports it: SCO bypassing HCI (the
        // chip's PCM pins), one channel.  ScoSupportNone with 0 channels
        // made BthMini fail its start right after this query.
        //
        Caps->ScoSupport           = ScoSupportHCIBypass;
        Caps->MaxScoChannels       = 1;
        Caps->IsDeviceIdleCapable  = FALSE;
        Caps->IsDeviceWakeCapable  = FALSE;
        Info   = sizeof (BTHX_CAPABILITIES);
        Status = STATUS_SUCCESS;
      }
      break;
    case IOCTL_BTHX_WRITE_HCI:
      Status = WriteHci (Bt, Request, In, (ULONG)InputBufferLength);
      break;
    case IOCTL_BTHX_READ_HCI:
      Status = ReadHci (Bt, Request, In, (ULONG)InputBufferLength);
      break;
    default:
      Status = STATUS_NOT_SUPPORTED;
      break;
  }
  if (Status != STATUS_PENDING) {
    WdfRequestCompleteWithInformation (Request, Status, Info);
  }
}

//
// ---------------------------------------------------------------------------
// The child BthMini binds to.
// ---------------------------------------------------------------------------
//

static ULONG  mChildIrpLogs;

/**
  Diagnostics: every IRP that reaches the child (PnP, power, create,
  ioctls), logged before KMDF handles it as usual.
**/
static NTSTATUS
EvtChildIrp (WDFDEVICE Device, PIRP Irp)
{
  PIO_STACK_LOCATION  Stack = IoGetCurrentIrpStackLocation (Irp);

  if (mChildIrpLogs++ < 120) {
    if ((Stack->MajorFunction == IRP_MJ_DEVICE_CONTROL) || (Stack->MajorFunction == IRP_MJ_INTERNAL_DEVICE_CONTROL)) {
      OsLog ("exynosbt: child IRP %u code 0x%08lx", Stack->MajorFunction, Stack->Parameters.DeviceIoControl.IoControlCode);
    } else if ((Stack->MajorFunction == IRP_MJ_PNP) && (Stack->MinorFunction == IRP_MN_QUERY_INTERFACE)) {
      CONST GUID  *G = Stack->Parameters.QueryInterface.InterfaceType;

      OsLog ("exynosbt: child PnP query interface {%08lx-%04x-%04x-%02x%02x-...} version %u", G->Data1, G->Data2,
             G->Data3, G->Data4[0], G->Data4[1], Stack->Parameters.QueryInterface.Version);
    } else {
      OsLog ("exynosbt: child IRP %u/%u", Stack->MajorFunction, Stack->MinorFunction);
    }
  }
  IoSkipCurrentIrpStackLocation (Irp);
  return WdfDeviceWdmDispatchPreprocessedIrp (Device, Irp);
}

static NTSTATUS
CreateChild (PBT_CONTEXT Bt)
{
  DECLARE_CONST_UNICODE_STRING (DeviceId, CHILD_DEVICE_ID);
  DECLARE_CONST_UNICODE_STRING (Id, BTHMINI_ID);
  DECLARE_CONST_UNICODE_STRING (Instance, L"0");
  DECLARE_CONST_UNICODE_STRING (Text, L"Bluetooth over UART (Broadcom)");
  PWDFDEVICE_INIT        Init;
  WDF_OBJECT_ATTRIBUTES  Attributes;
  WDF_IO_QUEUE_CONFIG    QueueConfig;
  WDFDEVICE              Pdo;
  NTSTATUS               Status;

  Init = WdfPdoInitAllocate (Bt->Device);
  if (Init == NULL) {
    return STATUS_INSUFFICIENT_RESOURCES;
  }
  Status = WdfPdoInitAssignDeviceID (Init, &DeviceId);
  if (NT_SUCCESS (Status)) Status = WdfPdoInitAddHardwareID (Init, &Id);
  if (NT_SUCCESS (Status)) Status = WdfPdoInitAddCompatibleID (Init, &Id);
  if (NT_SUCCESS (Status)) Status = WdfPdoInitAssignInstanceID (Init, &Instance);
  if (NT_SUCCESS (Status)) Status = WdfPdoInitAddDeviceText (Init, &Text, &Text, 0x409);
  if (NT_SUCCESS (Status)) {
    static CONST UCHAR  Majors[] = { IRP_MJ_CREATE, IRP_MJ_CLOSE, IRP_MJ_READ, IRP_MJ_WRITE, IRP_MJ_DEVICE_CONTROL,
                                     IRP_MJ_INTERNAL_DEVICE_CONTROL, IRP_MJ_PNP, IRP_MJ_POWER, IRP_MJ_SYSTEM_CONTROL };
    ULONG               M;

    WdfPdoInitSetDefaultLocale (Init, 0x409);
    for (M = 0; NT_SUCCESS (Status) && (M < ARRAYSIZE (Majors)); M++) {
      Status = WdfDeviceInitAssignWdmIrpPreprocessCallback (Init, EvtChildIrp, Majors[M], NULL, 0);
    }
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, PDO_CONTEXT);
    Status = WdfDeviceCreate (&Init, &Attributes, &Pdo);
  }
  if (!NT_SUCCESS (Status)) {
    if (Init != NULL) {
      WdfDeviceInitFree (Init);
    }
    return Status;
  }
  GetPdo (Pdo)->Bt = Bt;

  WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE (&QueueConfig, WdfIoQueueDispatchParallel);
  QueueConfig.EvtIoInternalDeviceControl = EvtPdoInternalIoctl;
  QueueConfig.EvtIoDeviceControl         = EvtPdoInternalIoctl;   // BthMini sends the BthX codes as device controls
  //
  // Not power managed: BthMini sends IOCTL_BTHX_GET_VERSION while it is
  // being added, before the child is started; a power-managed queue held
  // it until a start that never came (boot hung).
  //
  QueueConfig.PowerManaged               = WdfFalse;
  Status = WdfIoQueueCreate (Pdo, &QueueConfig, WDF_NO_OBJECT_ATTRIBUTES, NULL);
  if (NT_SUCCESS (Status)) {
    WDF_IO_QUEUE_CONFIG_INIT (&QueueConfig, WdfIoQueueDispatchManual);
    QueueConfig.PowerManaged = WdfFalse;
    Status = WdfIoQueueCreate (Pdo, &QueueConfig, WDF_NO_OBJECT_ATTRIBUTES, &Bt->ReadEvents);
  }
  if (NT_SUCCESS (Status)) {
    Status = WdfIoQueueCreate (Pdo, &QueueConfig, WDF_NO_OBJECT_ATTRIBUTES, &Bt->ReadAcl);
  }
  if (NT_SUCCESS (Status)) {
    Status = WdfFdoAddStaticChild (Bt->Device, Pdo);
  }
  if (NT_SUCCESS (Status)) {
    WdfSpinLockAcquire (Bt->Lock);
    Bt->Pdo = Pdo;
    WdfSpinLockRelease (Bt->Lock);
  }
  OsLog ("exynosbt: child MS_BTHX_BTHMINI - 0x%08lx", Status);
  return Status;
}

//
// ---------------------------------------------------------------------------
// PnP
// ---------------------------------------------------------------------------
//

NTSTATUS
EvtPrepareHardware (WDFDEVICE Device, WDFCMRESLIST Raw, WDFCMRESLIST Translated)
{
  PBT_CONTEXT                      Bt = GetBt (Device);
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  WDF_OBJECT_ATTRIBUTES            Attributes;
  ULONG                            Index;
  NTSTATUS                         Status;

  UNREFERENCED_PARAMETER (Raw);

  Bt->UartId.QuadPart = 0;
  Bt->GpioCount = 0;
  for (Index = 0; Index < WdfCmResourceListGetCount (Translated); Index++) {
    Desc = WdfCmResourceListGetDescriptor (Translated, Index);
    if (Desc->Type != CmResourceTypeConnection) {
      continue;
    }
    if ((Desc->u.Connection.Class == CM_RESOURCE_CONNECTION_CLASS_SERIAL) &&
        (Desc->u.Connection.Type == CM_RESOURCE_CONNECTION_TYPE_SERIAL_UART)) {
      Bt->UartId.LowPart  = Desc->u.Connection.IdLowPart;
      Bt->UartId.HighPart = Desc->u.Connection.IdHighPart;
    } else if ((Desc->u.Connection.Class == CM_RESOURCE_CONNECTION_CLASS_GPIO) &&
               (Desc->u.Connection.Type == CM_RESOURCE_CONNECTION_TYPE_GPIO_IO) && (Bt->GpioCount < 3)) {
      Bt->GpioId[Bt->GpioCount].LowPart  = Desc->u.Connection.IdLowPart;
      Bt->GpioId[Bt->GpioCount].HighPart = Desc->u.Connection.IdHighPart;
      Bt->GpioCount++;
    }
  }
  OsLog ("exynosbt: UART connection 0x%08lx, %lu GPIO lines", Bt->UartId.LowPart, Bt->GpioCount);
  if ((Bt->UartId.QuadPart == 0) || (Bt->GpioCount < 2)) {
    return STATUS_DEVICE_CONFIGURATION_ERROR;
  }

  Status = OpenTarget (Device, Bt->UartId, GENERIC_READ | GENERIC_WRITE, &Bt->Uart);
  OsLog ("exynosbt: UART open - 0x%08lx", Status);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  Status = OpenTarget (Device, Bt->GpioId[0], GENERIC_WRITE, &Bt->GpioEn);
  if (NT_SUCCESS (Status)) {
    Status = OpenTarget (Device, Bt->GpioId[1], GENERIC_WRITE, &Bt->GpioWake);
  }
  //
  // HOST_WAKE: connecting it as an input applies the DSDT's PullNone.  At
  // reset GPX2(1) has a pull-down, with which the controller does not start
  // (CTS stays inactive after BT_EN); the Samsung board driver clears it too.
  //
  if (NT_SUCCESS (Status) && (Bt->GpioCount >= 3) && (Bt->GpioHostWake == NULL)) {
    NTSTATUS  Wake = OpenTarget (Device, Bt->GpioId[2], GENERIC_READ, &Bt->GpioHostWake);

    OsLog ("exynosbt: HOST_WAKE open (pull none) - 0x%08lx", Wake);
  }
  OsLog ("exynosbt: GPIO open - 0x%08lx", Status);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  Status = BringUp (Bt);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosbt: bring-up failed 0x%08lx", Status);
    return Status;
  }

  //
  // Read pump (one request, reused), then the child.
  //
  if (Bt->RxRequest == NULL) {
    WDF_OBJECT_ATTRIBUTES_INIT (&Attributes);
    Attributes.ParentObject = Device;
    Status = WdfRequestCreate (&Attributes, Bt->Uart, &Bt->RxRequest);
    if (NT_SUCCESS (Status)) {
      Status = WdfMemoryCreate (&Attributes, NonPagedPool, BT_TAG, 1 + MAX_PACKET, &Bt->RxMemory, NULL);
    }
    if (!NT_SUCCESS (Status)) {
      return Status;
    }
  }
  if (Bt->Pdo == NULL) {
    Status = CreateChild (Bt);
  }
  Bt->Ready = TRUE;
  return Status;
}

/**
  The read pump runs while the device is in D0 (after the bring-up).
**/
NTSTATUS
EvtD0Entry (WDFDEVICE Device, WDF_POWER_DEVICE_STATE PreviousState)
{
  PBT_CONTEXT  Bt = GetBt (Device);

  UNREFERENCED_PARAMETER (PreviousState);

  if (Bt->Ready && (Bt->RxRequest != NULL)) {
    Bt->RxStop  = FALSE;
    Bt->RxState = RxType;
    KeClearEvent (&Bt->RxStopped);
    RxNext (Bt);
    OsLog ("exynosbt: D0, reading");
  }
  return STATUS_SUCCESS;
}

VOID
EvtCleanup (WDFOBJECT Object)
{
  PBT_CONTEXT  Bt = GetBt (Object);

  if (Bt->Ring != NULL) {
    ExFreePoolWithTag (Bt->Ring, BT_TAG);
    Bt->Ring = NULL;
  }
}

NTSTATUS
EvtD0Exit (WDFDEVICE Device, WDF_POWER_DEVICE_STATE TargetState)
{
  PBT_CONTEXT    Bt = GetBt (Device);
  LARGE_INTEGER  Wait;

  UNREFERENCED_PARAMETER (TargetState);

  if ((Bt->RxRequest != NULL) && !Bt->RxStop) {
    Bt->RxStop = TRUE;
    WdfRequestCancelSentRequest (Bt->RxRequest);
    Wait.QuadPart = -10000LL * 500;
    KeWaitForSingleObject (&Bt->RxStopped, Executive, KernelMode, FALSE, &Wait);
  }
  return STATUS_SUCCESS;
}

NTSTATUS
EvtReleaseHardware (WDFDEVICE Device, WDFCMRESLIST Translated)
{
  PBT_CONTEXT  Bt = GetBt (Device);

  UNREFERENCED_PARAMETER (Translated);

  GpioSet (Bt->GpioWake, 0);
  GpioSet (Bt->GpioEn, 0);               // controller off
  OsLog ("exynosbt: released (%lu packets in, %lu out, %lu dropped)", Bt->RxPackets, Bt->TxPackets, Bt->Dropped);
  if (Bt->GpioWake != NULL) { WdfObjectDelete (Bt->GpioWake); Bt->GpioWake = NULL; }
  if (Bt->GpioEn != NULL) { WdfObjectDelete (Bt->GpioEn); Bt->GpioEn = NULL; }
  if (Bt->GpioHostWake != NULL) { WdfObjectDelete (Bt->GpioHostWake); Bt->GpioHostWake = NULL; }
  Bt->Ready = FALSE;
  return STATUS_SUCCESS;
}

NTSTATUS
EvtDeviceAdd (WDFDRIVER Driver, PWDFDEVICE_INIT DeviceInit)
{
  WDF_PNPPOWER_EVENT_CALLBACKS  PnpPower;
  WDF_OBJECT_ATTRIBUTES         Attributes;
  WDFDEVICE                     Device;
  PBT_CONTEXT                   Bt;
  NTSTATUS                      Status;

  UNREFERENCED_PARAMETER (Driver);

  WDF_PNPPOWER_EVENT_CALLBACKS_INIT (&PnpPower);
  PnpPower.EvtDevicePrepareHardware = EvtPrepareHardware;
  PnpPower.EvtDeviceReleaseHardware = EvtReleaseHardware;
  PnpPower.EvtDeviceD0Entry         = EvtD0Entry;
  PnpPower.EvtDeviceD0Exit          = EvtD0Exit;
  WdfDeviceInitSetPnpPowerEventCallbacks (DeviceInit, &PnpPower);

  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, BT_CONTEXT);
  Attributes.EvtCleanupCallback = EvtCleanup;
  Status = WdfDeviceCreate (&DeviceInit, &Attributes, &Device);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  Bt = GetBt (Device);
  Bt->Device = Device;
  KeInitializeEvent (&Bt->RxStopped, NotificationEvent, TRUE);
  Status = WdfSpinLockCreate (WDF_NO_OBJECT_ATTRIBUTES, &Bt->Lock);
  if (NT_SUCCESS (Status)) {
    Bt->Ring = ExAllocatePoolWithTag (NonPagedPool, RING_PACKETS * sizeof (PACKET), BT_TAG);
    Status = (Bt->Ring != NULL) ? STATUS_SUCCESS : STATUS_INSUFFICIENT_RESOURCES;
  }
  OsLog ("exynosbt: device added - 0x%08lx", Status);
  return Status;
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG  Config;

  OsLogInit ();
  OsLog ("exynosbt: DriverEntry (built " __DATE__ " " __TIME__ ")");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  return WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, WDF_NO_HANDLE);
}
