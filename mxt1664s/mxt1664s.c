/** @file
  Atmel maXTouch mXT1664S touchscreen (v1awifi, SM-P900) as a HID
  multi-touch digitizer for Windows RT 8.1 (KMDF 1.13).  Ported from the
  GT-N7000's mxt540e.

  Stack: mshidkmdf.sys is the function driver, this is its lower filter
  and answers the IOCTL_HID_* requests it passes down.  The controller is
  reached through SpbCx (I2cSerialBus, exynosi2c) and its CHG line is a
  GpioInt (exynosgpio); the interrupt is handled at PASSIVE level so the
  ISR can read the message queue over I2C itself.

  Like UEFI's TouchDxe, this relies on the configuration Linux stored in
  the controller's NVM: it reads the object table, finds T5 (messages) and
  the touch object, T100 or (older firmware, this unit) T9, and reports up
  to 10 contacts.  UEFI powers the controller (MAX77802 LDO29, TOUCH_EN /
  EN_1 / RESET) and leaves it on.

  Progress and failures (step MXT_STEP_*, status) go to the OS log ring and
  from there over USB (exynosusb); a failed bring-up fails the device.
  Built with LOAD_TEST=1 it bugchecks 0x4D585421 ("MXT!") in PrepareHardware
  with P1 = I2C connection ID, P2 = interrupt (raw GSI), P3 = resource count.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <ntddk.h>
#include <wdf.h>
#include <hidport.h>
#include <spb.h>
#define RESHUB_USE_HELPER_ROUTINES
#include <reshub.h>
#include "oslog.h"

#ifndef LOAD_TEST
#define LOAD_TEST 0
#endif
#define MXT_BUGCHECK  0x4D585421

enum {
  MXT_STEP_NO_CONNECTION = 1,
  MXT_STEP_OPEN_TARGET,
  MXT_STEP_INFO_BLOCK,
  MXT_STEP_OBJECT_TABLE,
  MXT_STEP_NO_T5_T9,
  MXT_STEP_TOUCH_CONFIG,
  MXT_STEP_INTERRUPT,
};

//
// Object types and T9 message bits (see TouchDxe / Linux mxt1664s.c).
//
#define MXT_GEN_MESSAGEPROCESSOR_T5    5
#define MXT_TOUCH_MULTITOUCHSCREEN_T9  9
#define MXT_TOUCH_MULTITOUCHSCREEN_T100  100

//
// T100: the first two report IDs are screen status, then one per touch;
// message [1] bit 7 detect, [2-3] X, [4-5] Y (little endian); config
// [13-14] XRANGE, [24-25] YRANGE (mxts.c mxt_treat_T100_object()).
//
#define MXT_T100_SCREEN_REPORTS  2
#define MXT_T100_DETECT          0x80
#define MXT_T100_XRANGE          13
#define MXT_T100_YRANGE          24
#define MXT_T100_CFG_SIZE        26
#define MXT_T9_DETECT    0x80
#define MXT_T9_RELEASE   0x20
#define MXT_REPORT_NONE  0xFF

#define MXT_INFO_SIZE          7
#define MXT_OBJECT_TABLE       7
#define MXT_OBJECT_ENTRY_SIZE  6
#define MXT_MAX_OBJECTS        64
#define MXT_MAX_MSG_SIZE       16
#define MXT_MAX_DRAIN          64

#define MAX_CONTACTS  10

//
// HID: one input report (ID 1) with all contacts, and the Contact Count
// Maximum feature (ID 2) Windows 8 requires.
//
#define REPORTID_TOUCH     1
#define REPORTID_MAXCOUNT  2

#include <pshpack1.h>
typedef struct {
  UCHAR   Tip;            // bit 0
  UCHAR   ContactId;
  USHORT  X;
  USHORT  Y;
} HID_CONTACT;

typedef struct {
  UCHAR        ReportId;
  HID_CONTACT  Contacts[MAX_CONTACTS];
  UCHAR        ContactCount;
} HID_TOUCH_REPORT;

typedef struct {
  UCHAR  ReportId;
  UCHAR  MaxCount;
} HID_MAXCOUNT_REPORT;
#include <poppack.h>

//
// Logical maxima are patched in from the touch object's ranges at probe
// time; the physical size is the panel's (12.2" 16:10, 263 x 164 mm, in
// 0.01 cm).  X runs along the 2560-pixel side.
//
#define FINGER(n) \
    0x05, 0x0D,                    /*   Usage Page (Digitizer)          */ \
    0x09, 0x22,                    /*   Usage (Finger)                  */ \
    0xA1, 0x02,                    /*   Collection (Logical)            */ \
    0x15, 0x00,                    /*     Logical Minimum (0)           */ \
    0x25, 0x01,                    /*     Logical Maximum (1)           */ \
    0x75, 0x01,                    /*     Report Size (1)               */ \
    0x95, 0x01,                    /*     Report Count (1)              */ \
    0x09, 0x42,                    /*     Usage (Tip Switch)            */ \
    0x81, 0x02,                    /*     Input (Data,Var,Abs)          */ \
    0x95, 0x07,                    /*     Report Count (7)              */ \
    0x81, 0x03,                    /*     Input (Cnst,Var,Abs)          */ \
    0x75, 0x08,                    /*     Report Size (8)               */ \
    0x95, 0x01,                    /*     Report Count (1)              */ \
    0x25, MAX_CONTACTS - 1,        /*     Logical Maximum               */ \
    0x09, 0x51,                    /*     Usage (Contact Identifier)    */ \
    0x81, 0x02,                    /*     Input (Data,Var,Abs)          */ \
    0x05, 0x01,                    /*     Usage Page (Generic Desktop)  */ \
    0x75, 0x10,                    /*     Report Size (16)              */ \
    0x55, 0x0E,                    /*     Unit Exponent (-2)            */ \
    0x65, 0x11,                    /*     Unit (cm)                     */ \
    0x35, 0x00,                    /*     Physical Minimum (0)          */ \
    0x26, 0xFF, 0x0F,              /*     Logical Maximum (X range)     */ \
    0x46, 0x46, 0x0A,              /*     Physical Maximum (2630)       */ \
    0x09, 0x30,                    /*     Usage (X)                     */ \
    0x81, 0x02,                    /*     Input (Data,Var,Abs)          */ \
    0x26, 0xFF, 0x0F,              /*     Logical Maximum (Y range)     */ \
    0x46, 0x68, 0x06,              /*     Physical Maximum (1640)       */ \
    0x09, 0x31,                    /*     Usage (Y)                     */ \
    0x81, 0x02,                    /*     Input (Data,Var,Abs)          */ \
    0xC0                           /*   End Collection                  */

static UCHAR mReportDescriptor[] = {
  0x05, 0x0D,                      // Usage Page (Digitizer)
  0x09, 0x04,                      // Usage (Touch Screen)
  0xA1, 0x01,                      // Collection (Application)
  0x85, REPORTID_TOUCH,            //   Report ID
  FINGER (0), FINGER (1), FINGER (2), FINGER (3), FINGER (4),
  FINGER (5), FINGER (6), FINGER (7), FINGER (8), FINGER (9),
  0x05, 0x0D,                      //   Usage Page (Digitizer)
  0x09, 0x54,                      //   Usage (Contact Count)
  0x75, 0x08,                      //   Report Size (8)
  0x95, 0x01,                      //   Report Count (1)
  0x15, 0x00,                      //   Logical Minimum (0)
  0x25, MAX_CONTACTS,              //   Logical Maximum
  0x81, 0x02,                      //   Input (Data,Var,Abs)
  0x85, REPORTID_MAXCOUNT,         //   Report ID
  0x09, 0x55,                      //   Usage (Contact Count Maximum)
  0x25, MAX_CONTACTS,              //   Logical Maximum
  0xB1, 0x02,                      //   Feature (Data,Var,Abs)
  0xC0                             // End Collection
};

static HID_DESCRIPTOR mHidDescriptor = {
  sizeof (HID_DESCRIPTOR),
  0x21,                            // HID descriptor type
  0x0100,                          // HID 1.0
  0x00,                            // country
  1,
  { { 0x22, sizeof (mReportDescriptor) } }
};

typedef struct {
  BOOLEAN  Active;                 // reported down last frame
  BOOLEAN  Down;
  USHORT   X;
  USHORT   Y;
} FINGER_STATE;

typedef struct {
  WDFQUEUE      ReportQueue;       // pended IOCTL_HID_READ_REPORT
  WDFIOTARGET   Spb;
  WDFINTERRUPT  Interrupt;
  LARGE_INTEGER Connection;

  USHORT        MsgAddr;           // T5
  UCHAR         MsgSize;
  BOOLEAN       IsT100;
  UCHAR         TouchFirstReport;  // report ID of contact 0
  UCHAR         TouchReports;      // contacts the object reports
  USHORT        XRange;
  USHORT        YRange;
  UCHAR         XDropBits;
  UCHAR         YDropBits;

  FINGER_STATE  Fingers[MAX_CONTACTS];
  BOOLEAN       Reported;
  ULONG         Delivered;         // reports handed to HID class
  ULONG         Dropped;           // reports with no read pending
  ULONG         Ioctls;            // HID requests logged so far
} DEVICE_CONTEXT, *PDEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (DEVICE_CONTEXT, GetDeviceContext)

DRIVER_INITIALIZE                       DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD               EvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE         EvtPrepareHardware;
EVT_WDF_IO_QUEUE_IO_INTERNAL_DEVICE_CONTROL EvtInternalDeviceControl;
EVT_WDF_INTERRUPT_ISR                   EvtInterruptIsr;

static NTSTATUS
BringUpFailed (ULONG Step, NTSTATUS Status)
{
  OsLog ("mxt1664s: bring-up step %lu failed: 0x%08lx", Step, Status);
  return Status;
}

/**
  Write the 16-bit little-endian register address, then read Length bytes
  (one SPB sequence: repeated start).
**/
static NTSTATUS
MxtRead (PDEVICE_CONTEXT Ctx, USHORT Reg, PVOID Buffer, ULONG Length)
{
  UCHAR                          Addr[2];
  SPB_TRANSFER_LIST_AND_ENTRIES (2) Seq;
  WDF_MEMORY_DESCRIPTOR          Input;
  ULONG_PTR                      Bytes;

  Addr[0] = (UCHAR)Reg;
  Addr[1] = (UCHAR)(Reg >> 8);

  SPB_TRANSFER_LIST_INIT (&Seq.List, 2);
  Seq.List.Transfers[0] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE (SpbTransferDirectionToDevice, 0, Addr, sizeof (Addr));
  Seq.List.Transfers[1] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE (SpbTransferDirectionFromDevice, 0, Buffer, Length);

  WDF_MEMORY_DESCRIPTOR_INIT_BUFFER (&Input, &Seq, sizeof (Seq));
  return WdfIoTargetSendIoctlSynchronously (Ctx->Spb, NULL, IOCTL_SPB_EXECUTE_SEQUENCE,
                                            &Input, NULL, NULL, &Bytes);
}

/**
  Info block, object table and the touch object's ranges, as UEFI's
  TouchDxe (and Linux's mxts.c) read them.
**/
static NTSTATUS
MxtProbe (PDEVICE_CONTEXT Ctx)
{
  UCHAR     Info[MXT_INFO_SIZE];
  UCHAR     Table[MXT_MAX_OBJECTS * MXT_OBJECT_ENTRY_SIZE];
  UCHAR     Cfg[MXT_T100_CFG_SIZE];
  PUCHAR    Obj;
  USHORT    T9Addr;
  USHORT    T100Addr;
  UCHAR     T9Report = 0;
  UCHAR     T9Reports = 0;
  UCHAR     T100Report = 0;
  UCHAR     T100Reports = 0;
  UCHAR     ReportId;
  ULONG     Index;
  NTSTATUS  Status;

  Status = MxtRead (Ctx, 0, Info, sizeof (Info));
  if (!NT_SUCCESS (Status)) {
    return BringUpFailed (MXT_STEP_INFO_BLOCK, Status);
  }

  OsLog ("mxt1664s: family 0x%02x variant 0x%02x version 0x%02x build 0x%02x, %u objects",
         Info[0], Info[1], Info[2], Info[3], Info[6]);
  if (Info[6] > MXT_MAX_OBJECTS) {
    return BringUpFailed (MXT_STEP_OBJECT_TABLE, STATUS_BUFFER_OVERFLOW);
  }
  Status = MxtRead (Ctx, MXT_OBJECT_TABLE, Table, Info[6] * MXT_OBJECT_ENTRY_SIZE);
  if (!NT_SUCCESS (Status)) {
    return BringUpFailed (MXT_STEP_OBJECT_TABLE, Status);
  }

  T9Addr   = 0;
  T100Addr = 0;
  ReportId = 1;
  for (Index = 0; Index < Info[6]; Index++) {
    Obj = Table + Index * MXT_OBJECT_ENTRY_SIZE;
    if (Obj[0] == MXT_GEN_MESSAGEPROCESSOR_T5) {
      Ctx->MsgAddr = Obj[1] | (Obj[2] << 8);
      Ctx->MsgSize = (UCHAR)min (Obj[3] + 1, MXT_MAX_MSG_SIZE);
    } else if (Obj[0] == MXT_TOUCH_MULTITOUCHSCREEN_T9) {
      T9Addr    = Obj[1] | (Obj[2] << 8);
      T9Report  = ReportId;
      T9Reports = Obj[5];
    } else if (Obj[0] == MXT_TOUCH_MULTITOUCHSCREEN_T100) {
      T100Addr    = Obj[1] | (Obj[2] << 8);
      T100Report  = ReportId;
      T100Reports = Obj[5];
    }
    // Obj[4] = instances - 1, Obj[5] = report IDs per instance.
    ReportId = (UCHAR)(ReportId + Obj[5] * (Obj[4] + 1));
  }
  if ((Ctx->MsgAddr == 0) || ((T9Addr == 0) && (T100Addr == 0))) {
    return BringUpFailed (MXT_STEP_NO_T5_T9, STATUS_NOT_FOUND);
  }

  if (T100Addr != 0) {
    Ctx->IsT100           = TRUE;
    Ctx->TouchFirstReport = T100Report + MXT_T100_SCREEN_REPORTS;
    Ctx->TouchReports     = (T100Reports > MXT_T100_SCREEN_REPORTS) ? (T100Reports - MXT_T100_SCREEN_REPORTS) : 0;
    Status = MxtRead (Ctx, T100Addr, Cfg, MXT_T100_CFG_SIZE);
    if (!NT_SUCCESS (Status)) {
      return BringUpFailed (MXT_STEP_TOUCH_CONFIG, Status);
    }
    Ctx->XRange = Cfg[MXT_T100_XRANGE] | (Cfg[MXT_T100_XRANGE + 1] << 8);
    Ctx->YRange = Cfg[MXT_T100_YRANGE] | (Cfg[MXT_T100_YRANGE + 1] << 8);
    Ctx->XDropBits = Ctx->YDropBits = 0;
  } else {
    //
    // T9: [9] ORIENT (bit 0 swaps X/Y), [18-19] XRANGE, [20-21] YRANGE; a
    // range below 1024 is reported in 12-bit units (2 extra bits).
    //
    Ctx->IsT100           = FALSE;
    Ctx->TouchFirstReport = T9Report;
    Ctx->TouchReports     = T9Reports;
    Status = MxtRead (Ctx, T9Addr, Cfg, 22);
    if (!NT_SUCCESS (Status)) {
      return BringUpFailed (MXT_STEP_TOUCH_CONFIG, Status);
    }
    Ctx->XRange = Cfg[18] | (Cfg[19] << 8);
    Ctx->YRange = Cfg[20] | (Cfg[21] << 8);
    if ((Cfg[9] & 1) != 0) {
      Ctx->XDropBits = ((Cfg[21] & 0xC) == 0) ? 2 : 0;
      Ctx->YDropBits = ((Cfg[19] & 0xC) == 0) ? 2 : 0;
    } else {
      Ctx->XDropBits = ((Cfg[19] & 0xC) == 0) ? 2 : 0;
      Ctx->YDropBits = ((Cfg[21] & 0xC) == 0) ? 2 : 0;
    }
  }

  //
  // mxt_platform_data max_x/max_y = 4095 when the config does not say.
  //
  if (Ctx->XRange == 0) {
    Ctx->XRange = 4095;
  }
  if (Ctx->YRange == 0) {
    Ctx->YRange = 4095;
  }
  OsLog ("mxt1664s: T5 @0x%x (%u bytes), %s, contacts from report %u (%u)",
         Ctx->MsgAddr, Ctx->MsgSize, Ctx->IsT100 ? "T100" : "T9",
         Ctx->TouchFirstReport, Ctx->TouchReports);
  return STATUS_SUCCESS;
}

/**
  Put the real ranges into every finger's Logical Maximum (X, Y).
**/
static VOID
FixupReportDescriptor (PDEVICE_CONTEXT Ctx)
{
  ULONG  Index;

  for (Index = 0; Index + 7 < sizeof (mReportDescriptor); Index++) {
    // Logical Maximum (16-bit), Physical Maximum (16-bit), Usage (X or Y).
    if ((mReportDescriptor[Index] == 0x26) && (mReportDescriptor[Index + 3] == 0x46) &&
        (mReportDescriptor[Index + 6] == 0x09)) {
      USHORT  Max = (mReportDescriptor[Index + 7] == 0x30) ? Ctx->XRange : Ctx->YRange;
      mReportDescriptor[Index + 1] = (UCHAR)Max;
      mReportDescriptor[Index + 2] = (UCHAR)(Max >> 8);
    }
  }
}

static VOID
MxtHandleMessage (PDEVICE_CONTEXT Ctx, PUCHAR Msg)
{
  ULONG          Id;
  FINGER_STATE  *Finger;

  if ((Msg[0] < Ctx->TouchFirstReport) || (Msg[0] >= Ctx->TouchFirstReport + Ctx->TouchReports)) {
    return;
  }
  Id = Msg[0] - Ctx->TouchFirstReport;
  if (Id >= MAX_CONTACTS) {
    return;
  }

  Finger = &Ctx->Fingers[Id];
  if (Ctx->IsT100) {
    Finger->Down = (Msg[1] & MXT_T100_DETECT) != 0;
    if (Finger->Down) {
      Finger->X = (USHORT)min (Msg[2] | (Msg[3] << 8), Ctx->XRange);
      Finger->Y = (USHORT)min (Msg[4] | (Msg[5] << 8), Ctx->YRange);
    }
  } else if ((Msg[1] & MXT_T9_RELEASE) != 0) {
    Finger->Down = FALSE;
  } else if ((Msg[1] & MXT_T9_DETECT) != 0) {
    Finger->Down = TRUE;
    Finger->X    = (USHORT)min ((((Msg[2] << 4) | (Msg[4] >> 4)) >> Ctx->XDropBits), Ctx->XRange);
    Finger->Y    = (USHORT)min ((((Msg[3] << 4) | (Msg[4] & 0xF)) >> Ctx->YDropBits), Ctx->YRange);
  }
}

/**
  One report with every finger that is down, plus each one lifted since
  the last report (tip 0) so Windows sees the release.
**/
static VOID
SendReport (PDEVICE_CONTEXT Ctx)
{
  HID_TOUCH_REPORT  Report;
  WDFREQUEST        Request;
  WDFMEMORY         Memory;
  ULONG             Index;
  UCHAR             Count;

  RtlZeroMemory (&Report, sizeof (Report));
  Report.ReportId = REPORTID_TOUCH;
  Count           = 0;
  for (Index = 0; Index < MAX_CONTACTS; Index++) {
    FINGER_STATE  *Finger = &Ctx->Fingers[Index];

    if (!Finger->Down && !Finger->Active) {
      continue;
    }
    Report.Contacts[Count].Tip       = Finger->Down ? 1 : 0;
    Report.Contacts[Count].ContactId = (UCHAR)Index;
    Report.Contacts[Count].X         = Finger->X;
    Report.Contacts[Count].Y         = Finger->Y;
    Finger->Active = Finger->Down;
    Count++;
  }
  if (Count == 0) {
    return;
  }
  Report.ContactCount = Count;

  // No reader waiting: the frame is dropped, the next one has the state.
  if (!NT_SUCCESS (WdfIoQueueRetrieveNextRequest (Ctx->ReportQueue, &Request))) {
    if ((Ctx->Dropped++ % 100) == 0) {
      OsLog ("mxt1664s: no HID read pending, %lu reports dropped", Ctx->Dropped);
    }
    return;
  }
  if ((Ctx->Delivered++ % 100) == 0) {
    OsLog ("mxt1664s: %lu reports delivered (contacts %u, first at %u,%u)", Ctx->Delivered,
           Count, Report.Contacts[0].X, Report.Contacts[0].Y);
  }
  if (NT_SUCCESS (WdfRequestRetrieveOutputMemory (Request, &Memory)) &&
      NT_SUCCESS (WdfMemoryCopyFromBuffer (Memory, 0, &Report, sizeof (Report)))) {
    WdfRequestCompleteWithInformation (Request, STATUS_SUCCESS, sizeof (Report));
  } else {
    WdfRequestComplete (Request, STATUS_BUFFER_TOO_SMALL);
  }
}

/**
  Passive-level ISR: drain the message queue over I2C, then report.
**/
BOOLEAN
EvtInterruptIsr (WDFINTERRUPT Interrupt, ULONG MessageId)
{
  PDEVICE_CONTEXT  Ctx;
  UCHAR            Msg[MXT_MAX_MSG_SIZE];
  ULONG            Count;
  BOOLEAN          Any;

  UNREFERENCED_PARAMETER (MessageId);

  Ctx = GetDeviceContext (WdfInterruptGetDevice (Interrupt));
  Any = FALSE;
  for (Count = 0; Count < MXT_MAX_DRAIN; Count++) {
    if (!NT_SUCCESS (MxtRead (Ctx, Ctx->MsgAddr, Msg, Ctx->MsgSize)) || (Msg[0] == MXT_REPORT_NONE)) {
      break;
    }
    MxtHandleMessage (Ctx, Msg);
    Any = TRUE;
  }

  if (Any) {
    if (!Ctx->Reported) {
      Ctx->Reported = TRUE;
      OsLog ("mxt1664s: first touch report");
    }
    SendReport (Ctx);
  }
  return TRUE;
}

static NTSTATUS
CopyToOutput (WDFREQUEST Request, PVOID Data, size_t Length)
{
  WDFMEMORY  Memory;
  size_t     Size;
  NTSTATUS   Status;

  Status = WdfRequestRetrieveOutputMemory (Request, &Memory);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  WdfMemoryGetBuffer (Memory, &Size);
  if (Size < Length) {
    return STATUS_BUFFER_TOO_SMALL;
  }
  Status = WdfMemoryCopyFromBuffer (Memory, 0, Data, Length);
  if (NT_SUCCESS (Status)) {
    WdfRequestSetInformation (Request, Length);
  }
  return Status;
}

/**
  IOCTL_HID_GET_FEATURE through mshidkmdf: the report ID comes in the input
  buffer when there is one (UMDF-style), otherwise as the first byte of the
  output buffer (what RT 8.1 sends: input 0, output = the report buffer).
**/
static NTSTATUS
GetFeature (PDEVICE_CONTEXT Ctx, WDFREQUEST Request)
{
  WDFMEMORY            Memory;
  PUCHAR               Buffer;
  size_t               Size;
  UCHAR                ReportId;
  HID_MAXCOUNT_REPORT  Report;

  ReportId = 0;
  if (NT_SUCCESS (WdfRequestRetrieveInputMemory (Request, &Memory))) {
    Buffer = WdfMemoryGetBuffer (Memory, &Size);
    if (Size >= 1) {
      ReportId = Buffer[0];
    }
  } else if (NT_SUCCESS (WdfRequestRetrieveOutputMemory (Request, &Memory))) {
    Buffer = WdfMemoryGetBuffer (Memory, &Size);
    if (Size >= 1) {
      ReportId = Buffer[0];
    }
  }

  if ((ReportId != REPORTID_MAXCOUNT) && (ReportId != 0)) {
    OsLog ("mxt1664s: GET_FEATURE for unknown report %u", ReportId);
    return STATUS_INVALID_PARAMETER;
  }
  if (Ctx->Ioctls < 40) {
    OsLog ("mxt1664s: GET_FEATURE report %u -> contact count maximum %u", ReportId, MAX_CONTACTS);
  }

  Report.ReportId = REPORTID_MAXCOUNT;
  Report.MaxCount = MAX_CONTACTS;
  return CopyToOutput (Request, &Report, sizeof (Report));
}

VOID
EvtInternalDeviceControl (WDFQUEUE Queue, WDFREQUEST Request, size_t OutputBufferLength,
                          size_t InputBufferLength, ULONG IoControlCode)
{
  PDEVICE_CONTEXT         Ctx;
  HID_DEVICE_ATTRIBUTES   Attributes;
  NTSTATUS                Status;

  UNREFERENCED_PARAMETER (OutputBufferLength);
  UNREFERENCED_PARAMETER (InputBufferLength);

  Ctx = GetDeviceContext (WdfIoQueueGetDevice (Queue));
  if ((Ctx->Ioctls < 40) && (IoControlCode != IOCTL_HID_READ_REPORT || Ctx->Ioctls < 4)) {
    Ctx->Ioctls++;
    OsLog ("mxt1664s: HID ioctl 0x%08lx (in %Iu, out %Iu)", IoControlCode, InputBufferLength, OutputBufferLength);
  }

  switch (IoControlCode) {
    case IOCTL_HID_GET_DEVICE_DESCRIPTOR:
      Status = CopyToOutput (Request, &mHidDescriptor, sizeof (mHidDescriptor));
      break;

    case IOCTL_HID_GET_REPORT_DESCRIPTOR:
      Status = CopyToOutput (Request, mReportDescriptor, sizeof (mReportDescriptor));
      break;

    case IOCTL_HID_GET_DEVICE_ATTRIBUTES:
      RtlZeroMemory (&Attributes, sizeof (Attributes));
      Attributes.Size          = sizeof (Attributes);
      Attributes.VendorID      = 0x03EB;     // Atmel
      Attributes.ProductID     = 0x1664;
      Attributes.VersionNumber = 0x0100;
      Status = CopyToOutput (Request, &Attributes, sizeof (Attributes));
      break;

    case IOCTL_HID_READ_REPORT:
      Status = WdfRequestForwardToIoQueue (Request, Ctx->ReportQueue);
      if (NT_SUCCESS (Status)) {
        return;
      }
      break;

    case IOCTL_HID_GET_FEATURE:
      Status = GetFeature (Ctx, Request);
      break;

    case IOCTL_HID_SET_FEATURE:
    case IOCTL_HID_ACTIVATE_DEVICE:
    case IOCTL_HID_DEACTIVATE_DEVICE:
      Status = STATUS_SUCCESS;
      break;

    default:
      Status = STATUS_NOT_SUPPORTED;
      break;
  }

  WdfRequestComplete (Request, Status);
}

NTSTATUS
EvtPrepareHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
{
  PDEVICE_CONTEXT                  Ctx;
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  ULONG                            Index;
  ULONG                            Count;
  ULONG_PTR                        Gsi;
  BOOLEAN                          Found;
  WDF_IO_TARGET_OPEN_PARAMS        Open;
  NTSTATUS                         Status;

  DECLARE_UNICODE_STRING_SIZE (Path, RESOURCE_HUB_PATH_SIZE);

  Ctx   = GetDeviceContext (Device);
  Gsi   = 0;
  Found = FALSE;
  Count = WdfCmResourceListGetCount (ResourcesTranslated);
  for (Index = 0; Index < Count; Index++) {
    Desc = WdfCmResourceListGetDescriptor (ResourcesTranslated, Index);
    if ((Desc->Type == CmResourceTypeConnection) &&
        (Desc->u.Connection.Class == CM_RESOURCE_CONNECTION_CLASS_SERIAL) &&
        (Desc->u.Connection.Type == CM_RESOURCE_CONNECTION_TYPE_SERIAL_I2C) && !Found) {
      Ctx->Connection.LowPart  = Desc->u.Connection.IdLowPart;
      Ctx->Connection.HighPart = Desc->u.Connection.IdHighPart;
      Found = TRUE;
    } else if (Desc->Type == CmResourceTypeInterrupt) {
      Gsi = WdfCmResourceListGetDescriptor (ResourcesRaw, Index)->u.Interrupt.Vector;
    }
  }

  OsLog ("mxt1664s: I2C connection 0x%08lx, interrupt %Iu, %lu resources",
         Ctx->Connection.LowPart, Gsi, Count);
#if LOAD_TEST
  KeBugCheckEx (MXT_BUGCHECK, Ctx->Connection.LowPart, Gsi, Count, 0);
#else
  UNREFERENCED_PARAMETER (Gsi);
#endif

  if (!Found) {
    return BringUpFailed (MXT_STEP_NO_CONNECTION, STATUS_NOT_FOUND);
  }

  Status = RESOURCE_HUB_CREATE_PATH_FROM_ID (&Path, Ctx->Connection.LowPart, Ctx->Connection.HighPart);
  OsLog ("mxt1664s: resource hub path 0x%08lx (%u bytes)", Status, Path.Length);
  if (NT_SUCCESS (Status)) {
    Status = WdfIoTargetCreate (Device, WDF_NO_OBJECT_ATTRIBUTES, &Ctx->Spb);
    OsLog ("mxt1664s: target create 0x%08lx", Status);
  }
  if (NT_SUCCESS (Status)) {
    WDF_IO_TARGET_OPEN_PARAMS_INIT_OPEN_BY_NAME (&Open, &Path, GENERIC_READ | GENERIC_WRITE);
    Status = WdfIoTargetOpen (Ctx->Spb, &Open);
    OsLog ("mxt1664s: target open 0x%08lx", Status);
  }
  if (!NT_SUCCESS (Status)) {
    return BringUpFailed (MXT_STEP_OPEN_TARGET, Status);
  }

  OsLog ("mxt1664s: reading the info block");
  Status = MxtProbe (Ctx);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  FixupReportDescriptor (Ctx);
  OsLog ("mxt1664s: range %ux%u, ready", Ctx->XRange + 1, Ctx->YRange + 1);
  return STATUS_SUCCESS;
}

NTSTATUS
EvtDeviceAdd (WDFDRIVER Driver, PWDFDEVICE_INIT DeviceInit)
{
  WDF_PNPPOWER_EVENT_CALLBACKS  PnpPower;
  WDF_OBJECT_ATTRIBUTES         Attributes;
  WDF_IO_QUEUE_CONFIG           QueueConfig;
  WDF_INTERRUPT_CONFIG          InterruptConfig;
  PDEVICE_CONTEXT               Ctx;
  WDFDEVICE                     Device;
  WDFQUEUE                      Queue;
  NTSTATUS                      Status;

  UNREFERENCED_PARAMETER (Driver);

  WdfFdoInitSetFilter (DeviceInit);

  WDF_PNPPOWER_EVENT_CALLBACKS_INIT (&PnpPower);
  PnpPower.EvtDevicePrepareHardware = EvtPrepareHardware;
  WdfDeviceInitSetPnpPowerEventCallbacks (DeviceInit, &PnpPower);

  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, DEVICE_CONTEXT);
  Status = WdfDeviceCreate (&DeviceInit, &Attributes, &Device);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  Ctx = GetDeviceContext (Device);

  //
  // mshidkmdf sends its HID requests down as internal device controls.
  //
  WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE (&QueueConfig, WdfIoQueueDispatchParallel);
  QueueConfig.EvtIoInternalDeviceControl = EvtInternalDeviceControl;
  Status = WdfIoQueueCreate (Device, &QueueConfig, WDF_NO_OBJECT_ATTRIBUTES, &Queue);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  WDF_IO_QUEUE_CONFIG_INIT (&QueueConfig, WdfIoQueueDispatchManual);
  Status = WdfIoQueueCreate (Device, &QueueConfig, WDF_NO_OBJECT_ATTRIBUTES, &Ctx->ReportQueue);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  //
  // CHG (GpioInt): passive-level handling so the ISR can do I2C.
  //
  WDF_INTERRUPT_CONFIG_INIT (&InterruptConfig, EvtInterruptIsr, NULL);
  InterruptConfig.PassiveHandling = TRUE;
  Status = WdfInterruptCreate (Device, &InterruptConfig, WDF_NO_OBJECT_ATTRIBUTES, &Ctx->Interrupt);
  if (!NT_SUCCESS (Status)) {
    return BringUpFailed (MXT_STEP_INTERRUPT, Status);
  }
  return STATUS_SUCCESS;
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG  Config;

  OsLogInit ();
  OsLog ("mxt1664s: DriverEntry");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  return WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, WDF_NO_HANDLE);
}
