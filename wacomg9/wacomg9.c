/** @file
  Wacom G9PLL S Pen digitizer (v1awifi, SM-P900) as a HID pen for Windows
  RT 8.1 (KMDF 1.13).

  Stack like mxt1664s: mshidkmdf.sys is the function driver, this is its
  lower filter.  The digitizer is reached through SpbCx (I2cSerialBus on
  hs-i2c4, exynosi2c) and its IRQ is a GpioInt (GPX3(5), exynosgpio),
  handled at PASSIVE level so the ISR reads the packet over I2C itself.
  UEFI's TouchDxe powers the digitizer (PEN_LDO_EN) and leaves it on.

  The side button pressed while hovering is turned into a right click (a
  barrel + tip tap at the hover position); with the tip down it is the
  plain barrel switch.

  Packets (drivers/input/wacom/wacom_i2c_func.c, COM_COORD_NUM = 12 on V1A):
    [0] 0x80 in range, 0x40 eraser, 0x20 side button, 0x10 tip
    [1-2] X, [3-4] Y, [5-6] pressure (big endian)
  already in screen orientation (x_invert = y_invert = xy_switch = 0):
  X along the 2560-pixel side.

  Progress and failures go to the OS log ring.  Built with LOAD_TEST=1 it
  bugchecks 0x57414321 ("WAC!") in PrepareHardware with P1 = I2C
  connection ID, P2 = interrupt (raw GSI), P3 = resource count.

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
#define WAC_BUGCHECK  0x57414321

#define WACOM_COM_QUERY       0x2A
#define WACOM_QUERY_SIZE      9
#define WACOM_QUERY_DELAY_MS  100
#define WACOM_COORD_SIZE      12

#define WACOM_IN_RANGE  0x80
#define WACOM_ERASER    0x40
#define WACOM_SIDE      0x20
#define WACOM_TIP       0x10

//
// include/linux/wacom_i2c.h, CONFIG_V1A; the query can refine them.
//
#define WACOM_MAX_X         26266
#define WACOM_MAX_Y         16416
#define WACOM_MAX_PRESSURE  1023

#define REPORTID_PEN  1

//
// HID report bits.
//
#define PEN_TIP     0x01
#define PEN_BARREL  0x02
#define PEN_INVERT  0x04
#define PEN_ERASER  0x08
#define PEN_RANGE   0x10

#include <pshpack1.h>
typedef struct {
  UCHAR   ReportId;
  UCHAR   Buttons;
  USHORT  X;
  USHORT  Y;
  USHORT  Pressure;
} HID_PEN_REPORT;
#include <poppack.h>

//
// Pen digitizer; logical maxima patched from the query, physical size the
// panel's (263 x 164 mm, in 0.01 cm).
//
static UCHAR mReportDescriptor[] = {
  0x05, 0x0D,                      // Usage Page (Digitizer)
  0x09, 0x02,                      // Usage (Pen)
  0xA1, 0x01,                      // Collection (Application)
  0x85, REPORTID_PEN,              //   Report ID
  0x09, 0x20,                      //   Usage (Stylus)
  0xA1, 0x00,                      //   Collection (Physical)
  0x15, 0x00,                      //     Logical Minimum (0)
  0x25, 0x01,                      //     Logical Maximum (1)
  0x75, 0x01,                      //     Report Size (1)
  0x95, 0x05,                      //     Report Count (5)
  0x09, 0x42,                      //     Usage (Tip Switch)
  0x09, 0x44,                      //     Usage (Barrel Switch)
  0x09, 0x3C,                      //     Usage (Invert)
  0x09, 0x45,                      //     Usage (Eraser)
  0x09, 0x32,                      //     Usage (In Range)
  0x81, 0x02,                      //     Input (Data,Var,Abs)
  0x95, 0x03,                      //     Report Count (3)
  0x81, 0x03,                      //     Input (Cnst,Var,Abs)
  0x05, 0x01,                      //     Usage Page (Generic Desktop)
  0x75, 0x10,                      //     Report Size (16)
  0x95, 0x01,                      //     Report Count (1)
  0x55, 0x0E,                      //     Unit Exponent (-2)
  0x65, 0x11,                      //     Unit (cm)
  0x35, 0x00,                      //     Physical Minimum (0)
  0x26, 0x9A, 0x66,                //     Logical Maximum (26266)
  0x46, 0x46, 0x0A,                //     Physical Maximum (2630)
  0x09, 0x30,                      //     Usage (X)
  0x81, 0x02,                      //     Input (Data,Var,Abs)
  0x26, 0x20, 0x40,                //     Logical Maximum (16416)
  0x46, 0x68, 0x06,                //     Physical Maximum (1640)
  0x09, 0x31,                      //     Usage (Y)
  0x81, 0x02,                      //     Input (Data,Var,Abs)
  0x05, 0x0D,                      //     Usage Page (Digitizer)
  0x55, 0x00,                      //     Unit Exponent (0)
  0x65, 0x00,                      //     Unit (none)
  0x45, 0x00,                      //     Physical Maximum (0)
  0x26, 0xFF, 0x03,                //     Logical Maximum (1023)
  0x09, 0x30,                      //     Usage (Tip Pressure)
  0x81, 0x02,                      //     Input (Data,Var,Abs)
  0xC0,                            //   End Collection
  0xC0                             // End Collection
};

//
// Offsets of the patched Logical Maximum values in mReportDescriptor.
//
#define DESC_X_MAX         49
#define DESC_Y_MAX         59
#define DESC_PRESSURE_MAX  77

static HID_DESCRIPTOR mHidDescriptor = {
  sizeof (HID_DESCRIPTOR),
  0x21,                            // HID descriptor type
  0x0100,                          // HID 1.0
  0x00,                            // country
  1,
  { { 0x22, sizeof (mReportDescriptor) } }
};

enum {
  WAC_STEP_NO_CONNECTION = 1,
  WAC_STEP_OPEN_TARGET,
  WAC_STEP_QUERY,
  WAC_STEP_INTERRUPT,
};

typedef struct {
  WDFQUEUE       ReportQueue;      // pended IOCTL_HID_READ_REPORT
  WDFIOTARGET    Spb;
  WDFINTERRUPT   Interrupt;
  LARGE_INTEGER  Connection;

  USHORT         MaxX;
  USHORT         MaxY;
  USHORT         MaxPressure;
  HID_PEN_REPORT Last;
  BOOLEAN        Reported;
  ULONG          ButtonLogs;       // button changes logged so far
  ULONG          Delivered;
  ULONG          Dropped;
  ULONG          Ioctls;
} DEVICE_CONTEXT, *PDEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (DEVICE_CONTEXT, GetDeviceContext)

DRIVER_INITIALIZE                           DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD                   EvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE             EvtPrepareHardware;
EVT_WDF_IO_QUEUE_IO_INTERNAL_DEVICE_CONTROL EvtInternalDeviceControl;
EVT_WDF_INTERRUPT_ISR                       EvtInterruptIsr;

static NTSTATUS
BringUpFailed (ULONG Step, NTSTATUS Status)
{
  OsLog ("wacomg9: bring-up step %lu failed: 0x%08lx", Step, Status);
  return Status;
}

static NTSTATUS
SpbWrite (PDEVICE_CONTEXT Ctx, PVOID Buffer, ULONG Length)
{
  WDF_MEMORY_DESCRIPTOR  Input;
  ULONG_PTR              Bytes;

  WDF_MEMORY_DESCRIPTOR_INIT_BUFFER (&Input, Buffer, Length);
  return WdfIoTargetSendWriteSynchronously (Ctx->Spb, NULL, &Input, NULL, NULL, &Bytes);
}

static NTSTATUS
SpbRead (PDEVICE_CONTEXT Ctx, PVOID Buffer, ULONG Length)
{
  WDF_MEMORY_DESCRIPTOR  Output;
  ULONG_PTR              Bytes;
  NTSTATUS               Status;

  WDF_MEMORY_DESCRIPTOR_INIT_BUFFER (&Output, Buffer, Length);
  Status = WdfIoTargetSendReadSynchronously (Ctx->Spb, NULL, &Output, NULL, NULL, &Bytes);
  if (NT_SUCCESS (Status) && (Bytes != Length)) {
    Status = STATUS_DEVICE_PROTOCOL_ERROR;
  }
  return Status;
}

static VOID
Sleep (ULONG Ms)
{
  LARGE_INTEGER  Interval;

  Interval.QuadPart = -10000LL * Ms;
  KeDelayExecutionThread (KernelMode, FALSE, &Interval);
}

/**
  wacom_i2c_query(): who are you (also proves the digitizer is there).
**/
static NTSTATUS
WacomQuery (PDEVICE_CONTEXT Ctx)
{
  UCHAR     Command;
  UCHAR     Data[WACOM_QUERY_SIZE];
  ULONG     Try;
  NTSTATUS  Status;

  Status = STATUS_NO_SUCH_DEVICE;
  for (Try = 0; Try < 3; Try++) {
    Command = WACOM_COM_QUERY;
    Status  = SpbWrite (Ctx, &Command, 1);
    Sleep (WACOM_QUERY_DELAY_MS);
    if (!NT_SUCCESS (Status)) {
      continue;
    }
    Status = SpbRead (Ctx, Data, sizeof (Data));
    if (NT_SUCCESS (Status) && (Data[0] == 0x0F)) {
      OsLog ("wacomg9: max %u x %u, pressure %u, firmware 0x%02x%02x",
             (Data[1] << 8) | Data[2], (Data[3] << 8) | Data[4],
             (Data[5] << 8) | Data[6], Data[7], Data[8]);
      if ((((Data[5] << 8) | Data[6]) != 0) && (((Data[5] << 8) | Data[6]) <= 0xFFFF)) {
        Ctx->MaxPressure = (USHORT)((Data[5] << 8) | Data[6]);
      }
      return STATUS_SUCCESS;
    }
    if (NT_SUCCESS (Status)) {
      Status = STATUS_DEVICE_PROTOCOL_ERROR;
    }
  }
  return Status;
}

static VOID
PatchMax (ULONG Offset, USHORT Value)
{
  mReportDescriptor[Offset]     = (UCHAR)Value;
  mReportDescriptor[Offset + 1] = (UCHAR)(Value >> 8);
}

static VOID
SendReport (PDEVICE_CONTEXT Ctx, const HID_PEN_REPORT *Report)
{
  WDFREQUEST  Request;
  WDFMEMORY   Memory;

  // No reader waiting: the frame is dropped, the next one has the state.
  if (!NT_SUCCESS (WdfIoQueueRetrieveNextRequest (Ctx->ReportQueue, &Request))) {
    if ((Ctx->Dropped++ % 200) == 0) {
      OsLog ("wacomg9: no HID read pending, %lu reports dropped", Ctx->Dropped);
    }
    return;
  }
  if ((Ctx->Delivered++ % 500) == 0) {
    OsLog ("wacomg9: %lu reports delivered (buttons 0x%02x at %u,%u p %u)", Ctx->Delivered,
           Report->Buttons, Report->X, Report->Y, Report->Pressure);
  }
  if (NT_SUCCESS (WdfRequestRetrieveOutputMemory (Request, &Memory)) &&
      NT_SUCCESS (WdfMemoryCopyFromBuffer (Memory, 0, (PVOID)Report, sizeof (*Report)))) {
    WdfRequestCompleteWithInformation (Request, STATUS_SUCCESS, sizeof (*Report));
  } else {
    WdfRequestComplete (Request, STATUS_BUFFER_TOO_SMALL);
  }
}

/**
  Passive-level ISR: one coordinate packet per IRQ.
**/
BOOLEAN
EvtInterruptIsr (WDFINTERRUPT Interrupt, ULONG MessageId)
{
  PDEVICE_CONTEXT  Ctx;
  UCHAR            Data[WACOM_COORD_SIZE];
  HID_PEN_REPORT   Report;
  ULONG            X;
  ULONG            Y;

  UNREFERENCED_PARAMETER (MessageId);

  Ctx = GetDeviceContext (WdfInterruptGetDevice (Interrupt));
  if (!NT_SUCCESS (SpbRead (Ctx, Data, sizeof (Data)))) {
    return TRUE;
  }

  Report          = Ctx->Last;
  Report.ReportId = REPORTID_PEN;
  if ((Data[0] & WACOM_IN_RANGE) == 0) {
    Report.Buttons  = 0;           // out of range: last position, no switches
    Report.Pressure = 0;
  } else {
    X = ((ULONG)Data[1] << 8) | Data[2];
    Y = ((ULONG)Data[3] << 8) | Data[4];
    if ((X > Ctx->MaxX) || (Y > Ctx->MaxY)) {
      return TRUE;                 // wacom_i2c_coord_range(): outside the active area
    }
    Report.X        = (USHORT)X;
    Report.Y        = (USHORT)Y;
    Report.Pressure = (USHORT)min (((ULONG)Data[5] << 8) | Data[6], Ctx->MaxPressure);
    Report.Buttons  = PEN_RANGE;
    if ((Data[0] & WACOM_ERASER) != 0) {
      Report.Buttons |= ((Data[0] & WACOM_TIP) != 0) ? PEN_ERASER : PEN_INVERT;
    } else if ((Data[0] & WACOM_TIP) != 0) {
      Report.Buttons |= PEN_TIP;
    }
    if ((Data[0] & WACOM_SIDE) != 0) {
      Report.Buttons |= PEN_BARREL;
    }
    if ((Report.Buttons & (PEN_TIP | PEN_ERASER)) == 0) {
      Report.Pressure = 0;
    }
  }

  if (!Ctx->Reported) {
    Ctx->Reported = TRUE;
    OsLog ("wacomg9: first pen report (status 0x%02x at %u,%u)", Data[0], Report.X, Report.Y);
  }

  if (((Ctx->Last.Buttons ^ Report.Buttons) & (PEN_TIP | PEN_BARREL | PEN_INVERT | PEN_ERASER)) != 0) {
    if (Ctx->ButtonLogs < 100) {
      Ctx->ButtonLogs++;
      OsLog ("wacomg9: status 0x%02x -> buttons 0x%02x", Data[0], Report.Buttons);
    }

    //
    // Side button pressed while hovering = right click, like Samsung's
    // firmware does it: Windows only turns barrel + tip contact into a
    // right click, so send that tap (tip down with the barrel held, then
    // tip up) at the hover position.
    //
    if (((Report.Buttons & PEN_BARREL) != 0) && ((Ctx->Last.Buttons & PEN_BARREL) == 0) &&
        ((Report.Buttons & (PEN_TIP | PEN_ERASER | PEN_INVERT)) == 0)) {
      HID_PEN_REPORT  Tap;

      Tap          = Report;
      Tap.Buttons |= PEN_TIP;
      Tap.Pressure = Ctx->MaxPressure / 2;
      SendReport (Ctx, &Tap);
    }
  }

  Ctx->Last = Report;
  SendReport (Ctx, &Report);
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

VOID
EvtInternalDeviceControl (WDFQUEUE Queue, WDFREQUEST Request, size_t OutputBufferLength,
                          size_t InputBufferLength, ULONG IoControlCode)
{
  PDEVICE_CONTEXT        Ctx;
  HID_DEVICE_ATTRIBUTES  Attributes;
  NTSTATUS               Status;

  Ctx = GetDeviceContext (WdfIoQueueGetDevice (Queue));
  if ((Ctx->Ioctls < 30) && ((IoControlCode != IOCTL_HID_READ_REPORT) || (Ctx->Ioctls < 4))) {
    Ctx->Ioctls++;
    OsLog ("wacomg9: HID ioctl 0x%08lx (in %Iu, out %Iu)", IoControlCode, InputBufferLength, OutputBufferLength);
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
      Attributes.VendorID      = 0x056A;     // Wacom
      Attributes.ProductID     = 0x0900;     // G9PLL
      Attributes.VersionNumber = 0x0100;
      Status = CopyToOutput (Request, &Attributes, sizeof (Attributes));
      break;

    case IOCTL_HID_READ_REPORT:
      Status = WdfRequestForwardToIoQueue (Request, Ctx->ReportQueue);
      if (NT_SUCCESS (Status)) {
        return;
      }
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

  OsLog ("wacomg9: I2C connection 0x%08lx, interrupt %Iu, %lu resources",
         Ctx->Connection.LowPart, Gsi, Count);
#if LOAD_TEST
  KeBugCheckEx (WAC_BUGCHECK, Ctx->Connection.LowPart, Gsi, Count, 0);
#else
  UNREFERENCED_PARAMETER (Gsi);
#endif

  if (!Found) {
    return BringUpFailed (WAC_STEP_NO_CONNECTION, STATUS_NOT_FOUND);
  }

  Status = RESOURCE_HUB_CREATE_PATH_FROM_ID (&Path, Ctx->Connection.LowPart, Ctx->Connection.HighPart);
  if (NT_SUCCESS (Status)) {
    Status = WdfIoTargetCreate (Device, WDF_NO_OBJECT_ATTRIBUTES, &Ctx->Spb);
  }
  if (NT_SUCCESS (Status)) {
    WDF_IO_TARGET_OPEN_PARAMS_INIT_OPEN_BY_NAME (&Open, &Path, GENERIC_READ | GENERIC_WRITE);
    Status = WdfIoTargetOpen (Ctx->Spb, &Open);
  }
  if (!NT_SUCCESS (Status)) {
    return BringUpFailed (WAC_STEP_OPEN_TARGET, Status);
  }

  Ctx->MaxX        = WACOM_MAX_X;
  Ctx->MaxY        = WACOM_MAX_Y;
  Ctx->MaxPressure = WACOM_MAX_PRESSURE;
  Status = WacomQuery (Ctx);
  if (!NT_SUCCESS (Status)) {
    return BringUpFailed (WAC_STEP_QUERY, Status);
  }
  PatchMax (DESC_X_MAX, Ctx->MaxX);
  PatchMax (DESC_Y_MAX, Ctx->MaxY);
  PatchMax (DESC_PRESSURE_MAX, Ctx->MaxPressure);
  OsLog ("wacomg9: ready, %u x %u, pressure %u", Ctx->MaxX, Ctx->MaxY, Ctx->MaxPressure);
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
  // IRQ (GpioInt): passive-level handling so the ISR can do I2C.
  //
  WDF_INTERRUPT_CONFIG_INIT (&InterruptConfig, EvtInterruptIsr, NULL);
  InterruptConfig.PassiveHandling = TRUE;
  Status = WdfInterruptCreate (Device, &InterruptConfig, WDF_NO_OBJECT_ATTRIBUTES, &Ctx->Interrupt);
  if (!NT_SUCCESS (Status)) {
    return BringUpFailed (WAC_STEP_INTERRUPT, Status);
  }
  return STATUS_SUCCESS;
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG  Config;

  OsLogInit ();
  OsLog ("wacomg9: DriverEntry");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  return WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, WDF_NO_HANDLE);
}
