/** @file
  v1awifi (SM-P900) hardware buttons for Windows RT 8.1: a lower filter
  under mshidkmdf on the ACPI Windows Button Array device (PNP0C40), for
  which this RT 8.1 image has no inbox driver.

  ACPI (Exynos5420Pkg DSDT BTNS): four GpioInts on exynosgpio, both edges,
  in the button array order:
    0  power       GPX2(2)  -> HID System Control, System Power Down
    1  Windows     GPX0(5)  -> HID Keyboard, Left GUI (home key)
    2  volume up   GPX0(2)  -> HID Consumer, Volume Increment
    3  volume down GPX0(3)  -> HID Consumer, Volume Decrement
  All active low with pull-ups.

  An edge on any key starts a 15 ms debounce (passive-level ISR), then the
  levels of all four are read from the GPX0 / GPX2 data registers and every
  change is reported.  The data registers are read through a read-only
  mapping of the RT GPIO controller (exynosgpio owns it in ACPI, so it is
  not claimed again here); nothing is written there.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <ntddk.h>
#include <wdf.h>
#include <hidport.h>
#include "oslog.h"

#define GPIO_RT_BASE   0x13400000
#define GPX0_DAT       0xC04
#define GPX2_DAT       0xC44
#define DEBOUNCE_MS    15

enum { KEY_POWER, KEY_WINDOWS, KEY_VOLUP, KEY_VOLDOWN, KEY_COUNT };

typedef struct {
  const char  *Name;
  ULONG       DatOffset;
  ULONG       Bit;
} KEY;

static const KEY mKeys[KEY_COUNT] = {
  { "power",       GPX2_DAT, 2 },
  { "Windows",     GPX0_DAT, 5 },
  { "volume up",   GPX0_DAT, 2 },
  { "volume down", GPX0_DAT, 3 },
};

#define REPORTID_KEYBOARD  1
#define REPORTID_CONSUMER  2
#define REPORTID_SYSTEM    3

static const UCHAR mReportDescriptor[] = {
  //
  // Keyboard: modifier bits (Left GUI = the Windows key) + one key slot.
  //
  0x05, 0x01,                      // Usage Page (Generic Desktop)
  0x09, 0x06,                      // Usage (Keyboard)
  0xA1, 0x01,                      // Collection (Application)
  0x85, REPORTID_KEYBOARD,         //   Report ID
  0x05, 0x07,                      //   Usage Page (Keyboard)
  0x19, 0xE0,                      //   Usage Minimum (Left Control)
  0x29, 0xE7,                      //   Usage Maximum (Right GUI)
  0x15, 0x00,                      //   Logical Minimum (0)
  0x25, 0x01,                      //   Logical Maximum (1)
  0x75, 0x01,                      //   Report Size (1)
  0x95, 0x08,                      //   Report Count (8)
  0x81, 0x02,                      //   Input (Data,Var,Abs)
  0x19, 0x00,                      //   Usage Minimum (0)
  0x29, 0x65,                      //   Usage Maximum (0x65)
  0x25, 0x65,                      //   Logical Maximum (0x65)
  0x75, 0x08,                      //   Report Size (8)
  0x95, 0x01,                      //   Report Count (1)
  0x81, 0x00,                      //   Input (Data,Array,Abs)
  0xC0,                            // End Collection

  //
  // Consumer: volume up / down.
  //
  0x05, 0x0C,                      // Usage Page (Consumer)
  0x09, 0x01,                      // Usage (Consumer Control)
  0xA1, 0x01,                      // Collection (Application)
  0x85, REPORTID_CONSUMER,         //   Report ID
  0x09, 0xE9,                      //   Usage (Volume Increment)
  0x09, 0xEA,                      //   Usage (Volume Decrement)
  0x15, 0x00,                      //   Logical Minimum (0)
  0x25, 0x01,                      //   Logical Maximum (1)
  0x75, 0x01,                      //   Report Size (1)
  0x95, 0x02,                      //   Report Count (2)
  0x81, 0x02,                      //   Input (Data,Var,Abs)
  0x95, 0x06,                      //   Report Count (6)
  0x81, 0x03,                      //   Input (Const,Var,Abs)
  0xC0,                            // End Collection

  //
  // System control: power.
  //
  0x05, 0x01,                      // Usage Page (Generic Desktop)
  0x09, 0x80,                      // Usage (System Control)
  0xA1, 0x01,                      // Collection (Application)
  0x85, REPORTID_SYSTEM,           //   Report ID
  0x09, 0x81,                      //   Usage (System Power Down)
  0x15, 0x00,                      //   Logical Minimum (0)
  0x25, 0x01,                      //   Logical Maximum (1)
  0x75, 0x01,                      //   Report Size (1)
  0x95, 0x01,                      //   Report Count (1)
  0x81, 0x02,                      //   Input (Data,Var,Abs)
  0x95, 0x07,                      //   Report Count (7)
  0x81, 0x03,                      //   Input (Const,Var,Abs)
  0xC0,                            // End Collection
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
  WDFQUEUE      ReportQueue;       // pended IOCTL_HID_READ_REPORT
  WDFINTERRUPT  Interrupt[KEY_COUNT];
  PUCHAR        Gpio;              // RT GPIO controller, read only
  BOOLEAN       Down[KEY_COUNT];   // as last reported
  ULONG         Ioctls;
} DEVICE_CONTEXT, *PDEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (DEVICE_CONTEXT, GetDeviceContext)

DRIVER_INITIALIZE                           DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD                   EvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE             EvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE             EvtReleaseHardware;
EVT_WDF_IO_QUEUE_IO_INTERNAL_DEVICE_CONTROL EvtInternalDeviceControl;
EVT_WDF_INTERRUPT_ISR                       EvtInterruptIsr;

static BOOLEAN
KeyDown (PDEVICE_CONTEXT Ctx, ULONG Key)
{
  ULONG  Dat;

  Dat = READ_REGISTER_ULONG ((PULONG)(Ctx->Gpio + mKeys[Key].DatOffset));
  return ((Dat >> mKeys[Key].Bit) & 1) == 0;     // active low
}

static VOID
SendReport (PDEVICE_CONTEXT Ctx, const UCHAR *Report, ULONG Length)
{
  WDFREQUEST  Request;
  WDFMEMORY   Memory;

  if (!NT_SUCCESS (WdfIoQueueRetrieveNextRequest (Ctx->ReportQueue, &Request))) {
    OsLog ("exynosbtn: no HID read pending, report %u dropped", Report[0]);
    return;
  }
  if (NT_SUCCESS (WdfRequestRetrieveOutputMemory (Request, &Memory)) &&
      NT_SUCCESS (WdfMemoryCopyFromBuffer (Memory, 0, (PVOID)Report, Length))) {
    WdfRequestCompleteWithInformation (Request, STATUS_SUCCESS, Length);
  } else {
    WdfRequestComplete (Request, STATUS_BUFFER_TOO_SMALL);
  }
}

/**
  Report the keys whose state differs from what was last reported.
**/
static VOID
ReportChanges (PDEVICE_CONTEXT Ctx)
{
  BOOLEAN  Now[KEY_COUNT];
  UCHAR    Report[3];
  ULONG    Key;

  for (Key = 0; Key < KEY_COUNT; Key++) {
    Now[Key] = KeyDown (Ctx, Key);
    if (Now[Key] != Ctx->Down[Key]) {
      OsLog ("exynosbtn: %s %s", mKeys[Key].Name, Now[Key] ? "down" : "up");
    }
  }

  if (Now[KEY_WINDOWS] != Ctx->Down[KEY_WINDOWS]) {
    Report[0] = REPORTID_KEYBOARD;
    Report[1] = Now[KEY_WINDOWS] ? 0x08 : 0x00;  // Left GUI
    Report[2] = 0;
    SendReport (Ctx, Report, 3);
  }
  if ((Now[KEY_VOLUP] != Ctx->Down[KEY_VOLUP]) || (Now[KEY_VOLDOWN] != Ctx->Down[KEY_VOLDOWN])) {
    Report[0] = REPORTID_CONSUMER;
    Report[1] = (Now[KEY_VOLUP] ? 0x01 : 0) | (Now[KEY_VOLDOWN] ? 0x02 : 0);
    SendReport (Ctx, Report, 2);
  }
  if (Now[KEY_POWER] != Ctx->Down[KEY_POWER]) {
    Report[0] = REPORTID_SYSTEM;
    Report[1] = Now[KEY_POWER] ? 0x01 : 0x00;
    SendReport (Ctx, Report, 2);
  }

  for (Key = 0; Key < KEY_COUNT; Key++) {
    Ctx->Down[Key] = Now[Key];
  }
}

/**
  Passive-level ISR, shared by the four keys: debounce, then report.
**/
BOOLEAN
EvtInterruptIsr (WDFINTERRUPT Interrupt, ULONG MessageId)
{
  PDEVICE_CONTEXT  Ctx;
  LARGE_INTEGER    Delay;

  UNREFERENCED_PARAMETER (MessageId);

  Ctx = GetDeviceContext (WdfInterruptGetDevice (Interrupt));
  Delay.QuadPart = -(LONGLONG)DEBOUNCE_MS * 10000;
  KeDelayExecutionThread (KernelMode, FALSE, &Delay);
  if (Ctx->Gpio != NULL) {
    ReportChanges (Ctx);
  }
  return TRUE;
}

static NTSTATUS
CopyToOutput (WDFREQUEST Request, const VOID *Data, size_t Length)
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
  Status = WdfMemoryCopyFromBuffer (Memory, 0, (PVOID)Data, Length);
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
  if ((Ctx->Ioctls < 20) && (IoControlCode != IOCTL_HID_READ_REPORT || Ctx->Ioctls < 4)) {
    Ctx->Ioctls++;
    OsLog ("exynosbtn: HID ioctl 0x%08lx (in %Iu, out %Iu)", IoControlCode, InputBufferLength, OutputBufferLength);
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
      Attributes.VendorID      = 0x04E8;     // Samsung
      Attributes.ProductID     = 0x5420;
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
  PDEVICE_CONTEXT   Ctx;
  PHYSICAL_ADDRESS  Phys;
  ULONG             Index;
  ULONG             Interrupts;

  UNREFERENCED_PARAMETER (ResourcesRaw);

  Ctx = GetDeviceContext (Device);
  Interrupts = 0;
  for (Index = 0; Index < WdfCmResourceListGetCount (ResourcesTranslated); Index++) {
    if (WdfCmResourceListGetDescriptor (ResourcesTranslated, Index)->Type == CmResourceTypeInterrupt) {
      Interrupts++;
    }
  }

  Phys.QuadPart = GPIO_RT_BASE;
  Ctx->Gpio     = MmMapIoSpace (Phys, PAGE_SIZE, MmNonCached);
  if (Ctx->Gpio == NULL) {
    OsLog ("exynosbtn: cannot map the GPIO data registers");
    return STATUS_INSUFFICIENT_RESOURCES;
  }

  //
  // Start from the current levels, so a key held at boot is not reported.
  //
  for (Index = 0; Index < KEY_COUNT; Index++) {
    Ctx->Down[Index] = KeyDown (Ctx, Index);
  }
  OsLog ("exynosbtn: %lu interrupts, keys %u%u%u%u (power, Windows, up, down; 1 = down)", Interrupts,
         Ctx->Down[0], Ctx->Down[1], Ctx->Down[2], Ctx->Down[3]);
  return STATUS_SUCCESS;
}

NTSTATUS
EvtReleaseHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesTranslated)
{
  PDEVICE_CONTEXT  Ctx;

  UNREFERENCED_PARAMETER (ResourcesTranslated);

  Ctx = GetDeviceContext (Device);
  if (Ctx->Gpio != NULL) {
    MmUnmapIoSpace (Ctx->Gpio, PAGE_SIZE);
    Ctx->Gpio = NULL;
  }
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
  ULONG                         Key;
  NTSTATUS                      Status;

  UNREFERENCED_PARAMETER (Driver);

  WdfFdoInitSetFilter (DeviceInit);

  WDF_PNPPOWER_EVENT_CALLBACKS_INIT (&PnpPower);
  PnpPower.EvtDevicePrepareHardware = EvtPrepareHardware;
  PnpPower.EvtDeviceReleaseHardware = EvtReleaseHardware;
  WdfDeviceInitSetPnpPowerEventCallbacks (DeviceInit, &PnpPower);

  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, DEVICE_CONTEXT);
  Status = WdfDeviceCreate (&DeviceInit, &Attributes, &Device);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosbtn: WdfDeviceCreate failed: 0x%08lx", Status);
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
  // One interrupt object per GpioInt, in resource order.
  //
  for (Key = 0; Key < KEY_COUNT; Key++) {
    WDF_INTERRUPT_CONFIG_INIT (&InterruptConfig, EvtInterruptIsr, NULL);
    InterruptConfig.PassiveHandling = TRUE;
    Status = WdfInterruptCreate (Device, &InterruptConfig, WDF_NO_OBJECT_ATTRIBUTES, &Ctx->Interrupt[Key]);
    if (!NT_SUCCESS (Status)) {
      OsLog ("exynosbtn: interrupt %lu (%s) failed: 0x%08lx", Key, mKeys[Key].Name, Status);
      return Status;
    }
  }
  OsLog ("exynosbtn: device added");
  return STATUS_SUCCESS;
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG  Config;

  OsLogInit ();
  OsLog ("exynosbtn: DriverEntry");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  return WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, WDF_NO_HANDLE);
}
