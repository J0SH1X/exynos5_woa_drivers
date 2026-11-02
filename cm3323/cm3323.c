/** @file
  Capella / Vishay CM3323 RGBW colour sensor on v1awifi, as a HID Ambient
  Light sensor for Windows RT 8.1 (KMDF 1.13).

  Stack: mshidkmdf.sys is the function driver, this is its lower filter
  (common/hidsensor.h does the HID side); Windows' HID sensor class driver
  binds the collection (HID_DEVICE_UP:0020_U:0041).  The chip is on I2C3
  (exynosi2c, I2cSerialBus 0x10).

  The CM3323 has no interrupt output (none is wired on this board either),
  so a timer is the only way to sample it: a passive-level one-shot WDF
  timer, re-armed at the report interval, running only while Windows wants
  data.

  Registers from the v1awifi kernel (drivers/sensors/cm3323.c) and mainline
  (drivers/iio/light/cm3323.c): CS_CONF1 (16-bit, written as a word) = 0:
  on, auto mode, 40 ms integration; R / G / B / W at 0x08..0x0B.  Lux from
  the green channel: 0.18 lx per count at 40 ms (Vishay CM3323E datasheet),
  up to about 11800 lx.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <ntddk.h>
#include <wdf.h>
#include "oslog.h"
#include "spbreg.h"
#include "hidsensor.h"

#define CM3323_CONF     0x00
#define CM3323_RED      0x08
#define CM3323_GREEN    0x09
#define CM3323_BLUE     0x0A
#define CM3323_WHITE    0x0B

#define CONF_ON_40MS    0x0000
#define CONF_SHUTDOWN   0x0001

#define MIN_INTERVAL_MS       50          // one 40 ms integration plus margin
#define DEFAULT_INTERVAL_MS   250
#define DEFAULT_SENSITIVITY   5           // lux
#define RAW_LOG_REPORTS       20

//
// Report descriptor: Ambient Light, illuminance in lux (exponent 0).
//
static const UCHAR mReportDescriptor[] = {
  0x05, 0x20,                         // Usage Page (Sensor)
  0x09, 0x41,                         // Usage (Light: Ambient Light)
  0xA1, 0x00,                         // Collection (Physical)
  HIDSENSOR_PROPERTIES (0x14D1, 0x00),    // Illuminance | Change Sensitivity Abs
  HIDSENSOR_INPUT_STATE_EVENT (),
  HIDSENSOR_INPUT_U16 (0x04D1, 0x00),     // Illuminance
  0xC0                                // End Collection
};

#include <pshpack1.h>
typedef struct {
  HIDSENSOR_INPUT_HEADER  Header;
  USHORT                  Lux;
} ALS_REPORT;
#include <poppack.h>

typedef struct {
  HIDSENSOR     Sensor;
  WDFIOTARGET   Spb;
  WDFTIMER      Timer;
  WDFWAITLOCK   Lock;               // chip access: timer, polls, configuration
  BOOLEAN       InD0;
  BOOLEAN       Running;
  BOOLEAN       HaveLast;
  USHORT        LastLux;
  ULONG         RawLogged;
  ULONG         Samples;
} DEVICE_CONTEXT, *PDEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (DEVICE_CONTEXT, GetDeviceContext)

DRIVER_INITIALIZE                           DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD                   EvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE             EvtPrepareHardware;
EVT_WDF_DEVICE_D0_ENTRY                     EvtD0Entry;
EVT_WDF_DEVICE_D0_EXIT                      EvtD0Exit;
EVT_WDF_IO_QUEUE_IO_INTERNAL_DEVICE_CONTROL EvtInternalDeviceControl;
EVT_WDF_TIMER                               EvtTimer;

static NTSTATUS
WriteConf (PDEVICE_CONTEXT Ctx, USHORT Conf)
{
  UCHAR  Word[2];

  Word[0] = (UCHAR)Conf;
  Word[1] = (UCHAR)(Conf >> 8);
  return SpbRegWriteBytes (Ctx->Spb, CM3323_CONF, Word, sizeof (Word));
}

static NTSTATUS
ReadChannel (PDEVICE_CONTEXT Ctx, UCHAR Reg, USHORT *Value)
{
  UCHAR     Word[2];
  NTSTATUS  Status;

  Status = SpbRegRead (Ctx->Spb, Reg, Word, sizeof (Word));
  *Value = (USHORT)(Word[0] | (Word[1] << 8));
  return Status;
}

static ULONG
IntervalMs (PDEVICE_CONTEXT Ctx)
{
  return max (Ctx->Sensor.Feature.IntervalMs, (ULONG)MIN_INTERVAL_MS);
}

static NTSTATUS
ReadLux (PDEVICE_CONTEXT Ctx, USHORT *Lux)
{
  USHORT    Red, Green, Blue, White;
  NTSTATUS  Status;

  Status = ReadChannel (Ctx, CM3323_GREEN, &Green);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  *Lux = (USHORT)(((ULONG)Green * 18) / 100);
  if (Ctx->RawLogged < RAW_LOG_REPORTS) {
    Ctx->RawLogged++;
    ReadChannel (Ctx, CM3323_RED, &Red);
    ReadChannel (Ctx, CM3323_BLUE, &Blue);
    ReadChannel (Ctx, CM3323_WHITE, &White);
    OsLog ("cm3323: R %u G %u B %u W %u -> %u lx", Red, Green, Blue, White, *Lux);
  }
  return STATUS_SUCCESS;
}

static VOID
FillReport (PDEVICE_CONTEXT Ctx, ALS_REPORT *Report, USHORT Lux, UCHAR Event)
{
  Report->Header.ReportId    = HIDSENSOR_REPORT_ID;
  Report->Header.SensorState = Ctx->Sensor.Feature.SensorState;
  Report->Header.Event       = Event;
  Report->Lux                = Lux;
}

/**
  On / off as Windows asks; the timer runs only while on.  Called with
  Lock held.
**/
static NTSTATUS
ChipApply (PDEVICE_CONTEXT Ctx)
{
  BOOLEAN   Want = Ctx->InD0 && HidSensorWanted (&Ctx->Sensor);
  NTSTATUS  Status;

  if (Want == Ctx->Running) {
    return STATUS_SUCCESS;
  }
  Status = WriteConf (Ctx, Want ? CONF_ON_40MS : CONF_SHUTDOWN);
  Ctx->Running  = Want && NT_SUCCESS (Status);
  Ctx->HaveLast = FALSE;
  if (Ctx->Running) {
    WdfTimerStart (Ctx->Timer, WDF_REL_TIMEOUT_IN_MS (IntervalMs (Ctx)));
  }
  return Status;
}

/**
  Timer (PASSIVE level): one sample, reported when it moved by the change
  sensitivity; re-armed while running.
**/
VOID
EvtTimer (WDFTIMER Timer)
{
  PDEVICE_CONTEXT  Ctx = GetDeviceContext (WdfTimerGetParentObject (Timer));
  ALS_REPORT       Report;
  USHORT           Lux;
  LONG             Delta;

  WdfWaitLockAcquire (Ctx->Lock, NULL);
  if (!Ctx->Running) {
    WdfWaitLockRelease (Ctx->Lock);
    return;
  }
  if (NT_SUCCESS (ReadLux (Ctx, &Lux))) {
    Ctx->Samples++;
    Delta = (LONG)Lux - Ctx->LastLux;
    if (!Ctx->HaveLast || ((Delta < 0 ? -Delta : Delta) >= (LONG)Ctx->Sensor.Feature.Sensitivity)) {
      FillReport (Ctx, &Report, Lux, HIDSENSOR_EVENT_DATA_UPDATED);
      HidSensorSendInput (&Ctx->Sensor, &Report, sizeof (Report));
      Ctx->LastLux  = Lux;
      Ctx->HaveLast = TRUE;
    }
  }
  WdfTimerStart (Ctx->Timer, WDF_REL_TIMEOUT_IN_MS (IntervalMs (Ctx)));
  WdfWaitLockRelease (Ctx->Lock);
}

static VOID
OnConfigChanged (HIDSENSOR *Sensor)
{
  PDEVICE_CONTEXT  Ctx = CONTAINING_RECORD (Sensor, DEVICE_CONTEXT, Sensor);
  NTSTATUS         Status;

  WdfWaitLockAcquire (Ctx->Lock, NULL);
  Status = ChipApply (Ctx);
  WdfWaitLockRelease (Ctx->Lock);
  if (!NT_SUCCESS (Status)) {
    OsLog ("cm3323: reconfiguring failed: 0x%08lx", Status);
  }
}

static ULONG
OnPoll (HIDSENSOR *Sensor, PVOID Buffer, ULONG Size)
{
  PDEVICE_CONTEXT  Ctx = CONTAINING_RECORD (Sensor, DEVICE_CONTEXT, Sensor);
  USHORT           Lux;
  NTSTATUS         Status;

  if (Size < sizeof (ALS_REPORT)) {
    return 0;
  }
  WdfWaitLockAcquire (Ctx->Lock, NULL);
  if (!Ctx->Running) {
    WriteConf (Ctx, CONF_ON_40MS);      // one integration, then back off
    SpbSleepMs (MIN_INTERVAL_MS);
  }
  Status = ReadLux (Ctx, &Lux);
  if (!Ctx->Running) {
    WriteConf (Ctx, CONF_SHUTDOWN);
  }
  WdfWaitLockRelease (Ctx->Lock);
  if (!NT_SUCCESS (Status)) {
    return 0;
  }
  FillReport (Ctx, (ALS_REPORT *)Buffer, Lux, HIDSENSOR_EVENT_POLL_RESPONSE);
  return sizeof (ALS_REPORT);
}

VOID
EvtInternalDeviceControl (WDFQUEUE Queue, WDFREQUEST Request, size_t OutputBufferLength,
                          size_t InputBufferLength, ULONG IoControlCode)
{
  UNREFERENCED_PARAMETER (OutputBufferLength);
  UNREFERENCED_PARAMETER (InputBufferLength);

  HidSensorInternalIoctl (&GetDeviceContext (WdfIoQueueGetDevice (Queue))->Sensor, Request, IoControlCode);
}

NTSTATUS
EvtD0Entry (WDFDEVICE Device, WDF_POWER_DEVICE_STATE PreviousState)
{
  PDEVICE_CONTEXT  Ctx = GetDeviceContext (Device);
  USHORT           Conf;
  NTSTATUS         Status;

  UNREFERENCED_PARAMETER (PreviousState);

  WdfWaitLockAcquire (Ctx->Lock, NULL);
  Status = ReadChannel (Ctx, CM3323_CONF, &Conf);       // presence check: the chip has no ID register
  if (NT_SUCCESS (Status)) {
    Status = WriteConf (Ctx, CONF_SHUTDOWN);
  }
  Ctx->InD0    = NT_SUCCESS (Status);
  Ctx->Running = FALSE;
  if (NT_SUCCESS (Status)) {
    Status = ChipApply (Ctx);
  }
  WdfWaitLockRelease (Ctx->Lock);
  OsLog ("cm3323: D0 entry (CS_CONF1 was 0x%04x) - 0x%08lx", Conf, Status);
  return Status;
}

NTSTATUS
EvtD0Exit (WDFDEVICE Device, WDF_POWER_DEVICE_STATE TargetState)
{
  PDEVICE_CONTEXT  Ctx = GetDeviceContext (Device);

  UNREFERENCED_PARAMETER (TargetState);

  WdfWaitLockAcquire (Ctx->Lock, NULL);
  Ctx->InD0    = FALSE;
  Ctx->Running = FALSE;
  WriteConf (Ctx, CONF_SHUTDOWN);
  WdfWaitLockRelease (Ctx->Lock);
  WdfTimerStop (Ctx->Timer, TRUE);
  OsLog ("cm3323: D0 exit (%lu samples, %lu reports, %lu without a reader)", Ctx->Samples, Ctx->Sensor.Delivered,
         Ctx->Sensor.Dropped);
  return STATUS_SUCCESS;
}

NTSTATUS
EvtPrepareHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
{
  UNREFERENCED_PARAMETER (ResourcesRaw);

  return SpbOpenI2c (Device, ResourcesTranslated, "cm3323", &GetDeviceContext (Device)->Spb);
}

NTSTATUS
EvtDeviceAdd (WDFDRIVER Driver, PWDFDEVICE_INIT DeviceInit)
{
  WDF_PNPPOWER_EVENT_CALLBACKS  PnpPower;
  WDF_OBJECT_ATTRIBUTES         Attributes;
  WDF_IO_QUEUE_CONFIG           QueueConfig;
  WDF_TIMER_CONFIG              TimerConfig;
  PDEVICE_CONTEXT               Ctx;
  WDFDEVICE                     Device;
  WDFQUEUE                      Queue;
  NTSTATUS                      Status;

  UNREFERENCED_PARAMETER (Driver);

  WdfFdoInitSetFilter (DeviceInit);

  WDF_PNPPOWER_EVENT_CALLBACKS_INIT (&PnpPower);
  PnpPower.EvtDevicePrepareHardware = EvtPrepareHardware;
  PnpPower.EvtDeviceD0Entry         = EvtD0Entry;
  PnpPower.EvtDeviceD0Exit          = EvtD0Exit;
  WdfDeviceInitSetPnpPowerEventCallbacks (DeviceInit, &PnpPower);

  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, DEVICE_CONTEXT);
  Status = WdfDeviceCreate (&DeviceInit, &Attributes, &Device);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  Ctx = GetDeviceContext (Device);
  Ctx->Sensor.Name                 = "cm3323";
  Ctx->Sensor.ReportDescriptor     = mReportDescriptor;
  Ctx->Sensor.ReportDescriptorSize = sizeof (mReportDescriptor);
  Ctx->Sensor.VendorId             = 0x0AC0;        // placeholder IDs
  Ctx->Sensor.ProductId            = 0x3323;
  Ctx->Sensor.Manufacturer         = L"Capella Microsystems";
  Ctx->Sensor.Product              = L"CM3323 Ambient Light Sensor";
  Ctx->Sensor.ConfigChanged        = OnConfigChanged;
  Ctx->Sensor.Poll                 = OnPoll;
  Status = HidSensorInit (&Ctx->Sensor, Device, DEFAULT_INTERVAL_MS, DEFAULT_SENSITIVITY);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  Status = WdfWaitLockCreate (WDF_NO_OBJECT_ATTRIBUTES, &Ctx->Lock);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  // Passive-level one-shot timer (KMDF: passive timers cannot be periodic).
  WDF_TIMER_CONFIG_INIT (&TimerConfig, EvtTimer);
  WDF_OBJECT_ATTRIBUTES_INIT (&Attributes);
  Attributes.ParentObject   = Device;
  Attributes.ExecutionLevel = WdfExecutionLevelPassive;
  Status = WdfTimerCreate (&TimerConfig, &Attributes, &Ctx->Timer);
  if (!NT_SUCCESS (Status)) {
    OsLog ("cm3323: timer create failed: 0x%08lx", Status);
    return Status;
  }

  WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE (&QueueConfig, WdfIoQueueDispatchParallel);
  QueueConfig.EvtIoInternalDeviceControl = EvtInternalDeviceControl;
  return WdfIoQueueCreate (Device, &QueueConfig, WDF_NO_OBJECT_ATTRIBUTES, &Queue);
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG  Config;

  OsLogInit ();
  OsLog ("cm3323: DriverEntry (built " __DATE__ " " __TIME__ ")");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  return WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, WDF_NO_HANDLE);
}
