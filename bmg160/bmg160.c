/** @file
  Bosch BMG160 gyroscope (gyroscope half of the BMI055) on v1awifi, as a
  HID Gyrometer 3D for Windows RT 8.1 (KMDF 1.13).

  Stack: mshidkmdf.sys is the function driver, this is its lower filter
  (common/hidsensor.h does the HID side); Windows' HID sensor class driver
  binds the collection (HID_DEVICE_UP:0020_U:0076).  The chip is on I2C3
  (exynosi2c, I2cSerialBus 0x68); its INT1 pin (GPX3(1), GpioInt rising
  edge) carries the data-ready pulse, handled at PASSIVE level so the ISR
  reads the sample over I2C itself.

  Registers from mainline Linux (drivers/iio/gyro/bmg160_core.c) and the
  BMG160 datasheet: +-2000 dps (16.4 counts per dps), data rate 100, 200
  or 400 Hz (the chip's slowest is 100 Hz, so reports are also limited to
  the requested interval), data-ready routed to INT1 (push-pull, active
  high, non-latched); suspend outside D0 and while Windows wants no data.

  Axes: the mount matrix of mainline's exynos5420-galaxy-tab-pro-common.dtsi
  (the accelerometer's, same die: x = -chip y, y = -chip x, z = chip z);
  AXIS_* below if a test says otherwise.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <ntddk.h>
#include <wdf.h>
#include "oslog.h"
#include "spbreg.h"
#include "hidsensor.h"

#define BMG160_CHIP_ID      0x00
#define BMG160_RATE_X_LSB   0x02
#define BMG160_RANGE        0x0F
#define BMG160_BW           0x10
#define BMG160_MODE_LPM1    0x11
#define BMG160_SOFTRESET    0x14
#define BMG160_INT_EN_0     0x15
#define BMG160_INT_EN_1     0x16
#define BMG160_INT_MAP_1    0x18
#define BMG160_INT_RST_LATCH 0x21

#define BMG160_ID           0x0F
#define RANGE_2000DPS       0x00
#define MODE_NORMAL         0x00
#define MODE_SUSPEND        0x80
#define SOFTRESET_CMD       0xB6
#define INT_EN_DATA         0x80
#define INT_PUSH_PULL_HIGH  0x05        // INT1 / INT2 active high, push-pull
#define INT_MAP_INT1_DATA   0x01
#define INT_NON_LATCHED     0x00

//
// Data rate / filter codes (bmg160_samp_freq_table).
//
#define BW_100HZ  0x07                  // 100 Hz, 32 Hz filter
#define BW_200HZ  0x06                  // 200 Hz, 64 Hz filter
#define BW_400HZ  0x03                  // 400 Hz, 47 Hz filter

#define AXIS_X_SRC  1
#define AXIS_X_SIGN (-1)
#define AXIS_Y_SRC  0
#define AXIS_Y_SIGN (-1)
#define AXIS_Z_SRC  2
#define AXIS_Z_SIGN 1

#define DEFAULT_INTERVAL_MS   16
#define DEFAULT_SENSITIVITY   10          // 0.1 dps units: 1 dps
#define RAW_LOG_REPORTS       40

//
// Report descriptor: Gyrometer 3D, angular velocity in 0.1 deg/s
// (exponent -1).
//
static const UCHAR mReportDescriptor[] = {
  0x05, 0x20,                         // Usage Page (Sensor)
  0x09, 0x76,                         // Usage (Motion: Gyrometer 3D)
  0xA1, 0x00,                         // Collection (Physical)
  HIDSENSOR_PROPERTIES (0x1456, 0x0F),    // Angular Velocity | Change Sensitivity Abs, 10^-1
  HIDSENSOR_INPUT_STATE_EVENT (),
  HIDSENSOR_INPUT_S16 (0x0457, 0x0F),     // Angular Velocity about X
  HIDSENSOR_INPUT_S16 (0x0458, 0x0F),     // about Y
  HIDSENSOR_INPUT_S16 (0x0459, 0x0F),     // about Z
  0xC0                                // End Collection
};

#include <pshpack1.h>
typedef struct {
  HIDSENSOR_INPUT_HEADER  Header;
  SHORT                   Axis[3];      // 0.1 deg/s
} GYRO_REPORT;
#include <poppack.h>

typedef struct {
  HIDSENSOR     Sensor;
  WDFIOTARGET   Spb;
  WDFINTERRUPT  Interrupt;
  BOOLEAN       Running;
  BOOLEAN       HaveLast;
  SHORT         Last[3];
  ULONGLONG     LastTime;           // interrupt time of the last report, 100 ns
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
EVT_WDF_INTERRUPT_ISR                       EvtInterruptIsr;

#define RegRead(Ctx, Reg, Buf, Len)  SpbRegRead ((Ctx)->Spb, (Reg), (Buf), (Len))
#define RegWrite(Ctx, Reg, Val)       SpbRegWrite ((Ctx)->Spb, (Reg), (Val))

static UCHAR
RateFor (ULONG IntervalMs)
{
  if (IntervalMs >= 10) {
    return BW_100HZ;
  }
  if (IntervalMs >= 5) {
    return BW_200HZ;
  }
  return BW_400HZ;
}

static NTSTATUS
ChipApply (PDEVICE_CONTEXT Ctx)
{
  NTSTATUS  Status;

  if (!HidSensorWanted (&Ctx->Sensor)) {
    Status = RegWrite (Ctx, BMG160_INT_EN_0, 0);
    if (NT_SUCCESS (Status)) {
      Status = RegWrite (Ctx, BMG160_MODE_LPM1, MODE_SUSPEND);
    }
    Ctx->Running = FALSE;
    return Status;
  }
  Status = RegWrite (Ctx, BMG160_MODE_LPM1, MODE_NORMAL);
  if (NT_SUCCESS (Status)) {
    SpbSleepMs (30);                    // start-up from suspend
    Status = RegWrite (Ctx, BMG160_BW, RateFor (Ctx->Sensor.Feature.IntervalMs));
  }
  if (NT_SUCCESS (Status)) {
    Status = RegWrite (Ctx, BMG160_INT_EN_0, INT_EN_DATA);
  }
  Ctx->Running  = NT_SUCCESS (Status);
  Ctx->HaveLast = FALSE;
  return Status;
}

static NTSTATUS
ChipInit (PDEVICE_CONTEXT Ctx)
{
  UCHAR     Id;
  NTSTATUS  Status;

  Status = RegRead (Ctx, BMG160_CHIP_ID, &Id, 1);
  if (!NT_SUCCESS (Status) || (Id != BMG160_ID)) {
    OsLog ("bmg160: chip ID 0x%02x (want 0x%02x) - 0x%08lx", Id, BMG160_ID, Status);
    return NT_SUCCESS (Status) ? STATUS_DEVICE_NOT_CONNECTED : Status;
  }
  RegWrite (Ctx, BMG160_SOFTRESET, SOFTRESET_CMD);
  SpbSleepMs (80);                      // BMG160_MAX_STARTUP_TIME_MS
  Status = RegWrite (Ctx, BMG160_RANGE, RANGE_2000DPS);
  if (NT_SUCCESS (Status)) {
    Status = RegWrite (Ctx, BMG160_INT_EN_1, INT_PUSH_PULL_HIGH);
  }
  if (NT_SUCCESS (Status)) {
    Status = RegWrite (Ctx, BMG160_INT_RST_LATCH, INT_NON_LATCHED);
  }
  if (NT_SUCCESS (Status)) {
    Status = RegWrite (Ctx, BMG160_INT_MAP_1, INT_MAP_INT1_DATA);
  }
  if (NT_SUCCESS (Status)) {
    Status = ChipApply (Ctx);
  }
  return Status;
}

/**
  One sample, in Windows axes and 0.1 deg/s (16.4 counts per deg/s).
**/
static NTSTATUS
ReadSample (PDEVICE_CONTEXT Ctx, SHORT Out[3])
{
  UCHAR     Raw[6];
  LONG      Chip[3];
  ULONG     Index;
  NTSTATUS  Status;

  Status = RegRead (Ctx, BMG160_RATE_X_LSB, Raw, sizeof (Raw));
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  for (Index = 0; Index < 3; Index++) {
    Chip[Index] = (SHORT)(Raw[Index * 2] | (Raw[Index * 2 + 1] << 8));
    Chip[Index] = Chip[Index] * 25 / 41;          // * 10 / 16.4
  }
  Out[0] = (SHORT)(AXIS_X_SIGN * Chip[AXIS_X_SRC]);
  Out[1] = (SHORT)(AXIS_Y_SIGN * Chip[AXIS_Y_SRC]);
  Out[2] = (SHORT)(AXIS_Z_SIGN * Chip[AXIS_Z_SRC]);
  if ((Ctx->RawLogged < RAW_LOG_REPORTS) && ((Ctx->Samples % 25) == 0)) {
    Ctx->RawLogged++;
    OsLog ("bmg160: chip x %ld y %ld z %ld (0.1 dps) -> Windows x %d y %d z %d", Chip[0], Chip[1], Chip[2], Out[0],
           Out[1], Out[2]);
  }
  return STATUS_SUCCESS;
}

static VOID
FillReport (PDEVICE_CONTEXT Ctx, GYRO_REPORT *Report, const SHORT Axis[3], UCHAR Event)
{
  Report->Header.ReportId    = HIDSENSOR_REPORT_ID;
  Report->Header.SensorState = Ctx->Sensor.Feature.SensorState;
  Report->Header.Event       = Event;
  Report->Axis[0]            = Axis[0];
  Report->Axis[1]            = Axis[1];
  Report->Axis[2]            = Axis[2];
}

static BOOLEAN
Due (PDEVICE_CONTEXT Ctx, const SHORT Axis[3])
{
  ULONG      Index;
  LONG       Delta;
  ULONGLONG  Now;

  if (!Ctx->HaveLast) {
    return TRUE;
  }
  Now = KeQueryInterruptTime ();
  if (Now - Ctx->LastTime < (ULONGLONG)Ctx->Sensor.Feature.IntervalMs * 10000) {
    return FALSE;
  }
  for (Index = 0; Index < 3; Index++) {
    Delta = Axis[Index] - Ctx->Last[Index];
    if ((Delta < 0 ? -Delta : Delta) >= (LONG)Ctx->Sensor.Feature.Sensitivity) {
      return TRUE;
    }
  }
  return FALSE;
}

/**
  Data-ready (PASSIVE level): read the sample, report it once per interval
  when it moved by the change sensitivity.
**/
BOOLEAN
EvtInterruptIsr (WDFINTERRUPT Interrupt, ULONG MessageId)
{
  PDEVICE_CONTEXT  Ctx;
  SHORT            Axis[3];
  GYRO_REPORT      Report;

  UNREFERENCED_PARAMETER (MessageId);

  Ctx = GetDeviceContext (WdfInterruptGetDevice (Interrupt));
  if (!NT_SUCCESS (ReadSample (Ctx, Axis))) {
    return TRUE;
  }
  Ctx->Samples++;
  if (!Ctx->Running || !HidSensorWanted (&Ctx->Sensor) || !Due (Ctx, Axis)) {
    return TRUE;
  }
  FillReport (Ctx, &Report, Axis, HIDSENSOR_EVENT_DATA_UPDATED);
  HidSensorSendInput (&Ctx->Sensor, &Report, sizeof (Report));
  RtlCopyMemory (Ctx->Last, Axis, sizeof (Ctx->Last));
  Ctx->LastTime = KeQueryInterruptTime ();
  Ctx->HaveLast = TRUE;
  return TRUE;
}

static VOID
OnConfigChanged (HIDSENSOR *Sensor)
{
  PDEVICE_CONTEXT  Ctx = CONTAINING_RECORD (Sensor, DEVICE_CONTEXT, Sensor);
  NTSTATUS         Status;

  WdfInterruptAcquireLock (Ctx->Interrupt);
  Status = ChipApply (Ctx);
  WdfInterruptReleaseLock (Ctx->Interrupt);
  if (!NT_SUCCESS (Status)) {
    OsLog ("bmg160: reconfiguring failed: 0x%08lx", Status);
  }
}

static ULONG
OnPoll (HIDSENSOR *Sensor, PVOID Buffer, ULONG Size)
{
  PDEVICE_CONTEXT  Ctx = CONTAINING_RECORD (Sensor, DEVICE_CONTEXT, Sensor);
  SHORT            Axis[3];
  NTSTATUS         Status;

  if (Size < sizeof (GYRO_REPORT)) {
    return 0;
  }
  WdfInterruptAcquireLock (Ctx->Interrupt);
  Status = ReadSample (Ctx, Axis);
  WdfInterruptReleaseLock (Ctx->Interrupt);
  if (!NT_SUCCESS (Status)) {
    return 0;
  }
  FillReport (Ctx, (GYRO_REPORT *)Buffer, Axis, HIDSENSOR_EVENT_POLL_RESPONSE);
  return sizeof (GYRO_REPORT);
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
  NTSTATUS  Status;

  UNREFERENCED_PARAMETER (PreviousState);

  Status = ChipInit (GetDeviceContext (Device));
  OsLog ("bmg160: D0 entry - 0x%08lx", Status);
  return Status;
}

NTSTATUS
EvtD0Exit (WDFDEVICE Device, WDF_POWER_DEVICE_STATE TargetState)
{
  PDEVICE_CONTEXT  Ctx = GetDeviceContext (Device);

  UNREFERENCED_PARAMETER (TargetState);

  RegWrite (Ctx, BMG160_INT_EN_0, 0);
  RegWrite (Ctx, BMG160_MODE_LPM1, MODE_SUSPEND);
  Ctx->Running = FALSE;
  OsLog ("bmg160: D0 exit (%lu samples, %lu reports, %lu without a reader)", Ctx->Samples, Ctx->Sensor.Delivered,
         Ctx->Sensor.Dropped);
  return STATUS_SUCCESS;
}

NTSTATUS
EvtPrepareHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
{
  UNREFERENCED_PARAMETER (ResourcesRaw);

  return SpbOpenI2c (Device, ResourcesTranslated, "bmg160", &GetDeviceContext (Device)->Spb);
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
  PnpPower.EvtDeviceD0Entry         = EvtD0Entry;
  PnpPower.EvtDeviceD0Exit          = EvtD0Exit;
  WdfDeviceInitSetPnpPowerEventCallbacks (DeviceInit, &PnpPower);

  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, DEVICE_CONTEXT);
  Status = WdfDeviceCreate (&DeviceInit, &Attributes, &Device);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  Ctx = GetDeviceContext (Device);
  Ctx->Sensor.Name                 = "bmg160";
  Ctx->Sensor.ReportDescriptor     = mReportDescriptor;
  Ctx->Sensor.ReportDescriptorSize = sizeof (mReportDescriptor);
  Ctx->Sensor.VendorId             = 0x0108;
  Ctx->Sensor.ProductId            = 0x0160;
  Ctx->Sensor.Manufacturer         = L"Bosch Sensortec";
  Ctx->Sensor.Product              = L"BMG160 Gyroscope";
  Ctx->Sensor.ConfigChanged        = OnConfigChanged;
  Ctx->Sensor.Poll                 = OnPoll;
  Status = HidSensorInit (&Ctx->Sensor, Device, DEFAULT_INTERVAL_MS, DEFAULT_SENSITIVITY);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE (&QueueConfig, WdfIoQueueDispatchParallel);
  QueueConfig.EvtIoInternalDeviceControl = EvtInternalDeviceControl;
  Status = WdfIoQueueCreate (Device, &QueueConfig, WDF_NO_OBJECT_ATTRIBUTES, &Queue);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  WDF_INTERRUPT_CONFIG_INIT (&InterruptConfig, EvtInterruptIsr, NULL);
  InterruptConfig.PassiveHandling = TRUE;
  Status = WdfInterruptCreate (Device, &InterruptConfig, WDF_NO_OBJECT_ATTRIBUTES, &Ctx->Interrupt);
  if (!NT_SUCCESS (Status)) {
    OsLog ("bmg160: interrupt create failed: 0x%08lx", Status);
  }
  return Status;
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG  Config;

  OsLogInit ();
  OsLog ("bmg160: DriverEntry (built " __DATE__ " " __TIME__ ")");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  return WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, WDF_NO_HANDLE);
}
