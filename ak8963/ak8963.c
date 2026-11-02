/** @file
  Asahi Kasei AK8963C magnetometer on v1awifi, as a HID Compass 3D for
  Windows RT 8.1 (KMDF 1.13).

  Stack: mshidkmdf.sys is the function driver, this is its lower filter
  (common/hidsensor.h does the HID side); Windows' HID sensor class driver
  binds the collection (HID_DEVICE_UP:0020_U:0083).  The chip is on I2C3
  (exynosi2c, I2cSerialBus 0x0C); DRDY (GPX3(3), GpioInt level high) stays
  up until the sample is read (ST2), handled at PASSIVE level so the ISR
  reads it over I2C itself.  UEFI's SensorDxe takes the chip out of reset
  (GPG2(0)).

  Registers from the v1awifi kernel (drivers/sensors/ak8963c.c,
  ak8963c_reg.h) and mainline (drivers/iio/magnetometer/ak8975.c): 16-bit
  output, 0.15 uT (1.5 mG) per count, scaled by the fuse ROM's sensitivity
  adjustment (ASA: H * ((ASA - 128) / 256 + 1)); continuous mode at 8 or
  100 Hz, power-down in between and outside D0.

  Reported: magnetic flux in milligauss and the heading to magnetic north
  in 0.1 degree, from x / y alone (the tablet lying flat: no tilt
  compensation, which would need the accelerometer, and no hard-iron
  calibration yet).

  Axes: the mount matrix of mainline's exynos5420-galaxy-tab-pro-common.dtsi
  (x = chip x, y = -chip y, z = -chip z); AXIS_* below if a test says
  otherwise.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <ntddk.h>
#include <wdf.h>
#include "oslog.h"
#include "spbreg.h"
#include "hidsensor.h"

#define AK8963_WIA      0x00
#define AK8963_ST1      0x02
#define AK8963_CNTL1    0x0A
#define AK8963_CNTL2    0x0B
#define AK8963_ASAX     0x10

#define AK8963_ID       0x48
#define ST1_DRDY        0x01
#define ST2_HOFL        0x08             // magnetic sensor overflow
#define CNTL1_16BIT     0x10
#define MODE_POWER_DOWN 0x00
#define MODE_CONT_8HZ   0x02
#define MODE_CONT_100HZ 0x06
#define MODE_FUSE_ROM   0x0F
#define CNTL2_SRST      0x01

#define AXIS_X_SRC  0
#define AXIS_X_SIGN 1
#define AXIS_Y_SRC  1
#define AXIS_Y_SIGN (-1)
#define AXIS_Z_SRC  2
#define AXIS_Z_SIGN (-1)

#define DEFAULT_INTERVAL_MS   100
#define DEFAULT_SENSITIVITY   20          // 0.1 degree units: 2 degrees of heading
#define RAW_LOG_REPORTS       30

//
// Report descriptor: Compass 3D; heading in 0.1 degree (exponent -1),
// flux in milligauss (exponent 0).
//
static const UCHAR mReportDescriptor[] = {
  0x05, 0x20,                         // Usage Page (Sensor)
  0x09, 0x83,                         // Usage (Orientation: Compass 3D)
  0xA1, 0x00,                         // Collection (Physical)
  HIDSENSOR_PROPERTIES (0x1477, 0x0F),    // Heading Magnetic North | Change Sensitivity Abs, 10^-1
  HIDSENSOR_INPUT_STATE_EVENT (),
  HIDSENSOR_INPUT_U16 (0x0477, 0x0F),     // Heading Magnetic North
  HIDSENSOR_INPUT_S16 (0x0485, 0x00),     // Magnetic Flux X Axis
  HIDSENSOR_INPUT_S16 (0x0486, 0x00),     // Magnetic Flux Y Axis
  HIDSENSOR_INPUT_S16 (0x0487, 0x00),     // Magnetic Flux Z Axis
  0xC0                                // End Collection
};

#include <pshpack1.h>
typedef struct {
  HIDSENSOR_INPUT_HEADER  Header;
  USHORT                  Heading;      // 0.1 degree, 0..3599
  SHORT                   Flux[3];      // milligauss
} COMPASS_REPORT;
#include <poppack.h>

typedef struct {
  HIDSENSOR     Sensor;
  WDFIOTARGET   Spb;
  WDFINTERRUPT  Interrupt;
  BOOLEAN       Running;
  BOOLEAN       HaveLast;
  USHORT        LastHeading;
  ULONGLONG     LastTime;
  UCHAR         Asa[3];             // fuse ROM sensitivity adjustment
  UCHAR         Mode;               // continuous mode in use
  ULONG         RawLogged;
  ULONG         Samples;
  ULONG         Overflows;
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

/**
  atan2 in 0.1 degree (0..3599, counter-clockwise from +x), integer only:
  octant reduction plus atan(t) ~ 45 t + 15.64 t (1 - t) degrees on
  0 <= t <= 1 (error below 0.3 degree).
**/
static ULONG
Atan2Tenths (LONG Y, LONG X)
{
  LONG  Ax = (X < 0) ? -X : X;
  LONG  Ay = (Y < 0) ? -Y : Y;
  LONG  T;
  LONG  A;

  if ((Ax == 0) && (Ay == 0)) {
    return 0;
  }
  if (Ay <= Ax) {
    T = (LONG)(((LONGLONG)Ay << 10) / Ax);                  // 0..1024
    A = (450 * T + (156 * T * (1024 - T)) / 1024) / 1024;    // 0..450
  } else {
    T = (LONG)(((LONGLONG)Ax << 10) / Ay);
    A = 900 - (450 * T + (156 * T * (1024 - T)) / 1024) / 1024;
  }
  if (X < 0) {
    A = 1800 - A;
  }
  if (Y < 0) {
    A = 3600 - A;
  }
  return (ULONG)(A % 3600);
}

static NTSTATUS
SetMode (PDEVICE_CONTEXT Ctx, UCHAR Mode)
{
  NTSTATUS  Status;

  // Every mode change goes through power-down (100 us).
  Status = RegWrite (Ctx, AK8963_CNTL1, MODE_POWER_DOWN);
  SpbSleepMs (1);
  if (NT_SUCCESS (Status) && (Mode != MODE_POWER_DOWN)) {
    Status = RegWrite (Ctx, AK8963_CNTL1, (Mode == MODE_FUSE_ROM) ? Mode : (CNTL1_16BIT | Mode));
  }
  return Status;
}

static NTSTATUS
ChipApply (PDEVICE_CONTEXT Ctx)
{
  UCHAR     Data[8];
  NTSTATUS  Status;

  if (!HidSensorWanted (&Ctx->Sensor)) {
    Ctx->Running = FALSE;
    return SetMode (Ctx, MODE_POWER_DOWN);
  }
  Ctx->Mode = (Ctx->Sensor.Feature.IntervalMs >= 100) ? MODE_CONT_8HZ : MODE_CONT_100HZ;
  Status = SetMode (Ctx, Ctx->Mode);
  if (NT_SUCCESS (Status)) {
    // A sample still pending would keep DRDY up: read through ST2.
    RegRead (Ctx, AK8963_ST1, Data, sizeof (Data));
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

  Status = RegRead (Ctx, AK8963_WIA, &Id, 1);
  if (!NT_SUCCESS (Status) || (Id != AK8963_ID)) {
    OsLog ("ak8963: WIA 0x%02x (want 0x%02x) - 0x%08lx", Id, AK8963_ID, Status);
    return NT_SUCCESS (Status) ? STATUS_DEVICE_NOT_CONNECTED : Status;
  }
  RegWrite (Ctx, AK8963_CNTL2, CNTL2_SRST);
  SpbSleepMs (2);
  Status = SetMode (Ctx, MODE_FUSE_ROM);
  if (NT_SUCCESS (Status)) {
    Status = RegRead (Ctx, AK8963_ASAX, Ctx->Asa, sizeof (Ctx->Asa));
  }
  if (NT_SUCCESS (Status)) {
    OsLog ("ak8963: sensitivity adjustment %u %u %u", Ctx->Asa[0], Ctx->Asa[1], Ctx->Asa[2]);
    Status = ChipApply (Ctx);
  }
  return Status;
}

/**
  Read ST1..ST2 (ends the sample, drops DRDY).  FALSE if no sample was
  ready or the sensor overflowed.
**/
static BOOLEAN
ReadSample (PDEVICE_CONTEXT Ctx, SHORT Flux[3], USHORT *Heading)
{
  UCHAR  Data[8];                     // ST1, HXL..HZH, ST2
  LONG   Chip[3];
  ULONG  Index;

  if (!NT_SUCCESS (RegRead (Ctx, AK8963_ST1, Data, sizeof (Data))) || ((Data[0] & ST1_DRDY) == 0)) {
    return FALSE;
  }
  if ((Data[7] & ST2_HOFL) != 0) {
    Ctx->Overflows++;
    return FALSE;
  }
  for (Index = 0; Index < 3; Index++) {
    Chip[Index] = (SHORT)(Data[1 + Index * 2] | (Data[2 + Index * 2] << 8));
    Chip[Index] = Chip[Index] * ((LONG)Ctx->Asa[Index] + 128) / 256;   // fuse ROM adjustment
    Chip[Index] = Chip[Index] * 3 / 2;                                   // 1.5 mG per count
  }
  for (Index = 0; Index < 3; Index++) {
    LONG  V = (Index == 0) ? AXIS_X_SIGN * Chip[AXIS_X_SRC] :
              (Index == 1) ? AXIS_Y_SIGN * Chip[AXIS_Y_SRC] : AXIS_Z_SIGN * Chip[AXIS_Z_SRC];
    Flux[Index] = (SHORT)((V > 32767) ? 32767 : (V < -32767) ? -32767 : V);
  }
  //
  // Lying flat, y (top edge) to magnetic north: flux along +y, heading 0;
  // turned clockwise by 90 degrees north is on -x.  Heading = atan2 (-x, y).
  //
  *Heading = (USHORT)((3600 + 900 - Atan2Tenths (Flux[1], -Flux[0])) % 3600);
  if (Ctx->RawLogged < RAW_LOG_REPORTS) {
    Ctx->RawLogged++;
    OsLog ("ak8963: chip x %ld y %ld z %ld mG -> Windows x %d y %d z %d, heading %u.%u", Chip[0], Chip[1], Chip[2],
           Flux[0], Flux[1], Flux[2], *Heading / 10, *Heading % 10);
  }
  return TRUE;
}

static VOID
FillReport (PDEVICE_CONTEXT Ctx, COMPASS_REPORT *Report, const SHORT Flux[3], USHORT Heading, UCHAR Event)
{
  Report->Header.ReportId    = HIDSENSOR_REPORT_ID;
  Report->Header.SensorState = Ctx->Sensor.Feature.SensorState;
  Report->Header.Event       = Event;
  Report->Heading            = Heading;
  Report->Flux[0]            = Flux[0];
  Report->Flux[1]            = Flux[1];
  Report->Flux[2]            = Flux[2];
}

static BOOLEAN
Due (PDEVICE_CONTEXT Ctx, USHORT Heading)
{
  LONG  Delta;

  if (!Ctx->HaveLast) {
    return TRUE;
  }
  if (KeQueryInterruptTime () - Ctx->LastTime < (ULONGLONG)Ctx->Sensor.Feature.IntervalMs * 10000) {
    return FALSE;
  }
  Delta = (LONG)Heading - Ctx->LastHeading;
  if (Delta < 0) {
    Delta = -Delta;
  }
  if (Delta > 1800) {
    Delta = 3600 - Delta;             // across north
  }
  return Delta >= (LONG)Ctx->Sensor.Feature.Sensitivity;
}

/**
  DRDY (PASSIVE level): read the sample, report it once per interval when
  the heading moved by the change sensitivity.
**/
BOOLEAN
EvtInterruptIsr (WDFINTERRUPT Interrupt, ULONG MessageId)
{
  PDEVICE_CONTEXT  Ctx;
  SHORT            Flux[3];
  USHORT           Heading;
  COMPASS_REPORT   Report;

  UNREFERENCED_PARAMETER (MessageId);

  Ctx = GetDeviceContext (WdfInterruptGetDevice (Interrupt));
  if (!ReadSample (Ctx, Flux, &Heading)) {
    return TRUE;
  }
  Ctx->Samples++;
  if (!Ctx->Running || !HidSensorWanted (&Ctx->Sensor) || !Due (Ctx, Heading)) {
    return TRUE;
  }
  FillReport (Ctx, &Report, Flux, Heading, HIDSENSOR_EVENT_DATA_UPDATED);
  HidSensorSendInput (&Ctx->Sensor, &Report, sizeof (Report));
  Ctx->LastHeading = Heading;
  Ctx->LastTime    = KeQueryInterruptTime ();
  Ctx->HaveLast    = TRUE;
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
    OsLog ("ak8963: reconfiguring failed: 0x%08lx", Status);
  }
}

/**
  A poll: one single measurement when not running continuously, otherwise
  the next continuous sample (both within 10 ms).
**/
static ULONG
OnPoll (HIDSENSOR *Sensor, PVOID Buffer, ULONG Size)
{
  PDEVICE_CONTEXT  Ctx = CONTAINING_RECORD (Sensor, DEVICE_CONTEXT, Sensor);
  SHORT            Flux[3];
  USHORT           Heading;
  BOOLEAN          Got = FALSE;
  ULONG            Wait;

  if (Size < sizeof (COMPASS_REPORT)) {
    return 0;
  }
  WdfInterruptAcquireLock (Ctx->Interrupt);
  if (!Ctx->Running) {
    RegWrite (Ctx, AK8963_CNTL1, CNTL1_16BIT | 0x01);   // single measurement (7.2 ms)
  }
  for (Wait = 0; (Wait < 20) && !Got; Wait++) {
    SpbSleepMs (1);
    Got = ReadSample (Ctx, Flux, &Heading);
  }
  WdfInterruptReleaseLock (Ctx->Interrupt);
  if (!Got) {
    return 0;
  }
  FillReport (Ctx, (COMPASS_REPORT *)Buffer, Flux, Heading, HIDSENSOR_EVENT_POLL_RESPONSE);
  return sizeof (COMPASS_REPORT);
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
  OsLog ("ak8963: D0 entry - 0x%08lx", Status);
  return Status;
}

NTSTATUS
EvtD0Exit (WDFDEVICE Device, WDF_POWER_DEVICE_STATE TargetState)
{
  PDEVICE_CONTEXT  Ctx = GetDeviceContext (Device);

  UNREFERENCED_PARAMETER (TargetState);

  SetMode (Ctx, MODE_POWER_DOWN);
  Ctx->Running = FALSE;
  OsLog ("ak8963: D0 exit (%lu samples, %lu overflows, %lu reports, %lu without a reader)", Ctx->Samples,
         Ctx->Overflows, Ctx->Sensor.Delivered, Ctx->Sensor.Dropped);
  return STATUS_SUCCESS;
}

NTSTATUS
EvtPrepareHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
{
  UNREFERENCED_PARAMETER (ResourcesRaw);

  return SpbOpenI2c (Device, ResourcesTranslated, "ak8963", &GetDeviceContext (Device)->Spb);
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
  Ctx->Sensor.Name                 = "ak8963";
  Ctx->Sensor.ReportDescriptor     = mReportDescriptor;
  Ctx->Sensor.ReportDescriptorSize = sizeof (mReportDescriptor);
  Ctx->Sensor.VendorId             = 0x0AC0;        // placeholder IDs: Asahi Kasei has no USB vendor ID here
  Ctx->Sensor.ProductId            = 0x8963;
  Ctx->Sensor.Manufacturer         = L"Asahi Kasei Microdevices";
  Ctx->Sensor.Product              = L"AK8963C Magnetometer";
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
    OsLog ("ak8963: interrupt create failed: 0x%08lx", Status);
  }
  return Status;
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG  Config;

  OsLogInit ();
  OsLog ("ak8963: DriverEntry (built " __DATE__ " " __TIME__ ")");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  return WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, WDF_NO_HANDLE);
}
