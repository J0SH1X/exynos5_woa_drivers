/** @file
  Bosch BMA255 accelerometer (accelerometer half of the BMI055) on
  v1awifi, as a HID Accelerometer 3D for Windows RT 8.1 (KMDF 1.13).

  Stack: mshidkmdf.sys is the function driver, this is its lower filter
  (common/hidsensor.h does the HID side); Windows' HID sensor class driver
  binds the collection (HID_DEVICE_UP:0020_U:0073).  The chip is on I2C3
  (exynosi2c, I2cSerialBus 0x18); INT1 (GPX1(3), GpioInt level high) is
  its data-ready interrupt, handled at PASSIVE level so the ISR reads the
  sample over I2C itself, which also clears the interrupt.

  Registers from the v1awifi kernel (drivers/sensors/bma255.c,
  bma255_reg.h) and the BMA255 datasheet: the chip comes up in suspend
  (PMU_LPW 0x80), so D0 entry soft-resets it, sets +-2 g and the data rate
  for the requested report interval and routes data-ready to INT1; D0
  exit suspends it.

  Axes: the mount matrix of mainline's exynos5420-galaxy-tab-pro-common.dtsi
  (x = -chip y, y = -chip x, z = chip z), signs per Windows' convention
  (flat, face up: z = -1 g); AXIS_* below if a test says otherwise.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <ntddk.h>
#include <wdf.h>
#include "oslog.h"
#include "spbreg.h"
#include "hidsensor.h"

#define BMA255_CHIP_ID      0x00
#define BMA255_ACCD_X_LSB   0x02
#define BMA255_PMU_RANGE    0x0F
#define BMA255_PMU_BW       0x10
#define BMA255_PMU_LPW      0x11
#define BMA255_SOFTRESET    0x14
#define BMA255_INT_EN_1     0x17
#define BMA255_INT_MAP_1    0x1A
#define BMA255_INT_OUT_CTRL 0x20
#define BMA255_INT_RST_LATCH 0x21

#define BMA255_ID           0xFA
#define RANGE_2G            0x03
#define LPW_NORMAL          0x00
#define LPW_SUSPEND         0x80
#define SOFTRESET_CMD       0xB6
#define INT_EN_DATA         0x10
#define INT_MAP_INT1_DATA   0x01
#define INT_OUT_PUSH_PULL_HIGH 0x05     // INT1 / INT2 active high, push-pull
#define INT_NON_LATCHED     0x00

//
// Bandwidth codes: the chip updates at twice the filter bandwidth.
//   0x08: 7.81 Hz -> 64 ms, 0x09: 32 ms, 0x0A: 16 ms, 0x0B: 8 ms
//
#define BW_64MS  0x08
#define BW_32MS  0x09
#define BW_16MS  0x0A
#define BW_8MS   0x0B

//
// Chip axis feeding each Windows axis (0 x, 1 y, 2 z) and its sign.
//
#define AXIS_X_SRC  1
#define AXIS_X_SIGN (-1)
#define AXIS_Y_SRC  0
#define AXIS_Y_SIGN (-1)
#define AXIS_Z_SRC  2
#define AXIS_Z_SIGN 1

#define DEFAULT_INTERVAL_MS   32
#define DEFAULT_SENSITIVITY   20          // milli-g
#define RAW_LOG_REPORTS       40          // raw samples logged for the axis check

//
// Report descriptor: Accelerometer 3D, values in milli-g (exponent -3).
//
static const UCHAR mReportDescriptor[] = {
  0x05, 0x20,                         // Usage Page (Sensor)
  0x09, 0x73,                         // Usage (Motion: Accelerometer 3D)
  0xA1, 0x00,                         // Collection (Physical)
  HIDSENSOR_PROPERTIES (0x1452, 0x0D),    // Acceleration | Change Sensitivity Abs, 10^-3
  HIDSENSOR_INPUT_STATE_EVENT (),
  HIDSENSOR_INPUT_S16 (0x0453, 0x0D),     // Acceleration Axis X
  HIDSENSOR_INPUT_S16 (0x0454, 0x0D),     // Acceleration Axis Y
  HIDSENSOR_INPUT_S16 (0x0455, 0x0D),     // Acceleration Axis Z
  0xC0                                // End Collection
};

#include <pshpack1.h>
typedef struct {
  HIDSENSOR_INPUT_HEADER  Header;
  SHORT                   Axis[3];      // milli-g
} ACCEL_REPORT;
#include <poppack.h>

typedef struct {
  HIDSENSOR     Sensor;
  WDFIOTARGET   Spb;
  WDFINTERRUPT  Interrupt;
  BOOLEAN       Running;            // chip in normal mode, data-ready on
  BOOLEAN       HaveLast;
  SHORT         Last[3];            // last reported, milli-g
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
#define SleepMs(Ms)                   SpbSleepMs (Ms)

static UCHAR
BandwidthFor (ULONG IntervalMs)
{
  if (IntervalMs >= 64) {
    return BW_64MS;
  }
  if (IntervalMs >= 32) {
    return BW_32MS;
  }
  if (IntervalMs >= 16) {
    return BW_16MS;
  }
  return BW_8MS;
}

/**
  Normal mode at the requested rate with data-ready on INT1, or suspend,
  as Windows' reporting / power state asks.  Called with the interrupt
  lock held (or the interrupt disconnected).
**/
static NTSTATUS
ChipApply (PDEVICE_CONTEXT Ctx)
{
  NTSTATUS  Status;

  if (!HidSensorWanted (&Ctx->Sensor)) {
    Status = RegWrite (Ctx, BMA255_INT_EN_1, 0);
    if (NT_SUCCESS (Status)) {
      Status = RegWrite (Ctx, BMA255_PMU_LPW, LPW_SUSPEND);
    }
    Ctx->Running = FALSE;
    return Status;
  }
  Status = RegWrite (Ctx, BMA255_PMU_LPW, LPW_NORMAL);
  if (NT_SUCCESS (Status)) {
    SleepMs (2);
    Status = RegWrite (Ctx, BMA255_PMU_BW, BandwidthFor (Ctx->Sensor.Feature.IntervalMs));
  }
  if (NT_SUCCESS (Status)) {
    Status = RegWrite (Ctx, BMA255_INT_EN_1, INT_EN_DATA);
  }
  Ctx->Running = NT_SUCCESS (Status);
  Ctx->HaveLast = FALSE;                // first sample after a change is always reported
  return Status;
}

/**
  Soft reset, +-2 g, data-ready routed to INT1 (push-pull, active high),
  then the mode Windows asks for.
**/
static NTSTATUS
ChipInit (PDEVICE_CONTEXT Ctx)
{
  UCHAR     Id;
  NTSTATUS  Status;

  Status = RegRead (Ctx, BMA255_CHIP_ID, &Id, 1);
  if (!NT_SUCCESS (Status) || (Id != BMA255_ID)) {
    OsLog ("bma255: chip ID 0x%02x (want 0x%02x) - 0x%08lx", Id, BMA255_ID, Status);
    return NT_SUCCESS (Status) ? STATUS_DEVICE_NOT_CONNECTED : Status;
  }
  RegWrite (Ctx, BMA255_SOFTRESET, SOFTRESET_CMD);   // no ACK guaranteed after the reset command
  SleepMs (5);                                        // start-up 1.8 ms
  Status = RegWrite (Ctx, BMA255_PMU_RANGE, RANGE_2G);
  if (NT_SUCCESS (Status)) {
    Status = RegWrite (Ctx, BMA255_INT_OUT_CTRL, INT_OUT_PUSH_PULL_HIGH);
  }
  if (NT_SUCCESS (Status)) {
    Status = RegWrite (Ctx, BMA255_INT_RST_LATCH, INT_NON_LATCHED);
  }
  if (NT_SUCCESS (Status)) {
    Status = RegWrite (Ctx, BMA255_INT_MAP_1, INT_MAP_INT1_DATA);
  }
  if (NT_SUCCESS (Status)) {
    Status = ChipApply (Ctx);
  }
  return Status;
}

/**
  One sample, in Windows axes and milli-g (+-2 g: 1024 counts per g).
**/
static NTSTATUS
ReadSample (PDEVICE_CONTEXT Ctx, SHORT Out[3])
{
  UCHAR     Raw[6];
  LONG      Chip[3];
  ULONG     Index;
  NTSTATUS  Status;

  Status = RegRead (Ctx, BMA255_ACCD_X_LSB, Raw, sizeof (Raw));
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  for (Index = 0; Index < 3; Index++) {
    Chip[Index] = ((SHORT)(Raw[Index * 2] | (Raw[Index * 2 + 1] << 8))) >> 4;   // 12-bit, left-aligned
    Chip[Index] = Chip[Index] * 1000 / 1024;
  }
  Out[0] = (SHORT)(AXIS_X_SIGN * Chip[AXIS_X_SRC]);
  Out[1] = (SHORT)(AXIS_Y_SIGN * Chip[AXIS_Y_SRC]);
  Out[2] = (SHORT)(AXIS_Z_SIGN * Chip[AXIS_Z_SRC]);
  if (Ctx->RawLogged < RAW_LOG_REPORTS) {
    Ctx->RawLogged++;
    OsLog ("bma255: chip x %ld y %ld z %ld mg -> Windows x %d y %d z %d", Chip[0], Chip[1], Chip[2], Out[0], Out[1],
           Out[2]);
  }
  return STATUS_SUCCESS;
}

static VOID
FillReport (PDEVICE_CONTEXT Ctx, ACCEL_REPORT *Report, const SHORT Axis[3], UCHAR Event)
{
  Report->Header.ReportId    = HIDSENSOR_REPORT_ID;
  Report->Header.SensorState = Ctx->Sensor.Feature.SensorState;
  Report->Header.Event       = Event;
  Report->Axis[0]            = Axis[0];
  Report->Axis[1]            = Axis[1];
  Report->Axis[2]            = Axis[2];
}

static BOOLEAN
Changed (PDEVICE_CONTEXT Ctx, const SHORT Axis[3])
{
  ULONG  Index;
  LONG   Delta;

  if (!Ctx->HaveLast) {
    return TRUE;
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
  Data-ready (PASSIVE level): read the sample (clears INT1), report it when
  it moved by the change sensitivity.
**/
BOOLEAN
EvtInterruptIsr (WDFINTERRUPT Interrupt, ULONG MessageId)
{
  PDEVICE_CONTEXT  Ctx;
  SHORT            Axis[3];
  ACCEL_REPORT     Report;

  UNREFERENCED_PARAMETER (MessageId);

  Ctx = GetDeviceContext (WdfInterruptGetDevice (Interrupt));
  if (!NT_SUCCESS (ReadSample (Ctx, Axis))) {
    return TRUE;
  }
  Ctx->Samples++;
  if (!Ctx->Running || !HidSensorWanted (&Ctx->Sensor) || !Changed (Ctx, Axis)) {
    return TRUE;
  }
  FillReport (Ctx, &Report, Axis, HIDSENSOR_EVENT_DATA_UPDATED);
  HidSensorSendInput (&Ctx->Sensor, &Report, sizeof (Report));
  RtlCopyMemory (Ctx->Last, Axis, sizeof (Ctx->Last));
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
    OsLog ("bma255: reconfiguring failed: 0x%08lx", Status);
  }
}

static ULONG
OnPoll (HIDSENSOR *Sensor, PVOID Buffer, ULONG Size)
{
  PDEVICE_CONTEXT  Ctx = CONTAINING_RECORD (Sensor, DEVICE_CONTEXT, Sensor);
  SHORT            Axis[3];
  NTSTATUS         Status;

  if (Size < sizeof (ACCEL_REPORT)) {
    return 0;
  }
  WdfInterruptAcquireLock (Ctx->Interrupt);
  Status = ReadSample (Ctx, Axis);
  WdfInterruptReleaseLock (Ctx->Interrupt);
  if (!NT_SUCCESS (Status)) {
    return 0;
  }
  FillReport (Ctx, (ACCEL_REPORT *)Buffer, Axis, HIDSENSOR_EVENT_POLL_RESPONSE);
  return sizeof (ACCEL_REPORT);
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
  OsLog ("bma255: D0 entry - 0x%08lx", Status);
  return Status;
}

NTSTATUS
EvtD0Exit (WDFDEVICE Device, WDF_POWER_DEVICE_STATE TargetState)
{
  PDEVICE_CONTEXT  Ctx = GetDeviceContext (Device);

  UNREFERENCED_PARAMETER (TargetState);

  RegWrite (Ctx, BMA255_INT_EN_1, 0);
  RegWrite (Ctx, BMA255_PMU_LPW, LPW_SUSPEND);
  Ctx->Running = FALSE;
  OsLog ("bma255: D0 exit (%lu samples, %lu reports, %lu without a reader)", Ctx->Samples, Ctx->Sensor.Delivered,
         Ctx->Sensor.Dropped);
  return STATUS_SUCCESS;
}

NTSTATUS
EvtPrepareHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
{
  UNREFERENCED_PARAMETER (ResourcesRaw);

  return SpbOpenI2c (Device, ResourcesTranslated, "bma255", &GetDeviceContext (Device)->Spb);
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
  Ctx->Sensor.Name                 = "bma255";
  Ctx->Sensor.ReportDescriptor     = mReportDescriptor;
  Ctx->Sensor.ReportDescriptorSize = sizeof (mReportDescriptor);
  Ctx->Sensor.VendorId             = 0x0108;        // Bosch Sensortec (as in their HID sensor hubs)
  Ctx->Sensor.ProductId            = 0x0255;
  Ctx->Sensor.Manufacturer         = L"Bosch Sensortec";
  Ctx->Sensor.Product              = L"BMA255 Accelerometer";
  Ctx->Sensor.ConfigChanged        = OnConfigChanged;
  Ctx->Sensor.Poll                 = OnPoll;
  Status = HidSensorInit (&Ctx->Sensor, Device, DEFAULT_INTERVAL_MS, DEFAULT_SENSITIVITY);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  // mshidkmdf sends its HID requests down as internal device controls.
  WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE (&QueueConfig, WdfIoQueueDispatchParallel);
  QueueConfig.EvtIoInternalDeviceControl = EvtInternalDeviceControl;
  Status = WdfIoQueueCreate (Device, &QueueConfig, WDF_NO_OBJECT_ATTRIBUTES, &Queue);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  // INT1 data-ready (GpioInt): passive-level handling so the ISR can do I2C.
  WDF_INTERRUPT_CONFIG_INIT (&InterruptConfig, EvtInterruptIsr, NULL);
  InterruptConfig.PassiveHandling = TRUE;
  Status = WdfInterruptCreate (Device, &InterruptConfig, WDF_NO_OBJECT_ATTRIBUTES, &Ctx->Interrupt);
  if (!NT_SUCCESS (Status)) {
    OsLog ("bma255: interrupt create failed: 0x%08lx", Status);
  }
  return Status;
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG  Config;

  OsLogInit ();
  OsLog ("bma255: DriverEntry (built " __DATE__ " " __TIME__ ")");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  return WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, WDF_NO_HANDLE);
}
