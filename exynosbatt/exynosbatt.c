/** @file
  v1awifi (SM-P900) battery for Windows RT 8.1: a battery class miniport
  (battc.sys, KMDF 1.13) over the MAX17050 fuel gauge, which also keeps the
  MAX77888 charger configured for the attached cable.

  The gauge and charger logic is the UEFI BatteryDxe's, built unchanged
  (batterycore.c, gpioi2c.c -> ../../uefi/Exynos5420Pkg/...): bit-banged I2C
  on GPB3(6)/(7) (bus 17, charger + MUIC) and GPB0(3)/(4) (bus 18, gauge),
  cable-based input / charge current, soft regulation, temperature hold,
  recharge.  A passive 5 s timer (one-shot, re-armed) runs it and tells the battery class when
  the power state or the capacity changed.

  ACPI: _HID EXYN5008, Memory32Fixed over the GPIO controller holding GPB0
  and GPB3 (0x14010000 + 0x1000).

  Capacities: the gauge counts mAh; Windows wants mWh, converted at the
  pack's nominal 3.8 V.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#define OSLOG_GLOBAL
#include <ExynosEnv.h>

#include <wdf.h>
#include <batclass.h>

#include "../../uefi/Exynos5420Pkg/Drivers/BatteryDxe/BatteryCore.h"
#include "../../uefi/Exynos5420Pkg/Drivers/I2CDxe/GpioI2c.h"

#define NOMINAL_MV       3800
#define POLL_MS          5000
#define BATTERY_TAG      1
#define CRITICAL_CENTI   500       // 5 %

typedef struct {
  WDFDEVICE    Device;
  WDFWAITLOCK  Lock;
  WDFTIMER     Timer;
  PVOID        ClassHandle;
  PUCHAR       GpioB;
  SIZE_T       GpioBSize;
  BOOLEAN      Present;
  ULONG        LastPowerState;
  ULONG        LastCapacity;
  ULONG        Ioctls;           // IOCTLs logged so far
} BATT_CONTEXT, *PBATT_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (BATT_CONTEXT, BattGetContext)

DRIVER_INITIALIZE                    DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD            EvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE      EvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE      EvtReleaseHardware;
EVT_WDFDEVICE_WDM_IRP_PREPROCESS     EvtPreprocessDeviceControl;
EVT_WDF_TIMER                        EvtPollTimer;

UINTN gExynosGpioBBase;

//
// The shared code's I2C, on the bit-banged buses.
//
BOOLEAN
EnvI2cRead (UINT32 Bus, UINT8 Addr, UINT8 Reg, UINT8 *Buf, UINTN Count)
{
  EXYNOS_I2C_MSG  Msg[2];

  Msg[0].Addr  = Addr;
  Msg[0].Flags = 0;
  Msg[0].Len   = 1;
  Msg[0].Buf   = &Reg;
  Msg[1].Addr  = Addr;
  Msg[1].Flags = I2C_M_RD;
  Msg[1].Len   = (UINT16)Count;
  Msg[1].Buf   = Buf;
  return GpioI2cXfer (Bus, Msg, 2) == 2;
}

BOOLEAN
EnvI2cWrite (UINT32 Bus, UINT8 Addr, UINT8 *Buf, UINTN Count)
{
  EXYNOS_I2C_MSG  Msg;

  Msg.Addr  = Addr;
  Msg.Flags = 0;
  Msg.Len   = (UINT16)Count;
  Msg.Buf   = Buf;
  return GpioI2cXfer (Bus, &Msg, 1) == 1;
}

static ULONG
MahToMwh (UINT32 Mah)
{
  return (ULONG)(((UINT64)Mah * NOMINAL_MV) / 1000);
}

static BOOLEAN
ReadStatus (PBATT_CONTEXT Ctx, EXYNOS_BATTERY_STATUS *S)
{
  BOOLEAN  Ok;

  WdfWaitLockAcquire (Ctx->Lock, NULL);
  Ok = BatteryCoreGetStatus (S);
  WdfWaitLockRelease (Ctx->Lock);
  return Ok;
}

static VOID
FillStatus (const EXYNOS_BATTERY_STATUS *S, PBATTERY_STATUS Out)
{
  Out->PowerState = 0;
  if (S->ExternalPower) {
    Out->PowerState |= BATTERY_POWER_ON_LINE;
  }
  if (S->AvgMilliAmps > 0) {
    Out->PowerState |= BATTERY_CHARGING;
  } else if (S->AvgMilliAmps < 0) {
    Out->PowerState |= BATTERY_DISCHARGING;
  }
  if ((S->SocCentiPercent < CRITICAL_CENTI) && !(Out->PowerState & BATTERY_CHARGING)) {
    Out->PowerState |= BATTERY_CRITICAL;
  }
  Out->Capacity = MahToMwh (S->RemainingMah);
  Out->Voltage  = S->MilliVolts;
  Out->Rate     = (LONG)(((LONGLONG)S->AvgMilliAmps * (LONGLONG)S->MilliVolts) / 1000);
}

//
// Battery class callbacks (PASSIVE_LEVEL).
//
static NTSTATUS
BattQueryTag (PVOID Context, PULONG BatteryTag)
{
  PBATT_CONTEXT  Ctx = Context;

  if (!Ctx->Present) {
    *BatteryTag = BATTERY_TAG_INVALID;
    return STATUS_NO_SUCH_DEVICE;
  }
  *BatteryTag = BATTERY_TAG;
  return STATUS_SUCCESS;
}

static NTSTATUS
CopyOut (PVOID Buffer, ULONG BufferLength, PULONG ReturnedLength, const VOID *Data, ULONG Size)
{
  *ReturnedLength = Size;
  if (BufferLength < Size) {
    return STATUS_BUFFER_TOO_SMALL;
  }
  RtlCopyMemory (Buffer, Data, Size);
  return STATUS_SUCCESS;
}

static NTSTATUS
BattQueryInformation (PVOID Context, ULONG BatteryTag, BATTERY_QUERY_INFORMATION_LEVEL Level, LONG AtRate,
                      PVOID Buffer, ULONG BufferLength, PULONG ReturnedLength)
{
  PBATT_CONTEXT            Ctx = Context;
  EXYNOS_BATTERY_STATUS    S;
  BATTERY_INFORMATION      Info;
  BATTERY_REPORTING_SCALE  Scale;
  ULONG                    Value;
  static const WCHAR       Name[]   = L"SM-P900 battery";
  static const WCHAR       Vendor[] = L"Samsung";
  static const WCHAR       Unique[] = L"SMP900-MAX17050";

  *ReturnedLength = 0;
  if (BatteryTag != BATTERY_TAG) {
    return STATUS_NO_SUCH_DEVICE;
  }
  if (!ReadStatus (Ctx, &S)) {
    return STATUS_DEVICE_NOT_READY;
  }

  switch (Level) {
    case BatteryInformation:
      RtlZeroMemory (&Info, sizeof (Info));
      Info.Capabilities        = BATTERY_SYSTEM_BATTERY;
      Info.Technology          = 1;                    // rechargeable
      RtlCopyMemory (Info.Chemistry, "LION", 4);
      Info.DesignedCapacity    = MahToMwh (S.DesignMah);
      Info.FullChargedCapacity = MahToMwh (S.FullMah);
      Info.DefaultAlert1       = Info.FullChargedCapacity / 10;
      Info.DefaultAlert2       = Info.FullChargedCapacity / 20;
      Info.CriticalBias        = 0;
      Info.CycleCount          = S.CycleCentis / 100;
      return CopyOut (Buffer, BufferLength, ReturnedLength, &Info, sizeof (Info));

    case BatteryGranularityInformation:
      Scale.Granularity = 1;                           // mWh
      Scale.Capacity    = MahToMwh (S.FullMah);
      return CopyOut (Buffer, BufferLength, ReturnedLength, &Scale, sizeof (Scale));

    case BatteryTemperature:
      Value = (ULONG)(S.DeciCelsius + 2732);           // 0.1 K
      return CopyOut (Buffer, BufferLength, ReturnedLength, &Value, sizeof (Value));

    case BatteryEstimatedTime:
      if (AtRate < 0) {
        Value = (ULONG)(((ULONGLONG)MahToMwh (S.RemainingMah) * 3600) / (ULONG)(-AtRate));
      } else if ((AtRate == 0) && !S.ExternalPower && (S.AvgMilliAmps < 0)) {
        Value = S.TimeToEmptyMin * 60;
      } else {
        Value = BATTERY_UNKNOWN_TIME;
      }
      return CopyOut (Buffer, BufferLength, ReturnedLength, &Value, sizeof (Value));

    case BatteryDeviceName:
      return CopyOut (Buffer, BufferLength, ReturnedLength, Name, sizeof (Name));
    case BatteryManufactureName:
      return CopyOut (Buffer, BufferLength, ReturnedLength, Vendor, sizeof (Vendor));
    case BatteryUniqueID:
      return CopyOut (Buffer, BufferLength, ReturnedLength, Unique, sizeof (Unique));

    default:
      return STATUS_INVALID_DEVICE_REQUEST;
  }
}

static NTSTATUS
BattQueryStatus (PVOID Context, ULONG BatteryTag, PBATTERY_STATUS BatteryStatus)
{
  PBATT_CONTEXT          Ctx = Context;
  EXYNOS_BATTERY_STATUS  S;

  if (BatteryTag != BATTERY_TAG) {
    return STATUS_NO_SUCH_DEVICE;
  }
  if (!ReadStatus (Ctx, &S)) {
    return STATUS_DEVICE_NOT_READY;
  }
  FillStatus (&S, BatteryStatus);
  return STATUS_SUCCESS;
}

static NTSTATUS
BattSetStatusNotify (PVOID Context, ULONG BatteryTag, PBATTERY_NOTIFY BatteryNotify)
{
  UNREFERENCED_PARAMETER (Context);
  UNREFERENCED_PARAMETER (BatteryNotify);

  // The poll notifies on every power state change and 1 % capacity step.
  return (BatteryTag == BATTERY_TAG) ? STATUS_SUCCESS : STATUS_NO_SUCH_DEVICE;
}

static NTSTATUS
BattDisableStatusNotify (PVOID Context)
{
  UNREFERENCED_PARAMETER (Context);
  return STATUS_SUCCESS;
}

static NTSTATUS
BattSetInformation (PVOID Context, ULONG BatteryTag, BATTERY_SET_INFORMATION_LEVEL Level, PVOID Buffer)
{
  UNREFERENCED_PARAMETER (Context);
  UNREFERENCED_PARAMETER (BatteryTag);
  UNREFERENCED_PARAMETER (Level);
  UNREFERENCED_PARAMETER (Buffer);
  return STATUS_NOT_SUPPORTED;
}

//
// Charger upkeep and change notification.
//
VOID
EvtPollTimer (WDFTIMER Timer)
{
  PBATT_CONTEXT          Ctx;
  EXYNOS_BATTERY_STATUS  S;
  BATTERY_STATUS         Status;
  BOOLEAN                Ok;
  ULONG                  Step;

  Ctx = BattGetContext (WdfTimerGetParentObject (Timer));

  WdfWaitLockAcquire (Ctx->Lock, NULL);
  BatteryCorePoll ();
  Ok = BatteryCoreGetStatus (&S);
  WdfWaitLockRelease (Ctx->Lock);

  //
  // One-shot timer (KMDF passive-level timers cannot be periodic), re-armed
  // here; WdfTimerStop in ReleaseHardware ends the chain.
  //
  WdfTimerStart (Timer, WDF_REL_TIMEOUT_IN_MS (POLL_MS));
  if (!Ok || (Ctx->ClassHandle == NULL)) {
    return;
  }

  FillStatus (&S, &Status);
  Step = MAX (MahToMwh (S.FullMah) / 100, 1);
  if ((Status.PowerState != Ctx->LastPowerState) ||
      (Status.Capacity + Step <= Ctx->LastCapacity) || (Status.Capacity >= Ctx->LastCapacity + Step)) {
    Ctx->LastPowerState = Status.PowerState;
    Ctx->LastCapacity   = Status.Capacity;
    BatteryClassStatusNotify (Ctx->ClassHandle);
  }
}

//
// IOCTLs go to the battery class first (it completes the ones it handles);
// whatever it does not know goes back to the framework, which fails it with
// STATUS_INVALID_DEVICE_REQUEST.  No IoSkipCurrentIrpStackLocation before
// WdfDeviceWdmDispatchPreprocessedIrp: the framework takes the IRP at our
// own stack location, and skipping would make it read the one above it
// (past the end of the array for the topmost driver).
//
NTSTATUS
EvtPreprocessDeviceControl (WDFDEVICE Device, PIRP Irp)
{
  PBATT_CONTEXT      Ctx;
  PIO_STACK_LOCATION Stack;
  NTSTATUS           Status;

  Ctx   = BattGetContext (Device);
  Stack = IoGetCurrentIrpStackLocation (Irp);
  if (Ctx->Ioctls < 40) {
    Ctx->Ioctls++;
    OsLog ("exynosbatt: ioctl 0x%08lx (in %lu, out %lu, %s)", Stack->Parameters.DeviceIoControl.IoControlCode,
           Stack->Parameters.DeviceIoControl.InputBufferLength,
           Stack->Parameters.DeviceIoControl.OutputBufferLength,
           (Ctx->ClassHandle != NULL) ? "class registered" : "class not registered");
  }
  if (Ctx->ClassHandle != NULL) {
    Status = BatteryClassIoctl (Ctx->ClassHandle, Irp);
    if (Status != STATUS_NOT_SUPPORTED) {
      return Status;
    }
  }
  return WdfDeviceWdmDispatchPreprocessedIrp (Device, Irp);
}

NTSTATUS
EvtPrepareHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
{
  PBATT_CONTEXT                    Ctx;
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  BATTERY_MINIPORT_INFO            Info;
  ULONG                            Index;
  NTSTATUS                         Status;

  UNREFERENCED_PARAMETER (ResourcesRaw);

  Ctx = BattGetContext (Device);
  for (Index = 0; Index < WdfCmResourceListGetCount (ResourcesTranslated); Index++) {
    Desc = WdfCmResourceListGetDescriptor (ResourcesTranslated, Index);
    if ((Desc->Type == CmResourceTypeMemory) && (Ctx->GpioB == NULL)) {
      Ctx->GpioBSize = Desc->u.Memory.Length;
      Ctx->GpioB     = MmMapIoSpace (Desc->u.Memory.Start, Ctx->GpioBSize, MmNonCached);
    }
  }
  if ((Ctx->GpioB == NULL) || (Ctx->GpioBSize < 0x100)) {
    OsLog ("exynosbatt: GPIO memory resource missing");
    return STATUS_DEVICE_CONFIGURATION_ERROR;
  }
  gExynosGpioBBase = (UINTN)Ctx->GpioB;

  WdfWaitLockAcquire (Ctx->Lock, NULL);
  Ctx->Present = BatteryCoreInit ();
  BatteryCorePoll ();
  WdfWaitLockRelease (Ctx->Lock);
  OsLog ("exynosbatt: gauge %s", Ctx->Present ? "found" : "not answering");

  RtlZeroMemory (&Info, sizeof (Info));
  Info.MajorVersion        = BATTERY_CLASS_MAJOR_VERSION;
  Info.MinorVersion        = BATTERY_CLASS_MINOR_VERSION;
  Info.Context             = Ctx;
  Info.QueryTag            = BattQueryTag;
  Info.QueryInformation    = BattQueryInformation;
  Info.SetInformation      = BattSetInformation;
  Info.QueryStatus         = BattQueryStatus;
  Info.SetStatusNotify     = BattSetStatusNotify;
  Info.DisableStatusNotify = BattDisableStatusNotify;
  Info.Pdo                 = WdfDeviceWdmGetPhysicalDevice (Device);
  Info.DeviceName          = NULL;
  Status = BatteryClassInitializeDevice (&Info, &Ctx->ClassHandle);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosbatt: BatteryClassInitializeDevice failed: 0x%08lx", Status);
    Ctx->ClassHandle = NULL;
    return Status;
  }

  OsLog ("exynosbatt: registered with the battery class");
  WdfTimerStart (Ctx->Timer, WDF_REL_TIMEOUT_IN_MS (POLL_MS));
  return STATUS_SUCCESS;
}

NTSTATUS
EvtReleaseHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesTranslated)
{
  PBATT_CONTEXT  Ctx;

  UNREFERENCED_PARAMETER (ResourcesTranslated);

  Ctx = BattGetContext (Device);
  WdfTimerStop (Ctx->Timer, TRUE);
  if (Ctx->ClassHandle != NULL) {
    BatteryClassUnload (Ctx->ClassHandle);
    Ctx->ClassHandle = NULL;
  }
  if (Ctx->GpioB != NULL) {
    MmUnmapIoSpace (Ctx->GpioB, Ctx->GpioBSize);
    Ctx->GpioB = NULL;
  }
  return STATUS_SUCCESS;
}

NTSTATUS
EvtDeviceAdd (WDFDRIVER Driver, PWDFDEVICE_INIT DeviceInit)
{
  WDF_PNPPOWER_EVENT_CALLBACKS  PnpPower;
  WDF_OBJECT_ATTRIBUTES         Attributes;
  WDF_TIMER_CONFIG              TimerConfig;
  WDFDEVICE                     Device;
  PBATT_CONTEXT                 Ctx;
  NTSTATUS                      Status;

  UNREFERENCED_PARAMETER (Driver);

  WdfDeviceInitSetDeviceType (DeviceInit, FILE_DEVICE_BATTERY);
  WdfDeviceInitSetPowerPolicyOwnership (DeviceInit, TRUE);

  WDF_PNPPOWER_EVENT_CALLBACKS_INIT (&PnpPower);
  PnpPower.EvtDevicePrepareHardware = EvtPrepareHardware;
  PnpPower.EvtDeviceReleaseHardware = EvtReleaseHardware;
  WdfDeviceInitSetPnpPowerEventCallbacks (DeviceInit, &PnpPower);

  Status = WdfDeviceInitAssignWdmIrpPreprocessCallback (DeviceInit, EvtPreprocessDeviceControl,
                                                        IRP_MJ_DEVICE_CONTROL, NULL, 0);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosbatt: IRP preprocess callback failed: 0x%08lx", Status);
    return Status;
  }

  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, BATT_CONTEXT);
  Status = WdfDeviceCreate (&DeviceInit, &Attributes, &Device);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosbatt: WdfDeviceCreate failed: 0x%08lx", Status);
    return Status;
  }
  Ctx = BattGetContext (Device);
  Ctx->Device = Device;

  Status = WdfWaitLockCreate (WDF_NO_OBJECT_ATTRIBUTES, &Ctx->Lock);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosbatt: WdfWaitLockCreate failed: 0x%08lx", Status);
    return Status;
  }

  WDF_TIMER_CONFIG_INIT (&TimerConfig, EvtPollTimer);
  TimerConfig.AutomaticSerialization = FALSE;
  WDF_OBJECT_ATTRIBUTES_INIT (&Attributes);
  Attributes.ParentObject   = Device;
  Attributes.ExecutionLevel = WdfExecutionLevelPassive;
  Status = WdfTimerCreate (&TimerConfig, &Attributes, &Ctx->Timer);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosbatt: WdfTimerCreate failed: 0x%08lx", Status);
    return Status;
  }
  OsLog ("exynosbatt: device added");
  return STATUS_SUCCESS;
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG  Config;
  NTSTATUS           Status;

  OsLogInit ();
  OsLog ("exynosbatt: DriverEntry");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  Status = WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, WDF_NO_HANDLE);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosbatt: WdfDriverCreate failed: 0x%08lx", Status);
  }
  return Status;
}
