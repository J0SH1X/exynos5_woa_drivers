/** @file
  MAX77802 PMIC for Windows RT 8.1 (KMDF 1.13): the only owner of the PMIC's
  I2C connection (HS-I2C3, 0x09; DSDT EXYN5016), shared with the drivers
  that need rails through the direct-call interface in common/ExPmicIf.h
  (device interface + IRP_MN_QUERY_INTERFACE, as exynosgpu's).

  The DSDT connection is exclusive: before this driver, mxt1664s (key
  backlight LDO23) opened it first and exynosgpu got STATUS_DEVICE_BUSY.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#define OSLOG_GLOBAL
#include <ExynosEnv.h>
#include <wdf.h>
#define RESHUB_USE_HELPER_ROUTINES
#include <reshub.h>
#include <spb.h>
#include <initguid.h>
#include "../common/ExPmicIf.h"

typedef struct {
  WDFIOTARGET    I2c;
  LARGE_INTEGER  Connection;
  FAST_MUTEX     Lock;
  ULONG          Errors;
} PMIC_CONTEXT, *PPMIC_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (PMIC_CONTEXT, GetPmic)

DRIVER_INITIALIZE                DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD        EvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE  EvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE  EvtReleaseHardware;

static NTSTATUS
I2cRead (PPMIC_CONTEXT Ctx, UCHAR Reg, PUCHAR Value)
{
  SPB_TRANSFER_LIST_AND_ENTRIES (2) Seq;
  WDF_MEMORY_DESCRIPTOR          In;

  SPB_TRANSFER_LIST_INIT (&Seq.List, 2);
  Seq.List.Transfers[0] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE (SpbTransferDirectionToDevice, 0, &Reg, 1);
  Seq.List.Transfers[1] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE (SpbTransferDirectionFromDevice, 0, Value, 1);
  WDF_MEMORY_DESCRIPTOR_INIT_BUFFER (&In, &Seq, sizeof (Seq));
  return WdfIoTargetSendIoctlSynchronously (Ctx->I2c, NULL, IOCTL_SPB_EXECUTE_SEQUENCE, &In, NULL, NULL, NULL);
}

static NTSTATUS
I2cWrite (PPMIC_CONTEXT Ctx, UCHAR Reg, UCHAR Value)
{
  UCHAR                  Buf[2] = { Reg, Value };
  WDF_MEMORY_DESCRIPTOR  In;

  WDF_MEMORY_DESCRIPTOR_INIT_BUFFER (&In, Buf, sizeof (Buf));
  return WdfIoTargetSendWriteSynchronously (Ctx->I2c, NULL, &In, NULL, NULL, NULL);
}

static VOID
LogError (PPMIC_CONTEXT Ctx, PCSTR Op, UCHAR Reg, NTSTATUS Status)
{
  if (!NT_SUCCESS (Status) && (Ctx->Errors++ < 20)) {
    OsLog ("exynospmic: %s 0x%02x - 0x%08lx", Op, Reg, Status);
  }
}

//
// ---------------------------------------------------------------------------
// Interface (common/ExPmicIf.h)
// ---------------------------------------------------------------------------
//

static NTSTATUS
IfRead (PVOID Context, UCHAR Reg, PUCHAR Value)
{
  PPMIC_CONTEXT  Ctx = (PPMIC_CONTEXT)Context;
  NTSTATUS       Status;

  if ((Ctx->I2c == NULL) || (KeGetCurrentIrql () != PASSIVE_LEVEL)) {
    return STATUS_DEVICE_NOT_READY;
  }
  ExAcquireFastMutex (&Ctx->Lock);
  Status = I2cRead (Ctx, Reg, Value);
  ExReleaseFastMutex (&Ctx->Lock);
  LogError (Ctx, "read", Reg, Status);
  return Status;
}

static NTSTATUS
IfWrite (PVOID Context, UCHAR Reg, UCHAR Value)
{
  PPMIC_CONTEXT  Ctx = (PPMIC_CONTEXT)Context;
  NTSTATUS       Status;

  if ((Ctx->I2c == NULL) || (KeGetCurrentIrql () != PASSIVE_LEVEL)) {
    return STATUS_DEVICE_NOT_READY;
  }
  ExAcquireFastMutex (&Ctx->Lock);
  Status = I2cWrite (Ctx, Reg, Value);
  ExReleaseFastMutex (&Ctx->Lock);
  LogError (Ctx, "write", Reg, Status);
  return Status;
}

static NTSTATUS
IfUpdate (PVOID Context, UCHAR Reg, UCHAR Mask, UCHAR Value)
{
  PPMIC_CONTEXT  Ctx = (PPMIC_CONTEXT)Context;
  UCHAR          Old = 0;
  NTSTATUS       Status;

  if ((Ctx->I2c == NULL) || (KeGetCurrentIrql () != PASSIVE_LEVEL)) {
    return STATUS_DEVICE_NOT_READY;
  }
  ExAcquireFastMutex (&Ctx->Lock);
  Status = I2cRead (Ctx, Reg, &Old);
  if (NT_SUCCESS (Status)) {
    Status = I2cWrite (Ctx, Reg, (UCHAR)((Old & ~Mask) | (Value & Mask)));
  }
  ExReleaseFastMutex (&Ctx->Lock);
  LogError (Ctx, "update", Reg, Status);
  return Status;
}

static VOID
IfReference (PVOID Context)
{
  UNREFERENCED_PARAMETER (Context);
}

//
// ---------------------------------------------------------------------------
// PnP
// ---------------------------------------------------------------------------
//

NTSTATUS
EvtPrepareHardware (WDFDEVICE Device, WDFCMRESLIST Raw, WDFCMRESLIST Translated)
{
  PPMIC_CONTEXT                    Ctx = GetPmic (Device);
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  WDF_IO_TARGET_OPEN_PARAMS        Open;
  ULONG                            Index;
  UCHAR                            Id = 0;
  NTSTATUS                         Status;
  DECLARE_UNICODE_STRING_SIZE (Path, RESOURCE_HUB_PATH_SIZE);

  UNREFERENCED_PARAMETER (Raw);

  for (Index = 0; Index < WdfCmResourceListGetCount (Translated); Index++) {
    Desc = WdfCmResourceListGetDescriptor (Translated, Index);
    if ((Desc->Type == CmResourceTypeConnection) &&
        (Desc->u.Connection.Class == CM_RESOURCE_CONNECTION_CLASS_SERIAL) &&
        (Desc->u.Connection.Type == CM_RESOURCE_CONNECTION_TYPE_SERIAL_I2C)) {
      Ctx->Connection.LowPart  = Desc->u.Connection.IdLowPart;
      Ctx->Connection.HighPart = Desc->u.Connection.IdHighPart;
    }
  }
  if (Ctx->Connection.QuadPart == 0) {
    OsLog ("exynospmic: no I2C connection in the resources");
    return STATUS_DEVICE_CONFIGURATION_ERROR;
  }
  Status = RESOURCE_HUB_CREATE_PATH_FROM_ID (&Path, Ctx->Connection.LowPart, Ctx->Connection.HighPart);
  if (NT_SUCCESS (Status) && (Ctx->I2c == NULL)) {
    Status = WdfIoTargetCreate (Device, WDF_NO_OBJECT_ATTRIBUTES, &Ctx->I2c);
    if (NT_SUCCESS (Status)) {
      WDF_IO_TARGET_OPEN_PARAMS_INIT_OPEN_BY_NAME (&Open, &Path, GENERIC_READ | GENERIC_WRITE);
      Status = WdfIoTargetOpen (Ctx->I2c, &Open);
      if (!NT_SUCCESS (Status)) {
        WdfObjectDelete (Ctx->I2c);
        Ctx->I2c = NULL;
      }
    }
  }
  if (NT_SUCCESS (Status)) {
    Status = I2cRead (Ctx, 0x00, &Id);           // DEVICE_ID
  }
  OsLog ("exynospmic: MAX77802 I2C open, ID 0x%02x - 0x%08lx", Id, Status);
  return Status;
}

NTSTATUS
EvtReleaseHardware (WDFDEVICE Device, WDFCMRESLIST Translated)
{
  PPMIC_CONTEXT  Ctx = GetPmic (Device);

  UNREFERENCED_PARAMETER (Translated);

  if (Ctx->I2c != NULL) {
    ExAcquireFastMutex (&Ctx->Lock);
    WdfObjectDelete (Ctx->I2c);
    Ctx->I2c = NULL;
    ExReleaseFastMutex (&Ctx->Lock);
  }
  return STATUS_SUCCESS;
}

NTSTATUS
EvtDeviceAdd (WDFDRIVER Driver, PWDFDEVICE_INIT DeviceInit)
{
  WDF_PNPPOWER_EVENT_CALLBACKS  PnpPower;
  WDF_OBJECT_ATTRIBUTES         Attributes;
  WDF_QUERY_INTERFACE_CONFIG    Qi;
  WDFDEVICE                     Device;
  PPMIC_CONTEXT                 Ctx;
  NTSTATUS                      Status;
  static EX_PMIC_INTERFACE      If;

  UNREFERENCED_PARAMETER (Driver);

  WDF_PNPPOWER_EVENT_CALLBACKS_INIT (&PnpPower);
  PnpPower.EvtDevicePrepareHardware = EvtPrepareHardware;
  PnpPower.EvtDeviceReleaseHardware = EvtReleaseHardware;
  WdfDeviceInitSetPnpPowerEventCallbacks (DeviceInit, &PnpPower);

  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, PMIC_CONTEXT);
  Status = WdfDeviceCreate (&DeviceInit, &Attributes, &Device);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  Ctx = GetPmic (Device);
  ExInitializeFastMutex (&Ctx->Lock);

  RtlZeroMemory (&If, sizeof (If));
  If.Header.Size                 = sizeof (If);
  If.Header.Version              = EX_PMIC_IF_VERSION;
  If.Header.Context              = Ctx;
  If.Header.InterfaceReference   = IfReference;
  If.Header.InterfaceDereference = IfReference;
  If.Read                        = IfRead;
  If.Write                       = IfWrite;
  If.Update                      = IfUpdate;
  WDF_QUERY_INTERFACE_CONFIG_INIT (&Qi, (PINTERFACE)&If, &GUID_EXYNOS_PMIC_INTERFACE, NULL);
  Status = WdfDeviceAddQueryInterface (Device, &Qi);
  if (NT_SUCCESS (Status)) {
    Status = WdfDeviceCreateDeviceInterface (Device, &GUID_EXYNOS_PMIC_INTERFACE, NULL);
  }
  OsLog ("exynospmic: device added, interface - 0x%08lx", Status);
  return Status;
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG  Config;

  OsLogInit ();
  OsLog ("exynospmic: DriverEntry (built " __DATE__ " " __TIME__ ")");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  return WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, WDF_NO_HANDLE);
}
