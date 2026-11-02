/** @file
  Exynos5420 SPI controllers as SPB controllers for SpbCx, for v1awifi under
  Windows RT 8.1 (KMDF 1.13, SPB 1.0).  ACPI _HID EXYN500A:

    Memory32Fixed  the controller (SPI2: 0x12D40000, 0x100)
    Memory32Fixed  optional: the GPIO bank of the chip select.  Without it
                   the driver maps GPB1 (0x14010080, pin 2) itself, unclaimed:
                   BAT0 already claims that GPIO controller's page in ACPI.
  The chip select is a plain GPIO as on Linux (exynos_spi_cfg_cs()).

  The controller work is Exynos5420Pkg's SpiCore.c, built unchanged
  (SpiDxe uses it in UEFI).  Clock (SCK 6 MHz) and pin functions are left
  as UEFI's SpiDxe set them, like exynosi2c does.  Polled, PASSIVE_LEVEL,
  mode 0, 8-bit, one device (DeviceSelection 0).  A request is one
  chip-select cycle; a sequence keeps the chip selected across its
  transfers (the WM5102 reads: address + pad out, value in).

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#define OSLOG_GLOBAL
#include <ExynosEnv.h>
#include <wdf.h>
#include <spb.h>
#include <spbcx.h>
#include <reshub.h>

#include "../../uefi/Exynos5420Pkg/Drivers/SpiDxe/SpiCore.h"

#include <pshpack1.h>
typedef struct _PNP_SPI_SERIAL_BUS_DESCRIPTOR {
  PNP_SERIAL_BUS_DESCRIPTOR  SerialBusDescriptor;
  ULONG                      ConnectionSpeed;
  UCHAR                      DataBitLength;
  UCHAR                      Phase;
  UCHAR                      Polarity;
  USHORT                     DeviceSelection;
} PNP_SPI_SERIAL_BUS_DESCRIPTOR, *PPNP_SPI_SERIAL_BUS_DESCRIPTOR;
#include <poppack.h>

#define SPI_SERIAL_BUS_TYPE  0x02

#define CS_PIN               2           // GPB1(2)
#define CS_BANK_PAGE         0x14010000  // GPIO controller with GPA0..GPB4
#define CS_BANK_OFFSET       0x80        // GPB1

typedef struct {
  EXYNOS_SPI  Spi;
  PUCHAR      Regs;
  SIZE_T      RegsSize;
  PUCHAR      CsBank;
  SIZE_T      CsBankSize;
  PUCHAR      CsPage;                   // own mapping of CS_BANK_PAGE (no ACPI window)
  ULONG       Requests;
} DEVICE_CONTEXT, *PDEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (DEVICE_CONTEXT, GetDeviceContext)

DRIVER_INITIALIZE                DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD        EvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE  EvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE  EvtReleaseHardware;
EVT_SPB_TARGET_CONNECT           EvtTargetConnect;
EVT_SPB_CONTROLLER_READ          EvtIoRead;
EVT_SPB_CONTROLLER_WRITE         EvtIoWrite;
EVT_SPB_CONTROLLER_SEQUENCE      EvtIoSequence;

/**
  One transfer's MDL chain through the controller (selected already).
**/
static INTN
RunMdl (PDEVICE_CONTEXT Ctx, BOOLEAN IsRead, PMDL Mdl, SIZE_T Length, SIZE_T *Done)
{
  PUCHAR  Va;
  SIZE_T  Chunk;

  *Done = 0;
  for (; (Mdl != NULL) && (*Done < Length); Mdl = Mdl->Next) {
    Va = MmGetSystemAddressForMdlSafe (Mdl, NormalPagePriority | MdlMappingNoExecute);
    if (Va == NULL) {
      return -1;
    }
    Chunk = min ((SIZE_T)MmGetMdlByteCount (Mdl), Length - *Done);
    if (SpiCoreExchange (&Ctx->Spi, IsRead ? NULL : Va, IsRead ? Va : NULL, Chunk) != 0) {
      return -1;
    }
    *Done += Chunk;
  }
  return 0;
}

static VOID
RunRequest (WDFDEVICE Controller, SPBREQUEST Request, ULONG Count)
{
  PDEVICE_CONTEXT          Ctx;
  SPB_TRANSFER_DESCRIPTOR  Desc;
  PMDL                     Mdl;
  ULONG                    Index;
  SIZE_T                   Done;
  SIZE_T                   Total;
  INTN                     Result;

  Ctx    = GetDeviceContext (Controller);
  Total  = 0;
  Result = 0;
  Ctx->Requests++;

  SpiCoreSelect (&Ctx->Spi, TRUE);
  for (Index = 0; (Index < Count) && (Result == 0); Index++) {
    SPB_TRANSFER_DESCRIPTOR_INIT (&Desc);
    SpbRequestGetTransferParameters (Request, Index, &Desc, &Mdl);
    if (Desc.DelayInUs != 0) {
      KeStallExecutionProcessor (Desc.DelayInUs);
    }
    Result = RunMdl (Ctx, Desc.Direction == SpbTransferDirectionFromDevice, Mdl, Desc.TransferLength, &Done);
    Total += Done;
  }
  SpiCoreSelect (&Ctx->Spi, FALSE);

  if (Result != 0) {
    OsLog ("exynosspi: transfer %lu of %lu failed after %Iu bytes", Index, Count, Total);
  }
  WdfRequestSetInformation ((WDFREQUEST)Request, Total);
  SpbRequestComplete (Request, (Result == 0) ? STATUS_SUCCESS : STATUS_IO_TIMEOUT);
}

VOID
EvtIoRead (WDFDEVICE Controller, SPBTARGET Target, SPBREQUEST Request, size_t Length)
{
  UNREFERENCED_PARAMETER (Target);
  UNREFERENCED_PARAMETER (Length);
  RunRequest (Controller, Request, 1);
}

VOID
EvtIoWrite (WDFDEVICE Controller, SPBTARGET Target, SPBREQUEST Request, size_t Length)
{
  UNREFERENCED_PARAMETER (Target);
  UNREFERENCED_PARAMETER (Length);
  RunRequest (Controller, Request, 1);
}

VOID
EvtIoSequence (WDFDEVICE Controller, SPBTARGET Target, SPBREQUEST Request, ULONG TransferCount)
{
  UNREFERENCED_PARAMETER (Target);
  RunRequest (Controller, Request, TransferCount);
}

/**
  Accept the SpiSerialBus connections this controller can serve: device 0,
  8-bit, mode 0 (the clock rate is UEFI's).
**/
NTSTATUS
EvtTargetConnect (WDFDEVICE Controller, SPBTARGET Target)
{
  SPB_CONNECTION_PARAMETERS                     Params;
  PRH_QUERY_CONNECTION_PROPERTIES_OUTPUT_BUFFER Conn;
  PPNP_SPI_SERIAL_BUS_DESCRIPTOR                Spi;

  UNREFERENCED_PARAMETER (Controller);

  SPB_CONNECTION_PARAMETERS_INIT (&Params);
  SpbTargetGetConnectionParameters (Target, &Params);
  Conn = (PRH_QUERY_CONNECTION_PROPERTIES_OUTPUT_BUFFER)Params.ConnectionParameters;
  if ((Conn == NULL) || (Conn->PropertiesLength < sizeof (PNP_SPI_SERIAL_BUS_DESCRIPTOR))) {
    return STATUS_INVALID_PARAMETER;
  }
  Spi = (PPNP_SPI_SERIAL_BUS_DESCRIPTOR)Conn->ConnectionProperties;
  if (Spi->SerialBusDescriptor.SerialBusType != SPI_SERIAL_BUS_TYPE) {
    return STATUS_INVALID_PARAMETER;
  }
  OsLog ("exynosspi: target device %u, %lu Hz requested, %u bits, phase %u polarity %u",
         Spi->DeviceSelection, Spi->ConnectionSpeed, Spi->DataBitLength, Spi->Phase, Spi->Polarity);
  if ((Spi->DeviceSelection != 0) || (Spi->DataBitLength != 8) || (Spi->Phase != 0) || (Spi->Polarity != 0)) {
    return STATUS_NOT_SUPPORTED;
  }
  return STATUS_SUCCESS;
}

NTSTATUS
EvtPrepareHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
{
  PDEVICE_CONTEXT                  Ctx;
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  ULONG                            Index;
  ULONG                            Count;

  UNREFERENCED_PARAMETER (ResourcesRaw);

  Ctx   = GetDeviceContext (Device);
  Count = WdfCmResourceListGetCount (ResourcesTranslated);
  for (Index = 0; Index < Count; Index++) {
    Desc = WdfCmResourceListGetDescriptor (ResourcesTranslated, Index);
    if (Desc->Type != CmResourceTypeMemory) {
      continue;
    }
    if (Ctx->Regs == NULL) {
      Ctx->RegsSize = Desc->u.Memory.Length;
      Ctx->Regs     = MmMapIoSpace (Desc->u.Memory.Start, Ctx->RegsSize, MmNonCached);
    } else if (Ctx->CsBank == NULL) {
      Ctx->CsBankSize = Desc->u.Memory.Length;
      Ctx->CsBank     = MmMapIoSpace (Desc->u.Memory.Start, Ctx->CsBankSize, MmNonCached);
    }
  }
  if ((Ctx->CsBank == NULL) && (Ctx->CsPage == NULL)) {
    PHYSICAL_ADDRESS  Page;

    Page.QuadPart = CS_BANK_PAGE;
    Ctx->CsPage   = MmMapIoSpace (Page, PAGE_SIZE, MmNonCached);
  }
  if ((Ctx->Regs == NULL) || ((Ctx->CsBank == NULL) && (Ctx->CsPage == NULL))) {
    OsLog ("exynosspi: controller window or chip select bank missing");
    return STATUS_DEVICE_CONFIGURATION_ERROR;
  }

  Ctx->Spi.Base   = (UINTN)Ctx->Regs;
  Ctx->Spi.CsBank = (Ctx->CsBank != NULL) ? (UINTN)Ctx->CsBank : (UINTN)(Ctx->CsPage + CS_BANK_OFFSET);
  Ctx->Spi.CsPin  = CS_PIN;
  SpiCoreInit (&Ctx->Spi);
  OsLog ("exynosspi: controller at 0x%08lx ready", WdfCmResourceListGetDescriptor (ResourcesRaw, 0)->u.Memory.Start.LowPart);
  return STATUS_SUCCESS;
}

NTSTATUS
EvtReleaseHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesTranslated)
{
  PDEVICE_CONTEXT  Ctx;

  UNREFERENCED_PARAMETER (ResourcesTranslated);

  Ctx = GetDeviceContext (Device);
  OsLog ("exynosspi: released after %lu requests, %lu timeouts", Ctx->Requests, Ctx->Spi.Timeouts);
  if (Ctx->Regs != NULL) {
    MmUnmapIoSpace (Ctx->Regs, Ctx->RegsSize);
    Ctx->Regs = NULL;
  }
  if (Ctx->CsBank != NULL) {
    MmUnmapIoSpace (Ctx->CsBank, Ctx->CsBankSize);
    Ctx->CsBank = NULL;
  }
  if (Ctx->CsPage != NULL) {
    MmUnmapIoSpace (Ctx->CsPage, PAGE_SIZE);
    Ctx->CsPage = NULL;
  }
  return STATUS_SUCCESS;
}

NTSTATUS
EvtDeviceAdd (WDFDRIVER Driver, PWDFDEVICE_INIT DeviceInit)
{
  WDF_PNPPOWER_EVENT_CALLBACKS  PnpPower;
  WDF_OBJECT_ATTRIBUTES         Attributes;
  SPB_CONTROLLER_CONFIG         SpbConfig;
  WDFDEVICE                     Device;
  NTSTATUS                      Status;

  UNREFERENCED_PARAMETER (Driver);

  Status = SpbDeviceInitConfig (DeviceInit);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  WDF_PNPPOWER_EVENT_CALLBACKS_INIT (&PnpPower);
  PnpPower.EvtDevicePrepareHardware = EvtPrepareHardware;
  PnpPower.EvtDeviceReleaseHardware = EvtReleaseHardware;
  WdfDeviceInitSetPnpPowerEventCallbacks (DeviceInit, &PnpPower);

  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, DEVICE_CONTEXT);
  Attributes.ExecutionLevel = WdfExecutionLevelPassive;
  Status = WdfDeviceCreate (&DeviceInit, &Attributes, &Device);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  SPB_CONTROLLER_CONFIG_INIT (&SpbConfig);
  SpbConfig.ControllerDispatchType = WdfIoQueueDispatchSequential;
  SpbConfig.EvtSpbTargetConnect    = EvtTargetConnect;
  SpbConfig.EvtSpbIoRead           = EvtIoRead;
  SpbConfig.EvtSpbIoWrite          = EvtIoWrite;
  SpbConfig.EvtSpbIoSequence       = EvtIoSequence;
  return SpbDeviceInitialize (Device, &SpbConfig);
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG  Config;

  OsLogInit ();
  OsLog ("exynosspi: DriverEntry (built " __DATE__ " " __TIME__ ")");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  return WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, WDF_NO_HANDLE);
}
