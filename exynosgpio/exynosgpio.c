/** @file
  Exynos5420 GPIO interrupt lines as a GpioClx client for v1awifi under
  Windows RT 8.1 (KMDF 1.13).

  ACPI: _HID EXYN5003, one Memory32Fixed over the RT and T GPIO controllers
  (0x13400000 + 0x20000) and one interrupt per bank, in bank order:

    bank 0  pins 0-7   GPD1 (T controller, base + 0x10000), GPIO group
                       interrupt (group 5 of the T bank, SPI 78 = GSI 110,
                       shared by all T groups): touchscreen CHG = GPD1(0)
    bank 1  pins 8-15  GPX3 (RT controller), EINT24-31 (SPI 32 = GSI 64,
                       shared by EINT16-31): S Pen IRQ = GPX3(5) = pin 13

  Register layouts: drivers/gpio/gpio-samsung.c, plat-s5p/irq-gpioint.c
  (CON 0x700 / MASK 0x900 / PEND 0xA00 + 4 * group) and mach-exynos
  regs-gpio.h (EINT CON 0xE00 / MASK 0xF00 / PEND 0xF40 + 4 * (eint / 8)).
  Both use the same 3-bit trigger encoding per pin and write-1-to-clear
  pending bits.

  With LOAD_TEST it bugchecks 0x4750494F ("GPIO") in PrepareController with
  P1 = memory base, P2 = first interrupt (raw GSI), P3 = resource count.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <ntddk.h>
#include <wdf.h>
#include <gpioclx.h>
#include "oslog.h"

#ifndef LOAD_TEST
#define LOAD_TEST 0
#endif
#define GPIO_BUGCHECK  0x4750494F

#define PINS_PER_BANK  8
#define BANKS          2

#define FUNC_INPUT   0x0
#define FUNC_OUTPUT  0x1
#define FUNC_INT     0xF          // EINT / GPIO interrupt function

#define INT_LEVEL_LOW     0x0
#define INT_LEVEL_HIGH    0x1
#define INT_EDGE_FALLING  0x2
#define INT_EDGE_RISING   0x3
#define INT_EDGE_BOTH     0x4

#define PUD_NONE  0x0
#define PUD_DOWN  0x1
#define PUD_UP    0x3

typedef struct {
  const char  *Name;
  ULONG       Con;            // offsets from the mapped base (RT controller)
  ULONG       Dat;
  ULONG       Pud;
  ULONG       IntCon;
  ULONG       IntMask;
  ULONG       IntPend;
} BANK;

static const BANK mBanks[BANKS] = {
  { "GPD1", 0x100A0, 0x100A4, 0x100A8, 0x10714, 0x10914, 0x10A14 },
  { "GPX3", 0x00C60, 0x00C64, 0x00C68, 0x00E0C, 0x00F0C, 0x00F4C },
};

//
// Everything else that shares the two interrupt lines, masked at start:
// the 13 T-controller GPIO groups, and EINT16-23 (GPX2).
//
#define T_GROUP_MASK_FIRST  0x10900
#define T_GROUP_COUNT       13
#define EINT16_23_MASK      0x00F08

typedef struct {
  PUCHAR  Base;
  SIZE_T  Size;
} GPIO_CONTEXT, *PGPIO_CONTEXT;

DRIVER_INITIALIZE          DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD  EvtDeviceAdd;
EVT_WDF_DRIVER_UNLOAD      EvtDriverUnload;

static ULONG
RegRead (PGPIO_CONTEXT Ctx, ULONG Reg)
{
  return READ_REGISTER_ULONG ((PULONG)(Ctx->Base + Reg));
}

static VOID
RegWrite (PGPIO_CONTEXT Ctx, ULONG Reg, ULONG Value)
{
  WRITE_REGISTER_ULONG ((PULONG)(Ctx->Base + Reg), Value);
}

static VOID
RegField (PGPIO_CONTEXT Ctx, ULONG Reg, ULONG Shift, ULONG Width, ULONG Value)
{
  ULONG  Mask;

  Mask = ((1u << Width) - 1) << Shift;
  RegWrite (Ctx, Reg, (RegRead (Ctx, Reg) & ~Mask) | ((Value << Shift) & Mask));
}

static ULONG
PullValue (UCHAR PullConfiguration)
{
  switch (PullConfiguration) {
    case GPIO_PIN_PULL_CONFIGURATION_PULLUP:   return PUD_UP;
    case GPIO_PIN_PULL_CONFIGURATION_PULLDOWN: return PUD_DOWN;
    default:                                   return PUD_NONE;
  }
}

static ULONG
IntType (KINTERRUPT_MODE Mode, KINTERRUPT_POLARITY Polarity)
{
  if (Mode == LevelSensitive) {
    return (Polarity == InterruptActiveLow) ? INT_LEVEL_LOW : INT_LEVEL_HIGH;
  }
  switch (Polarity) {
    case InterruptActiveLow:  return INT_EDGE_FALLING;
    case InterruptActiveBoth: return INT_EDGE_BOTH;
    default:                  return INT_EDGE_RISING;
  }
}

static const BANK *
BankOf (BANK_ID BankId)
{
  return &mBanks[(BankId < BANKS) ? BankId : 0];
}

static NTSTATUS GPIO_EXPORT
GpioPrepareController (WDFDEVICE Device, PVOID Context, WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
{
  PGPIO_CONTEXT                    Ctx;
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  ULONG                            Index;
  ULONG                            Count;
  ULONG_PTR                        Gsi;

  UNREFERENCED_PARAMETER (Device);

  Ctx   = Context;
  Gsi   = 0;
  Count = WdfCmResourceListGetCount (ResourcesTranslated);
  for (Index = 0; Index < Count; Index++) {
    Desc = WdfCmResourceListGetDescriptor (ResourcesTranslated, Index);
    if ((Desc->Type == CmResourceTypeMemory) && (Ctx->Base == NULL)) {
      Ctx->Size = Desc->u.Memory.Length;
      Ctx->Base = MmMapIoSpace (Desc->u.Memory.Start, Ctx->Size, MmNonCached);
      if (Ctx->Base == NULL) {
        return STATUS_INSUFFICIENT_RESOURCES;
      }
    } else if ((Desc->Type == CmResourceTypeInterrupt) && (Gsi == 0)) {
      Gsi = WdfCmResourceListGetDescriptor (ResourcesRaw, Index)->u.Interrupt.Vector;
    }
  }

  if ((Ctx->Base == NULL) || (Ctx->Size < 0x20000)) {
    OsLog ("exynosgpio: memory resource missing or too small");
    return STATUS_DEVICE_CONFIGURATION_ERROR;
  }
  OsLog ("exynosgpio: controller mapped, first GSI %Iu, %lu resources", Gsi, Count);

#if LOAD_TEST
  KeBugCheckEx (GPIO_BUGCHECK,
                (ULONG_PTR)WdfCmResourceListGetDescriptor (ResourcesRaw, 0)->u.Memory.Start.QuadPart,
                Gsi, Count, 0);
#else
  UNREFERENCED_PARAMETER (Gsi);
#endif
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioReleaseController (WDFDEVICE Device, PVOID Context)
{
  PGPIO_CONTEXT  Ctx;

  UNREFERENCED_PARAMETER (Device);

  Ctx = Context;
  if (Ctx->Base != NULL) {
    MmUnmapIoSpace (Ctx->Base, Ctx->Size);
    Ctx->Base = NULL;
  }
  return STATUS_SUCCESS;
}

/**
  Nothing may fire before GpioClx enables it: mask every source on the two
  shared lines and drop whatever is pending.
**/
static NTSTATUS GPIO_EXPORT
GpioStartController (PVOID Context, BOOLEAN RestoreContext, WDF_POWER_DEVICE_STATE PreviousPowerState)
{
  PGPIO_CONTEXT  Ctx;
  ULONG          Group;
  ULONG          Bank;

  UNREFERENCED_PARAMETER (PreviousPowerState);

  Ctx = Context;
  if (!RestoreContext) {
    for (Group = 0; Group < T_GROUP_COUNT; Group++) {
      RegWrite (Ctx, T_GROUP_MASK_FIRST + Group * 4, 0xFF);
    }
    RegWrite (Ctx, EINT16_23_MASK, 0xFF);
    for (Bank = 0; Bank < BANKS; Bank++) {
      RegWrite (Ctx, mBanks[Bank].IntMask, 0xFF);
      RegWrite (Ctx, mBanks[Bank].IntPend, 0xFF);
    }
  }
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioStopController (PVOID Context, BOOLEAN SaveContext, WDF_POWER_DEVICE_STATE TargetState)
{
  UNREFERENCED_PARAMETER (Context);
  UNREFERENCED_PARAMETER (SaveContext);
  UNREFERENCED_PARAMETER (TargetState);
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioQueryControllerBasicInformation (PVOID Context, PCLIENT_CONTROLLER_BASIC_INFORMATION Information)
{
  UNREFERENCED_PARAMETER (Context);

  Information->Version             = GPIO_CONTROLLER_BASIC_INFORMATION_VERSION;
  Information->Size                = sizeof (*Information);
  Information->TotalPins           = BANKS * PINS_PER_BANK;
  Information->NumberOfPinsPerBank = PINS_PER_BANK;
  Information->Flags.MemoryMappedController   = 1;
  Information->Flags.FormatIoRequestsAsMasks  = 1;
  Information->Flags.EmulateDebouncing        = 1;
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioEnableInterrupt (PVOID Context, PGPIO_ENABLE_INTERRUPT_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;
  const BANK     *Bank;
  ULONG          Pin;

  Ctx  = Context;
  Bank = BankOf (Params->BankId);
  Pin  = Params->PinNumber;
  OsLog ("exynosgpio: enable %s(%lu) %s active %s", Bank->Name, Pin,
         (Params->InterruptMode == LevelSensitive) ? "level" : "edge",
         (Params->Polarity == InterruptActiveLow) ? "low" : "high");

  RegWrite (Ctx, Bank->IntMask, RegRead (Ctx, Bank->IntMask) | (1u << Pin));
  RegField (Ctx, Bank->Pud, Pin * 2, 2, PullValue (Params->PullConfiguration));
  RegField (Ctx, Bank->IntCon, Pin * 4, 3, IntType (Params->InterruptMode, Params->Polarity));
  RegField (Ctx, Bank->Con, Pin * 4, 4, FUNC_INT);
  RegWrite (Ctx, Bank->IntPend, 1u << Pin);
  RegWrite (Ctx, Bank->IntMask, RegRead (Ctx, Bank->IntMask) & ~(1u << Pin));
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioDisableInterrupt (PVOID Context, PGPIO_DISABLE_INTERRUPT_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;
  const BANK     *Bank;

  Ctx  = Context;
  Bank = BankOf (Params->BankId);
  RegWrite (Ctx, Bank->IntMask, RegRead (Ctx, Bank->IntMask) | (1u << Params->PinNumber));
  RegWrite (Ctx, Bank->IntPend, 1u << Params->PinNumber);
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioMaskInterrupts (PVOID Context, PGPIO_MASK_INTERRUPT_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;
  const BANK     *Bank;

  Ctx  = Context;
  Bank = BankOf (Params->BankId);
  RegWrite (Ctx, Bank->IntMask, RegRead (Ctx, Bank->IntMask) | (ULONG)(Params->PinMask & 0xFF));
  Params->FailedMask = 0;
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioUnmaskInterrupt (PVOID Context, PGPIO_ENABLE_INTERRUPT_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;
  const BANK     *Bank;

  Ctx  = Context;
  Bank = BankOf (Params->BankId);
  RegWrite (Ctx, Bank->IntMask, RegRead (Ctx, Bank->IntMask) & ~(1u << Params->PinNumber));
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioQueryActiveInterrupts (PVOID Context, PGPIO_QUERY_ACTIVE_INTERRUPTS_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;

  Ctx = Context;
  Params->ActiveMask = (RegRead (Ctx, BankOf (Params->BankId)->IntPend) & 0xFF) & Params->EnabledMask;
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioClearActiveInterrupts (PVOID Context, PGPIO_CLEAR_ACTIVE_INTERRUPTS_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;

  Ctx = Context;
  RegWrite (Ctx, BankOf (Params->BankId)->IntPend, (ULONG)(Params->ClearActiveMask & 0xFF));
  Params->FailedClearMask = 0;
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioQueryEnabledInterrupts (PVOID Context, PGPIO_QUERY_ENABLED_INTERRUPTS_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;
  const BANK     *Bank;
  ULONG          Con;
  ULONG          Mask;
  ULONG          Pin;

  Ctx  = Context;
  Bank = BankOf (Params->BankId);
  Con  = RegRead (Ctx, Bank->Con);
  Mask = RegRead (Ctx, Bank->IntMask);
  Params->EnabledMask = 0;
  for (Pin = 0; Pin < PINS_PER_BANK; Pin++) {
    if ((((Con >> (Pin * 4)) & 0xF) == FUNC_INT) && (((Mask >> Pin) & 1) == 0)) {
      Params->EnabledMask |= (ULONG64)1 << Pin;
    }
  }
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioReconfigureInterrupt (PVOID Context, PGPIO_RECONFIGURE_INTERRUPTS_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;
  const BANK     *Bank;

  Ctx  = Context;
  Bank = BankOf (Params->BankId);
  RegField (Ctx, Bank->IntCon, Params->PinNumber * 4, 3, IntType (Params->InterruptMode, Params->Polarity));
  RegWrite (Ctx, Bank->IntPend, 1u << Params->PinNumber);
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioConnectIoPins (PVOID Context, PGPIO_CONNECT_IO_PINS_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;
  const BANK     *Bank;
  ULONG          Index;
  ULONG          Pin;
  ULONG          Func;

  Ctx  = Context;
  Bank = BankOf (Params->BankId);
  Func = (Params->ConnectMode == ConnectModeOutput) ? FUNC_OUTPUT : FUNC_INPUT;
  for (Index = 0; Index < Params->PinCount; Index++) {
    Pin = Params->PinNumberTable[Index];
    RegField (Ctx, Bank->Pud, Pin * 2, 2, PullValue (Params->PullConfiguration));
    RegField (Ctx, Bank->Con, Pin * 4, 4, Func);
  }
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioDisconnectIoPins (PVOID Context, PGPIO_DISCONNECT_IO_PINS_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;
  const BANK     *Bank;
  ULONG          Index;

  Ctx  = Context;
  Bank = BankOf (Params->BankId);
  if (Params->DisconnectFlags.PreserveConfiguration == 0) {
    for (Index = 0; Index < Params->PinCount; Index++) {
      RegField (Ctx, Bank->Con, Params->PinNumberTable[Index] * 4, 4, FUNC_INPUT);
    }
  }
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioReadPins (PVOID Context, PGPIO_READ_PINS_MASK_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;

  Ctx = Context;
  *Params->PinValues = RegRead (Ctx, BankOf (Params->BankId)->Dat) & 0xFF;
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioWritePins (PVOID Context, PGPIO_WRITE_PINS_MASK_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;
  const BANK     *Bank;
  ULONG          Dat;

  Ctx  = Context;
  Bank = BankOf (Params->BankId);
  Dat  = RegRead (Ctx, Bank->Dat);
  Dat |= (ULONG)(Params->SetMask & 0xFF);
  Dat &= ~(ULONG)(Params->ClearMask & 0xFF);
  RegWrite (Ctx, Bank->Dat, Dat);
  return STATUS_SUCCESS;
}

NTSTATUS
EvtDeviceAdd (WDFDRIVER Driver, PWDFDEVICE_INIT DeviceInit)
{
  WDF_OBJECT_ATTRIBUTES  Attributes;
  WDFDEVICE              Device;
  NTSTATUS               Status;

  Status = GPIO_CLX_ProcessAddDevicePreDeviceCreate (Driver, DeviceInit, &Attributes);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  Status = WdfDeviceCreate (&DeviceInit, &Attributes, &Device);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  return GPIO_CLX_ProcessAddDevicePostDeviceCreate (Driver, Device);
}

VOID
EvtDriverUnload (WDFDRIVER Driver)
{
  GPIO_CLX_UnregisterClient (Driver);
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG                Config;
  WDFDRIVER                        Driver;
  GPIO_CLIENT_REGISTRATION_PACKET  Packet;
  NTSTATUS                         Status;

  OsLogInit ();
  OsLog ("exynosgpio: DriverEntry");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  Config.EvtDriverUnload = EvtDriverUnload;
  Status = WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, &Driver);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  //
  // Version 2 (Windows 8.1): this header's packet is version 4, so register
  // only the part an 8.1 GpioClx knows.
  //
  RtlZeroMemory (&Packet, sizeof (Packet));
  Packet.Version               = 2;
  Packet.Size                  = FIELD_OFFSET (GPIO_CLIENT_REGISTRATION_PACKET, CLIENT_ConnectFunctionConfigPins);
  Packet.ControllerContextSize = sizeof (GPIO_CONTEXT);
  Packet.CLIENT_PrepareController              = GpioPrepareController;
  Packet.CLIENT_ReleaseController              = GpioReleaseController;
  Packet.CLIENT_StartController                = GpioStartController;
  Packet.CLIENT_StopController                 = GpioStopController;
  Packet.CLIENT_QueryControllerBasicInformation = GpioQueryControllerBasicInformation;
  Packet.CLIENT_EnableInterrupt                = GpioEnableInterrupt;
  Packet.CLIENT_DisableInterrupt               = GpioDisableInterrupt;
  Packet.CLIENT_UnmaskInterrupt                = GpioUnmaskInterrupt;
  Packet.CLIENT_MaskInterrupts                 = GpioMaskInterrupts;
  Packet.CLIENT_QueryActiveInterrupts          = GpioQueryActiveInterrupts;
  Packet.CLIENT_ClearActiveInterrupts          = GpioClearActiveInterrupts;
  Packet.CLIENT_ConnectIoPins                  = GpioConnectIoPins;
  Packet.CLIENT_DisconnectIoPins               = GpioDisconnectIoPins;
  Packet.CLIENT_ReadGpioPinsUsingMask          = GpioReadPins;
  Packet.CLIENT_WriteGpioPinsUsingMask         = GpioWritePins;
  Packet.CLIENT_ReconfigureInterrupt           = GpioReconfigureInterrupt;
  Packet.CLIENT_QueryEnabledInterrupts         = GpioQueryEnabledInterrupts;

  Status = GPIO_CLX_RegisterClient (Driver, &Packet, RegistryPath);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosgpio: GpioClx registration v%u size %u failed: 0x%08lx", Packet.Version, Packet.Size, Status);
  }
  return Status;
}
