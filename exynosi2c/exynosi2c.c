/** @file
  Exynos5420 I2C controllers as SPB controllers for SpbCx, for v1awifi under
  Windows RT 8.1 (KMDF 1.13, SPB 1.0).  Two controller types, told apart by
  the ACPI hardware ID:

    EXYN5002  s3c2410-style I2C (I2C0..3; the mXT1664S touchscreen is on
              I2C0): polling the byte-done (IRQPEND) bit, the sequence of
              UEFI's I2CDxe (u-boot / Linux s3c24xx_i2c).
    EXYN5004  high-speed I2C (exynos5-hs-i2c; the Wacom digitizer is on
              hs-i2c4): auto mode with FIFO polling, UEFI's HsI2c.c
              (Linux i2c-exynos5.c HSI2C_POLLING).

  One Memory32Fixed each.  Pins and clock gates are left set up by UEFI
  (both drivers there touch these buses).  Transfers run at PASSIVE level
  in the SpbCx I/O callbacks; the controller interrupts are not used.

  With LOAD_TEST it bugchecks 0x49324321 ("I2C!") in PrepareHardware with
  P1 = memory base, P2 = interrupt (raw GSI), P3 = resource count.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <ntddk.h>
#include <wdf.h>
#include <spb.h>
#include <spbcx.h>
#include <reshub.h>
#include "oslog.h"

#ifndef LOAD_TEST
#define LOAD_TEST 0
#endif
#define LOAD_TEST_BUGCHECK  0x49324321


//
// The I2C flavour of the resource hub's serial bus descriptor (ACPI
// I2cSerialBus), as the Windows I2C controller samples define it.
//
#include <pshpack1.h>
typedef struct _PNP_I2C_SERIAL_BUS_DESCRIPTOR {
  PNP_SERIAL_BUS_DESCRIPTOR  SerialBusDescriptor;
  ULONG                      ConnectionSpeed;
  USHORT                     SlaveAddress;
} PNP_I2C_SERIAL_BUS_DESCRIPTOR, *PPNP_I2C_SERIAL_BUS_DESCRIPTOR;
#include <poppack.h>

#define I2C_SERIAL_BUS_TYPE                         0x01
#define I2C_SERIAL_BUS_SPECIFIC_FLAG_10BIT_ADDRESS  0x0001

//
// Registers and bits (plat/regs-iic.h).
//
#define IICCON   0x00
#define IICSTAT  0x04
#define IICADD   0x08
#define IICDS    0x0C

#define IICCON_ACKEN    (1u << 7)
#define IICCON_TXDIV    (1u << 6)   // prescaler 16 (0) / 512 (1)
#define IICCON_IRQEN    (1u << 5)
#define IICCON_IRQPEND  (1u << 4)
#define IICCON_SCALE    0x0Fu

#define IICSTAT_MODE_RX  (2u << 6)
#define IICSTAT_MODE_TX  (3u << 6)
#define IICSTAT_START    (1u << 5)  // write: start/stop, read: bus busy
#define IICSTAT_TXRXEN   (1u << 4)
#define IICSTAT_NACK     (1u << 0)

#define I2C_INPUT_CLOCK_HZ  66000000    // ACLK_66_PERIC
#define I2C_DEFAULT_HZ      100000
#define I2C_BYTE_TIMEOUT_US 10000

//
// High-speed I2C (i2c-exynos5.h).
//
#define HSI2C_CTL           0x00
#define HSI2C_FIFO_CTL      0x04
#define HSI2C_TRAILING_CTL  0x08
#define HSI2C_INT_ENABLE    0x20
#define HSI2C_FIFO_STATUS   0x30
#define HSI2C_TX_DATA       0x34
#define HSI2C_RX_DATA       0x38
#define HSI2C_CONF          0x40
#define HSI2C_AUTO_CONFIG   0x44
#define HSI2C_TIMEOUT       0x48
#define HSI2C_TRANS_STATUS  0x50
#define HSI2C_TIMING_FS1    0x60
#define HSI2C_TIMING_FS2    0x64
#define HSI2C_TIMING_FS3    0x68
#define HSI2C_TIMING_SLA    0x6C
#define HSI2C_ADDR          0x70

#define HSCTL_FUNC_MODE_I2C   (1u << 0)
#define HSCTL_MASTER          (1u << 3)
#define HSCTL_RXCHON          (1u << 6)
#define HSCTL_TXCHON          (1u << 7)
#define HSCTL_SW_RST          (1u << 31)
#define HSFIFO_CTL_DEFAULT    ((1u << 0) | (1u << 1) | (0x20u << 16) | (0x20u << 4))
#define HSFIFO_TX_LEVEL(s)    ((s) & 0x7F)
#define HSFIFO_RX_EMPTY       (1u << 24)
#define HSFIFO_ALL_EMPTY      0x01000100u
#define HSAUTO_READ_WRITE     (1u << 16)
#define HSAUTO_STOP_AFTER     (1u << 17)
#define HSAUTO_MASTER_RUN     (1u << 31)
#define HSTIMEOUT_EN          (1u << 31)
#define HSTRANS_IDLE_NO_STOP  0x20000u
#define HSI2C_PCLK_HZ         66600000u
#define HSI2C_MSG_TIMEOUT_US  50000

typedef struct {
  PUCHAR   Base;
  SIZE_T   Size;
  BOOLEAN  HighSpeed;       // EXYN5004: exynos5-hs-i2c
} DEVICE_CONTEXT, *PDEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (DEVICE_CONTEXT, GetDeviceContext)

typedef struct {
  USHORT  Address;
  ULONG   Speed;
} TARGET_CONTEXT, *PTARGET_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (TARGET_CONTEXT, GetTargetContext)

typedef enum {
  XferOk,
  XferNack,
  XferTimeout
} XFER_RESULT;

DRIVER_INITIALIZE                DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD        EvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE  EvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE  EvtReleaseHardware;
EVT_SPB_TARGET_CONNECT           EvtTargetConnect;
EVT_SPB_CONTROLLER_READ          EvtIoRead;
EVT_SPB_CONTROLLER_WRITE         EvtIoWrite;
EVT_SPB_CONTROLLER_SEQUENCE      EvtIoSequence;

static ULONG
RegRead (PDEVICE_CONTEXT Ctx, ULONG Reg)
{
  return READ_REGISTER_ULONG ((PULONG)(Ctx->Base + Reg));
}

static VOID
RegWrite (PDEVICE_CONTEXT Ctx, ULONG Reg, ULONG Value)
{
  WRITE_REGISTER_ULONG ((PULONG)(Ctx->Base + Reg), Value);
}

/**
  Prescaler/divider for the target's bus speed, then idle in master
  transmit mode (u-boot i2c_ch_init()).
**/
static VOID
I2cInit (PDEVICE_CONTEXT Ctx, ULONG Speed)
{
  ULONG  Pres;
  ULONG  Div;
  ULONG  Con;

  Pres = 16;
  if ((I2C_INPUT_CLOCK_HZ / Pres / (IICCON_SCALE + 1)) > Speed) {
    Pres = 512;
  }
  for (Div = 0; (Div < IICCON_SCALE) && ((I2C_INPUT_CLOCK_HZ / Pres / (Div + 1)) > Speed); Div++) {
  }

  Con = Div | IICCON_ACKEN | IICCON_IRQEN | ((Pres == 512) ? IICCON_TXDIV : 0);
  RegWrite (Ctx, IICCON, Con);
  RegWrite (Ctx, IICSTAT, 0);
  RegWrite (Ctx, IICADD, 0);
  RegWrite (Ctx, IICSTAT, IICSTAT_MODE_TX | IICSTAT_TXRXEN);
}

static XFER_RESULT
I2cWaitByte (PDEVICE_CONTEXT Ctx)
{
  ULONG  Us;

  for (Us = 0; Us < I2C_BYTE_TIMEOUT_US; Us++) {
    if ((RegRead (Ctx, IICCON) & IICCON_IRQPEND) != 0) {
      return ((RegRead (Ctx, IICSTAT) & IICSTAT_NACK) != 0) ? XferNack : XferOk;
    }
    KeStallExecutionProcessor (1);
  }
  return XferTimeout;
}

/**
  Clearing the pend bit acknowledges the finished byte and clocks the next.
**/
static VOID
I2cNextByte (PDEVICE_CONTEXT Ctx)
{
  RegWrite (Ctx, IICCON, RegRead (Ctx, IICCON) & ~IICCON_IRQPEND);
}

/**
  Bytes of one SPB transfer live in an MDL chain.
**/
typedef struct {
  PMDL    Mdl;
  PUCHAR  Va;
  SIZE_T  Offset;
} MDL_CURSOR;

static BOOLEAN
CursorNext (MDL_CURSOR *Cursor, PUCHAR *Byte)
{
  while (Cursor->Mdl != NULL) {
    if (Cursor->Va == NULL) {
      Cursor->Va = MmGetSystemAddressForMdlSafe (Cursor->Mdl, NormalPagePriority | MdlMappingNoExecute);
      if (Cursor->Va == NULL) {
        return FALSE;
      }
    }
    if (Cursor->Offset < MmGetMdlByteCount (Cursor->Mdl)) {
      *Byte = Cursor->Va + Cursor->Offset++;
      return TRUE;
    }
    Cursor->Mdl    = Cursor->Mdl->Next;
    Cursor->Va     = NULL;
    Cursor->Offset = 0;
  }
  return FALSE;
}

/**
  One transfer with a (repeated) start; the caller sends STOP.
**/
static XFER_RESULT
I2cTransfer (
  PDEVICE_CONTEXT  Ctx,
  USHORT           Address,
  BOOLEAN          IsRead,
  PMDL             Mdl,
  SIZE_T           Length,
  BOOLEAN          Repeated,
  SIZE_T           *Done
  )
{
  ULONG        Mode;
  XFER_RESULT  Result;
  MDL_CURSOR   Cursor;
  PUCHAR       Byte;
  SIZE_T       Index;

  *Done  = 0;
  Mode   = IsRead ? IICSTAT_MODE_RX : IICSTAT_MODE_TX;
  Cursor.Mdl    = Mdl;
  Cursor.Va     = NULL;
  Cursor.Offset = 0;

  // A previous read NACKed its last byte: ACK again.
  RegWrite (Ctx, IICCON, RegRead (Ctx, IICCON) | IICCON_ACKEN);

  // On a repeated start the previous byte's pend bit still holds SCL low;
  // clear it only once START is armed, or the address goes out as data.
  RegWrite (Ctx, IICSTAT, IICSTAT_TXRXEN | Mode);
  RegWrite (Ctx, IICDS, (ULONG)(Address << 1) | (IsRead ? 1 : 0));
  KeStallExecutionProcessor (1);
  RegWrite (Ctx, IICSTAT, IICSTAT_TXRXEN | IICSTAT_START | Mode);
  if (Repeated) {
    I2cNextByte (Ctx);
  }

  Result = I2cWaitByte (Ctx);
  if (Result != XferOk) {
    return Result;
  }

  for (Index = 0; Index < Length; Index++) {
    if (!CursorNext (&Cursor, &Byte)) {
      return XferTimeout;
    }
    if (IsRead) {
      if (Index == Length - 1) {
        // NACK the last byte so the slave releases the bus.
        RegWrite (Ctx, IICCON, RegRead (Ctx, IICCON) & ~IICCON_ACKEN);
      }
      I2cNextByte (Ctx);
      Result = I2cWaitByte (Ctx);
      *Byte  = (UCHAR)RegRead (Ctx, IICDS);
      if ((Result == XferNack) && (Index == Length - 1)) {
        Result = XferOk;
      }
    } else {
      RegWrite (Ctx, IICDS, *Byte);
      I2cNextByte (Ctx);
      Result = I2cWaitByte (Ctx);
    }
    if (Result != XferOk) {
      return Result;
    }
    (*Done)++;
  }

  return XferOk;
}

static VOID
I2cStop (PDEVICE_CONTEXT Ctx, BOOLEAN LastWasRead)
{
  RegWrite (Ctx, IICSTAT, IICSTAT_TXRXEN | (LastWasRead ? IICSTAT_MODE_RX : IICSTAT_MODE_TX));
  I2cNextByte (Ctx);
}

//
// ---------------------------------------------------------------------------
// High-speed I2C (UEFI HsI2c.c).
// ---------------------------------------------------------------------------
//

/**
  Soft reset, I2C master mode, fast-mode timing for Speed from PCLK
  (exynos5_i2c_init() / exynos5_i2c_set_timing()).
**/
static VOID
HsInit (PDEVICE_CONTEXT Ctx, ULONG Speed)
{
  ULONG  Filter;
  ULONG  Temp0;
  ULONG  Temp1;
  ULONG  Temp2;
  ULONG  Div;
  ULONG  SclL;

  RegWrite (Ctx, HSI2C_CTL, RegRead (Ctx, HSI2C_CTL) | HSCTL_SW_RST);
  RegWrite (Ctx, HSI2C_CTL, RegRead (Ctx, HSI2C_CTL) & ~HSCTL_SW_RST);
  RegWrite (Ctx, HSI2C_CONF, RegRead (Ctx, HSI2C_CONF) | (0x7u << 16));
  RegWrite (Ctx, HSI2C_CTL, HSCTL_FUNC_MODE_I2C | HSCTL_MASTER);
  RegWrite (Ctx, HSI2C_TRAILING_CTL, 0xF);
  RegWrite (Ctx, HSI2C_INT_ENABLE, 0);

  Speed  = min (max (Speed, 100000u), 400000u);
  Filter = (RegRead (Ctx, HSI2C_CONF) >> 16) & 0x7;
  Temp0  = (HSI2C_PCLK_HZ / Speed) - 8 - 2 * Filter;
  Temp2  = 0;
  for (Div = 0; Div < 256; Div++) {
    Temp1 = Temp0 / (Div + 1);
    if (Temp1 < 128) {
      Temp2 = Temp1 - 2;
      break;
    }
  }
  SclL = Temp2 / 2;
  RegWrite (Ctx, HSI2C_TIMING_FS1, (SclL << 24) | (SclL << 16) | (SclL << 8));
  RegWrite (Ctx, HSI2C_TIMING_FS2, ((SclL / 2) << 24) | (SclL << 8) | (Temp2 / 2));
  RegWrite (Ctx, HSI2C_TIMING_FS3, (Div << 16) | Temp2);
  RegWrite (Ctx, HSI2C_TIMING_SLA, SclL / 2);
}

/**
  One transfer in auto mode.  Stop: end with STOP (last transfer), else
  leave the bus for a repeated start.  A NACK shows up as a timeout.
**/
static XFER_RESULT
HsTransfer (
  PDEVICE_CONTEXT  Ctx,
  USHORT           Address,
  BOOLEAN          IsRead,
  PMDL             Mdl,
  SIZE_T           Length,
  BOOLEAN          Stop,
  SIZE_T           *Done
  )
{
  MDL_CURSOR  Cursor;
  PUCHAR      Byte;
  ULONG       Ctl;
  ULONG       Auto;
  ULONG       Waited;

  *Done         = 0;
  Cursor.Mdl    = Mdl;
  Cursor.Va     = NULL;
  Cursor.Offset = 0;
  if (Length > 0xFFFF) {
    return XferTimeout;
  }

  RegWrite (Ctx, HSI2C_TIMEOUT, RegRead (Ctx, HSI2C_TIMEOUT) & ~HSTIMEOUT_EN);
  RegWrite (Ctx, HSI2C_FIFO_CTL, HSFIFO_CTL_DEFAULT);

  Ctl  = RegRead (Ctx, HSI2C_CTL) & ~(HSCTL_RXCHON | HSCTL_TXCHON);
  Ctl |= IsRead ? HSCTL_RXCHON : HSCTL_TXCHON;
  Auto  = RegRead (Ctx, HSI2C_AUTO_CONFIG) & ~(HSAUTO_READ_WRITE | HSAUTO_STOP_AFTER | 0xFFFF);
  Auto |= IsRead ? HSAUTO_READ_WRITE : 0;
  Auto |= Stop ? HSAUTO_STOP_AFTER : 0;
  Auto |= (ULONG)Length;

  RegWrite (Ctx, HSI2C_ADDR, ((ULONG)Address & 0x7F) << 10);
  RegWrite (Ctx, HSI2C_CTL, Ctl);
  RegWrite (Ctx, HSI2C_AUTO_CONFIG, Auto);
  RegWrite (Ctx, HSI2C_AUTO_CONFIG, Auto | HSAUTO_MASTER_RUN);

  Waited = 0;
  if (IsRead) {
    while (*Done < Length) {
      if ((RegRead (Ctx, HSI2C_FIFO_STATUS) & HSFIFO_RX_EMPTY) == 0) {
        if (!CursorNext (&Cursor, &Byte)) {
          return XferTimeout;
        }
        *Byte = (UCHAR)RegRead (Ctx, HSI2C_RX_DATA);
        (*Done)++;
        continue;
      }
      if (Waited++ >= HSI2C_MSG_TIMEOUT_US) {
        return (*Done == 0) ? XferNack : XferTimeout;
      }
      KeStallExecutionProcessor (1);
    }
    return XferOk;
  }

  while (*Done < Length) {
    if (HSFIFO_TX_LEVEL (RegRead (Ctx, HSI2C_FIFO_STATUS)) < 64) {
      if (!CursorNext (&Cursor, &Byte)) {
        return XferTimeout;
      }
      RegWrite (Ctx, HSI2C_TX_DATA, *Byte);
      (*Done)++;
      continue;
    }
    if (Waited++ >= HSI2C_MSG_TIMEOUT_US) {
      return XferTimeout;
    }
    KeStallExecutionProcessor (1);
  }

  for (Waited = 0; Waited < HSI2C_MSG_TIMEOUT_US; Waited++) {
    ULONG  Fifo  = RegRead (Ctx, HSI2C_FIFO_STATUS);
    ULONG  Trans = RegRead (Ctx, HSI2C_TRANS_STATUS);

    if ((Fifo == HSFIFO_ALL_EMPTY) &&
        ((Trans == 0) || (!Stop && (Trans == HSTRANS_IDLE_NO_STOP)))) {
      return XferOk;
    }
    KeStallExecutionProcessor (1);
  }
  return (Length != 0) ? XferNack : XferTimeout;
}

/**
  Run Count transfers of Request back to back and complete it.
**/
static VOID
I2cRunRequest (
  WDFDEVICE   Controller,
  SPBTARGET   Target,
  SPBREQUEST  Request,
  ULONG       Count
  )
{
  PDEVICE_CONTEXT          Ctx;
  PTARGET_CONTEXT          Tgt;
  SPB_TRANSFER_DESCRIPTOR  Desc;
  PMDL                     Mdl;
  ULONG                    Index;
  SIZE_T                   Done;
  SIZE_T                   Total;
  XFER_RESULT              Result;
  BOOLEAN                  IsRead;
  NTSTATUS                 Status;

  Ctx    = GetDeviceContext (Controller);
  Tgt    = GetTargetContext (Target);
  Total  = 0;
  Result = XferOk;
  IsRead = FALSE;

  if (Ctx->HighSpeed) {
    HsInit (Ctx, Tgt->Speed);
  } else {
    I2cInit (Ctx, Tgt->Speed);
  }

  for (Index = 0; (Index < Count) && (Result == XferOk); Index++) {
    SPB_TRANSFER_DESCRIPTOR_INIT (&Desc);
    SpbRequestGetTransferParameters (Request, Index, &Desc, &Mdl);
    if (Desc.DelayInUs != 0) {
      KeStallExecutionProcessor (Desc.DelayInUs);
    }
    IsRead = (Desc.Direction == SpbTransferDirectionFromDevice);
    if (Ctx->HighSpeed) {
      Result = HsTransfer (Ctx, Tgt->Address, IsRead, Mdl, Desc.TransferLength, Index == Count - 1, &Done);
    } else {
      Result = I2cTransfer (Ctx, Tgt->Address, IsRead, Mdl, Desc.TransferLength, Index != 0, &Done);
    }
    Total += Done;
  }

  if (!Ctx->HighSpeed) {
    I2cStop (Ctx, IsRead);
  } else if (Result != XferOk) {
    HsInit (Ctx, Tgt->Speed);       // leave the controller usable
  }

  if (Result != XferOk) {
    OsLog ("exynosi2c: 0x%02x transfer %lu of %lu: %s after %Iu bytes", Tgt->Address, Index,
           Count, (Result == XferNack) ? "NACK" : "timeout", Total);
  }

  if (Result == XferOk) {
    Status = STATUS_SUCCESS;
  } else if ((Result == XferNack) && (Total == 0)) {
    Status = STATUS_NO_SUCH_DEVICE;       // address not acknowledged
  } else if (Result == XferNack) {
    Status = STATUS_SUCCESS;              // partial write: SPB reports the count
  } else {
    Status = STATUS_IO_TIMEOUT;
  }

  WdfRequestSetInformation ((WDFREQUEST)Request, Total);
  SpbRequestComplete (Request, Status);
}

VOID
EvtIoRead (WDFDEVICE Controller, SPBTARGET Target, SPBREQUEST Request, size_t Length)
{
  UNREFERENCED_PARAMETER (Length);
  I2cRunRequest (Controller, Target, Request, 1);
}

VOID
EvtIoWrite (WDFDEVICE Controller, SPBTARGET Target, SPBREQUEST Request, size_t Length)
{
  UNREFERENCED_PARAMETER (Length);
  I2cRunRequest (Controller, Target, Request, 1);
}

VOID
EvtIoSequence (WDFDEVICE Controller, SPBTARGET Target, SPBREQUEST Request, ULONG TransferCount)
{
  I2cRunRequest (Controller, Target, Request, TransferCount);
}

/**
  Pick the slave address and speed out of the target's I2cSerialBus
  connection descriptor.
**/
NTSTATUS
EvtTargetConnect (WDFDEVICE Controller, SPBTARGET Target)
{
  SPB_CONNECTION_PARAMETERS                     Params;
  PRH_QUERY_CONNECTION_PROPERTIES_OUTPUT_BUFFER Conn;
  PPNP_SERIAL_BUS_DESCRIPTOR                    Bus;
  PPNP_I2C_SERIAL_BUS_DESCRIPTOR                I2c;
  PTARGET_CONTEXT                               Tgt;

  UNREFERENCED_PARAMETER (Controller);

  OsLog ("exynosi2c: target connect");
  SPB_CONNECTION_PARAMETERS_INIT (&Params);
  SpbTargetGetConnectionParameters (Target, &Params);
  Conn = (PRH_QUERY_CONNECTION_PROPERTIES_OUTPUT_BUFFER)Params.ConnectionParameters;
  OsLog ("exynosi2c: connection parameters at %p", Conn);
  if (Conn == NULL) {
    return STATUS_INVALID_PARAMETER;
  }
  OsLog ("exynosi2c: properties %lu bytes", Conn->PropertiesLength);
  if (Conn->PropertiesLength < sizeof (PNP_I2C_SERIAL_BUS_DESCRIPTOR)) {
    return STATUS_INVALID_PARAMETER;
  }

  Bus = (PPNP_SERIAL_BUS_DESCRIPTOR)Conn->ConnectionProperties;
  if (Bus->SerialBusType != I2C_SERIAL_BUS_TYPE) {
    return STATUS_INVALID_PARAMETER;
  }

  I2c = (PPNP_I2C_SERIAL_BUS_DESCRIPTOR)Bus;
  if ((I2c->SerialBusDescriptor.TypeSpecificFlags & I2C_SERIAL_BUS_SPECIFIC_FLAG_10BIT_ADDRESS) != 0) {
    return STATUS_NOT_SUPPORTED;
  }

  Tgt          = GetTargetContext (Target);
  Tgt->Address = I2c->SlaveAddress;
  Tgt->Speed   = (I2c->ConnectionSpeed != 0) ? I2c->ConnectionSpeed : I2C_DEFAULT_HZ;
  OsLog ("exynosi2c: target 0x%02x at %lu Hz", Tgt->Address, Tgt->Speed);
  return STATUS_SUCCESS;
}

/**
  EXYN5004 is a high-speed controller, anything else s3c2410-style.
  Windows 8.1 lists the ACPI ID both as ACPI\VEN_EXYN&DEV_5004 and as
  ACPI\EXYN5004 (plus *EXYN5004): match "EXYN5004" or "DEV_5004" anywhere
  in the hardware ID multi-sz.
**/
static BOOLEAN
MatchAt (const WCHAR *Ids, ULONG Count, ULONG Index, const WCHAR *Text)
{
  ULONG  Offset;

  for (Offset = 0; Text[Offset] != 0; Offset++) {
    if ((Index + Offset >= Count) || (Ids[Index + Offset] != Text[Offset])) {
      return FALSE;
    }
  }
  return TRUE;
}

static BOOLEAN
IsHighSpeedController (WDFDEVICE Device)
{
  WCHAR     Ids[256];
  ULONG     Length;
  ULONG     Count;
  ULONG     Index;
  NTSTATUS  Status;

  RtlZeroMemory (Ids, sizeof (Ids));
  Status = WdfDeviceQueryProperty (Device, DevicePropertyHardwareID, sizeof (Ids) - 2 * sizeof (WCHAR),
                                   Ids, &Length);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosi2c: hardware ID query failed: 0x%08lx", Status);
    return FALSE;
  }
  Count = min (Length / sizeof (WCHAR), ARRAYSIZE (Ids));
  for (Index = 0; Index < Count; Index++) {
    if (MatchAt (Ids, Count, Index, L"EXYN5004") || MatchAt (Ids, Count, Index, L"DEV_5004")) {
      return TRUE;
    }
  }
  return FALSE;
}

NTSTATUS
EvtPrepareHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
{
  PDEVICE_CONTEXT                  Ctx;
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  ULONG                            Index;
  ULONG                            Count;
  ULONG_PTR                        Gsi;

  Ctx   = GetDeviceContext (Device);
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
    } else if (Desc->Type == CmResourceTypeInterrupt) {
      Gsi = WdfCmResourceListGetDescriptor (ResourcesRaw, Index)->u.Interrupt.Vector;
    }
  }

  if (Ctx->Base == NULL) {
    OsLog ("exynosi2c: no memory resource");
    return STATUS_DEVICE_CONFIGURATION_ERROR;
  }
  Ctx->HighSpeed = IsHighSpeedController (Device);
  OsLog ("exynosi2c: %s controller at 0x%08lx, GSI %Iu, %lu resources",
         Ctx->HighSpeed ? "hs-i2c" : "i2c",
         WdfCmResourceListGetDescriptor (ResourcesRaw, 0)->u.Memory.Start.LowPart, Gsi, Count);

#if LOAD_TEST
  KeBugCheckEx (LOAD_TEST_BUGCHECK,
                (ULONG_PTR)WdfCmResourceListGetDescriptor (ResourcesRaw, 0)->u.Memory.Start.QuadPart,
                Gsi, Count, 0);
#else
  UNREFERENCED_PARAMETER (Gsi);
#endif
  return STATUS_SUCCESS;
}

NTSTATUS
EvtReleaseHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesTranslated)
{
  PDEVICE_CONTEXT  Ctx;

  UNREFERENCED_PARAMETER (ResourcesTranslated);

  Ctx = GetDeviceContext (Device);
  if (Ctx->Base != NULL) {
    MmUnmapIoSpace (Ctx->Base, Ctx->Size);
    Ctx->Base = NULL;
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

  //
  // Passive level: transfers poll with KeStallExecutionProcessor.
  //
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
  Status = SpbDeviceInitialize (Device, &SpbConfig);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, TARGET_CONTEXT);
  SpbControllerSetTargetAttributes (Device, &Attributes);
  return STATUS_SUCCESS;
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG  Config;

  OsLogInit ();
  OsLog ("exynosi2c: DriverEntry");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  return WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, WDF_NO_HANDLE);
}
