/** @file
  Exynos5420 UART (s3c24xx / exynos4210-uart) as a SerCx2 controller
  driver for Windows RT 8.1 (KMDF 1.13).  v1awifi: UART0 (0x12C00000,
  SPI 51 = GSI 83, 256-byte FIFOs, RTS/CTS on GPA0[2]/[3]) carries the
  Broadcom Bluetooth controller; the BT driver (exynosbt) opens it through
  the resource hub (UartSerialBus).

  Transfers of DMA_MIN_TRANSACTION bytes and more use system DMA (PL330
  PDMA0 through the halextpl330 HAL extension, FixedDMA request lines 45 /
  46 = UART0 RX / TX); SerCx2 needs PIO as well and uses it for the short
  ones.  Everything is interrupt driven: the receive interrupt (FIFO trigger
  or receive timeout) and the transmit interrupt (FIFO at or below its
  trigger) are unmasked only while SerCx2 waits for them (ready / new-data
  notifications); the ISR masks the source again and the DPC tells SerCx2.
  During a DMA transaction the UART's RX / TX mode is DMA (single 1-byte
  requests, RX trigger 1 byte so nothing waits in the FIFO); the receive
  timeout still interrupts and serves as SerCx2's new-data notification.

  Clock: the driver gives UART0 the same source as UART2 (the console the
  bootloader runs at 115200) and picks the divider that serves 3 Mbaud
  best; the source rate comes from that PLL's M/P/S (CMU_TOP).  Pins
  GPA0[0..3] are switched to the UART (function 2, no pull).

  Registers: include/linux/serial_s3c.h, drivers/tty/serial/samsung_tty.c;
  clocks: drivers/clk/samsung/clk-exynos5420.c.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#define OSLOG_GLOBAL
#include <ExynosEnv.h>
#include <wdf.h>
#include <ntddser.h>
#include <sercx/2.0/sercx.h>
#include <reshub.h>

//
// UART registers.
//
#define ULCON          0x00
#define ULCON_CS8      0x3
#define ULCON_STOPB    (1u << 2)
#define ULCON_PARITY_SHIFT 3
#define UCON           0x04
#define UCON_DEFAULT   ((1u << 9) | (1u << 8) | (1u << 7) | (1u << 6) | (1u << 2) | (1u << 0) | (3u << 12))
                       // TX/RX level interrupts, RX timeout (4 frames x 8), error interrupt, IRQ mode
#define UCON_SBREAK    (1u << 4)
#define UCON_RXMODE    (3u << 0)
#define UCON_RX_DMA    (2u << 0)
#define UCON_RX_IRQ    (1u << 0)
#define UCON_TXMODE    (3u << 2)
#define UCON_TX_DMA    (2u << 2)
#define UCON_TX_IRQ    (1u << 2)
#define UFCON          0x08
#define UFCON_FIFO     (1u << 0)
#define UFCON_RXRESET  (1u << 1)
#define UFCON_TXRESET  (1u << 2)
#define UFCON_TRIG     ((4u << 4) | (4u << 8))   // RX at 32 bytes, TX at 32 bytes (256-byte FIFOs)
#define UFCON_RXTRIG_MASK (7u << 4)
#define UFCON_RXTRIG_32   (4u << 4)
#define UFCON_RXTRIG_1    (0u << 4)
#define UMCON          0x0C
#define UMCON_RTS      (1u << 0)                 // drives nRTS active
#define UMCON_AFC      (1u << 4)
#define UMCON_AFC_160  (3u << 5)                 // nRTS deasserted at 160 bytes (256-byte FIFO)
#define UTRSTAT        0x10
#define UTRSTAT_TXE    (1u << 2)
#define UTRSTAT_TIMEOUT (1u << 3)
#define UERSTAT        0x14
#define UFSTAT         0x18
#define UFSTAT_RXCNT(v)  ((v) & 0xFF)
#define UFSTAT_RXFULL    (1u << 8)
#define UFSTAT_TXCNT(v)  (((v) >> 16) & 0xFF)
#define UFSTAT_TXFULL    (1u << 24)
#define UMSTAT         0x1C
#define UMSTAT_CTS     (1u << 0)
#define UTXH           0x20
#define URXH           0x24
#define UBRDIV         0x28
#define UFRACVAL       0x2C
#define UINTP          0x30
#define UINTM          0x38
#define UINT_RXD       (1u << 0)
#define UINT_ERROR     (1u << 1)
#define UINT_TXD       (1u << 2)
#define UINT_MODEM     (1u << 3)

#define FIFO_SIZE      256

//
// CMU_TOP (0x10020000): UART0 mux SRC_PERIC0[6:4], UART2 [14:12]; divider
// DIV_PERIC0[11:8] / [19:16]; gates.  mout_group2: fin_pll, cpll, dpll,
// mpll, spll, ipll, epll, rpll.
//
#define CMU_TOP_PHYS        0x10020000
#define SRC_PERIC0          0x250
#define DIV_PERIC0          0x558
#define GATE_TOP_SCLK_PERIC 0x850
#define GATE_IP_PERIC       0x950
#define GATE_BUS_FSYS0      0x740                // bit 1: PDMA0
#define XTAL_HZ             24000000u
static CONST USHORT mPllCon0[8] = { 0, 0x120, 0x128, 0x180, 0x160, 0x150, 0x130, 0x140 };
static CONST CHAR  *mPllName[8] = { "fin", "cpll", "dpll", "mpll", "spll", "ipll", "epll", "rpll" };

#define GPIO_B_PHYS         0x14010000           // GPA0 at +0x00 (CON, DAT, PUD)
#define TARGET_BAUD_FOR_CLK 3000000u
#define MAX_SCLK_HZ         150000000u
#define DMA_MAX_TRANSFER    4096
#define DMA_MIN_TRANSACTION 32

typedef struct {
  PUCHAR              Regs;
  PUCHAR              Cmu;
  PUCHAR              Gpio;
  WDFINTERRUPT        Interrupt;
  SERCX2PIORECEIVE    PioReceive;
  SERCX2PIOTRANSMIT   PioTransmit;
  SERCX2SYSTEMDMARECEIVE  DmaReceive;
  SERCX2SYSTEMDMATRANSMIT DmaTransmit;
  PHYSICAL_ADDRESS    RegsPhys;
  BOOLEAN             RxDma;             // UART RX in DMA mode (a DMA receive is running)
  BOOLEAN             NewDataNotify;     // under the interrupt lock
  ULONG               DmaRx;
  ULONG               DmaTx;
  ULONG               SclkHz;
  ULONG               BaudRate;
  SERIAL_LINE_CONTROL LineControl;
  SERIAL_HANDFLOW     HandFlow;
  BOOLEAN             Rts;
  BOOLEAN             Dtr;
  BOOLEAN             RxNotify;          // under the interrupt lock
  BOOLEAN             TxNotify;
  volatile LONG       Events;            // UINT_* seen by the ISR, for the DPC
  ULONG               Errors;
  ULONG               Ioctls;
} UART_CONTEXT, *PUART_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (UART_CONTEXT, GetUart)

//
// The SerCx2 PIO objects carry their device (no SerCx2 getter for it).
//
typedef struct {
  WDFDEVICE  Device;
} PIO_CONTEXT;           // also on the system DMA objects

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (PIO_CONTEXT, GetPio)

DRIVER_INITIALIZE                               DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD                       EvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE                 EvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE                 EvtReleaseHardware;
EVT_WDF_DEVICE_D0_ENTRY                         EvtD0Entry;
EVT_WDF_DEVICE_D0_EXIT                          EvtD0Exit;
EVT_WDF_INTERRUPT_ISR                           EvtIsr;
EVT_WDF_INTERRUPT_DPC                           EvtDpc;
EVT_SERCX2_APPLY_CONFIG                         EvtApplyConfig;
EVT_SERCX2_CONTROL                              EvtControl;
EVT_SERCX2_PURGE_FIFOS                          EvtPurgeFifos;
EVT_SERCX2_PIO_RECEIVE_READ_BUFFER              EvtRxRead;
EVT_SERCX2_PIO_RECEIVE_ENABLE_READY_NOTIFICATION EvtRxEnable;
EVT_SERCX2_PIO_RECEIVE_CANCEL_READY_NOTIFICATION EvtRxCancel;
EVT_SERCX2_PIO_TRANSMIT_WRITE_BUFFER            EvtTxWrite;
EVT_SERCX2_PIO_TRANSMIT_ENABLE_READY_NOTIFICATION EvtTxEnable;
EVT_SERCX2_PIO_TRANSMIT_CANCEL_READY_NOTIFICATION EvtTxCancel;
EVT_SERCX2_SYSTEM_DMA_RECEIVE_INITIALIZE_TRANSACTION      EvtDmaRxInit;
EVT_SERCX2_SYSTEM_DMA_RECEIVE_CLEANUP_TRANSACTION         EvtDmaRxCleanup;
EVT_SERCX2_SYSTEM_DMA_RECEIVE_ENABLE_NEW_DATA_NOTIFICATION EvtDmaRxEnableNewData;
EVT_SERCX2_SYSTEM_DMA_RECEIVE_CANCEL_NEW_DATA_NOTIFICATION EvtDmaRxCancelNewData;
EVT_SERCX2_SYSTEM_DMA_TRANSMIT_INITIALIZE_TRANSACTION     EvtDmaTxInit;
EVT_SERCX2_SYSTEM_DMA_TRANSMIT_CLEANUP_TRANSACTION        EvtDmaTxCleanup;

#define RD(o)     READ_REGISTER_ULONG ((PULONG)(Ctx->Regs + (o)))
#define WR(o, v)  WRITE_REGISTER_ULONG ((PULONG)(Ctx->Regs + (o)), (v))
#define CMU(o)    ((PULONG)(Ctx->Cmu + (o)))

static ULONG
PllHz (ULONG Con)
{
  ULONG  M = (Con >> 16) & 0x3FF;
  ULONG  P = (Con >> 8) & 0x3F;
  ULONG  S = Con & 0x7;

  if (P == 0) {
    return 0;
  }
  return (ULONG)(((ULONGLONG)XTAL_HZ * M) / ((ULONGLONG)P << S));
}

/**
  UART0 gets UART2's clock source; the divider (0..15) with the smallest
  3 Mbaud error (UBRDIV + UFRACVAL / 16 resolution) that keeps SCLK under
  MAX_SCLK_HZ.
**/
static VOID
SetupClock (PUART_CONTEXT Ctx)
{
  ULONG  Src = READ_REGISTER_ULONG (CMU (SRC_PERIC0));
  ULONG  Mux = (Src >> 12) & 7;          // UART2's
  ULONG  Parent = (Mux == 0) ? XTAL_HZ : PllHz (READ_REGISTER_ULONG (CMU (mPllCon0[Mux])));
  ULONG  BestDiv = 0;
  ULONG  BestErr = MAXULONG;
  ULONG  Div;

  for (Div = 0; Div < 16; Div++) {
    ULONG      Sclk = Parent / (Div + 1);
    ULONGLONG  Sixteenths;
    ULONG      Actual;
    ULONG      Err;

    if ((Sclk > MAX_SCLK_HZ) || (Sclk < 16 * TARGET_BAUD_FOR_CLK)) {
      continue;
    }
    Sixteenths = ((ULONGLONG)Sclk + TARGET_BAUD_FOR_CLK / 2) / TARGET_BAUD_FOR_CLK;   // divisor in 1/16
    Actual = (ULONG)((ULONGLONG)Sclk / Sixteenths);
    Err = (Actual > TARGET_BAUD_FOR_CLK) ? Actual - TARGET_BAUD_FOR_CLK : TARGET_BAUD_FOR_CLK - Actual;
    if (Err < BestErr) {
      BestErr = Err;
      BestDiv = Div;
    }
  }
  if (BestErr == MAXULONG) {
    BestDiv = (Parent > MAX_SCLK_HZ) ? (Parent + MAX_SCLK_HZ - 1) / MAX_SCLK_HZ - 1 : 0;
  }

  WRITE_REGISTER_ULONG (CMU (SRC_PERIC0), (Src & ~(7u << 4)) | (Mux << 4));
  WRITE_REGISTER_ULONG (CMU (DIV_PERIC0), (READ_REGISTER_ULONG (CMU (DIV_PERIC0)) & ~(0xFu << 8)) | (BestDiv << 8));
  WRITE_REGISTER_ULONG (CMU (GATE_TOP_SCLK_PERIC), READ_REGISTER_ULONG (CMU (GATE_TOP_SCLK_PERIC)) | 1u);
  WRITE_REGISTER_ULONG (CMU (GATE_IP_PERIC), READ_REGISTER_ULONG (CMU (GATE_IP_PERIC)) | 1u);
  WRITE_REGISTER_ULONG (CMU (GATE_BUS_FSYS0), READ_REGISTER_ULONG (CMU (GATE_BUS_FSYS0)) | (1u << 1));   // PDMA0
  Ctx->SclkHz = Parent / (BestDiv + 1);
  OsLog ("exynosuart: clock %s %lu Hz / %lu = %lu Hz (3 Mbaud error %lu)", mPllName[Mux], Parent, BestDiv + 1,
         Ctx->SclkHz, BestErr);
}

static VOID
SetupPins (PUART_CONTEXT Ctx)
{
  PULONG  Con = (PULONG)(Ctx->Gpio + 0x00);
  PULONG  Pud = (PULONG)(Ctx->Gpio + 0x08);

  WRITE_REGISTER_ULONG (Con, (READ_REGISTER_ULONG (Con) & ~0xFFFFu) | 0x2222u);
  WRITE_REGISTER_ULONG (Pud, READ_REGISTER_ULONG (Pud) & ~0xFFu);
}

static NTSTATUS
SetBaud (PUART_CONTEXT Ctx, ULONG Baud)
{
  ULONGLONG  Sixteenths;

  if ((Baud == 0) || (Ctx->SclkHz < 16 * Baud)) {
    return STATUS_INVALID_PARAMETER;
  }
  Sixteenths = ((ULONGLONG)Ctx->SclkHz + Baud / 2) / Baud;          // SCLK / baud in 1/16 units
  WR (UBRDIV, (ULONG)(Sixteenths / 16) - 1);
  WR (UFRACVAL, (ULONG)(Sixteenths % 16));
  Ctx->BaudRate = Baud;
  return STATUS_SUCCESS;
}

static VOID
ApplyLine (PUART_CONTEXT Ctx)
{
  ULONG  Lcon = ULCON_CS8;

  if (Ctx->LineControl.WordLength >= 5 && Ctx->LineControl.WordLength <= 8) {
    Lcon = Ctx->LineControl.WordLength - 5;
  }
  if (Ctx->LineControl.StopBits == STOP_BITS_2) {
    Lcon |= ULCON_STOPB;
  }
  switch (Ctx->LineControl.Parity) {
    case ODD_PARITY:   Lcon |= 4u << ULCON_PARITY_SHIFT; break;
    case EVEN_PARITY:  Lcon |= 5u << ULCON_PARITY_SHIFT; break;
    case MARK_PARITY:  Lcon |= 6u << ULCON_PARITY_SHIFT; break;
    case SPACE_PARITY: Lcon |= 7u << ULCON_PARITY_SHIFT; break;
    default: break;
  }
  WR (ULCON, Lcon);
}

static VOID
ApplyModem (PUART_CONTEXT Ctx)
{
  BOOLEAN  Afc = (Ctx->HandFlow.ControlHandShake & SERIAL_CTS_HANDSHAKE) != 0;

  //
  // With CTS handshake the controller does both directions (nRTS from its
  // RX FIFO level, TX held while nCTS is inactive); otherwise nRTS follows
  // SET_RTS / CLR_RTS.
  //
  WR (UMCON, Afc ? (UMCON_AFC | UMCON_AFC_160) : (Ctx->Rts ? UMCON_RTS : 0));
}

static VOID
HwInit (PUART_CONTEXT Ctx)
{
  SetupClock (Ctx);
  SetupPins (Ctx);
  WR (UINTM, 0xF);                       // all masked
  WR (UINTP, 0xF);
  WR (UFCON, UFCON_FIFO | UFCON_RXRESET | UFCON_TXRESET | UFCON_TRIG);
  WR (UCON, UCON_DEFAULT);
  ApplyLine (Ctx);
  SetBaud (Ctx, Ctx->BaudRate);
  ApplyModem (Ctx);
  WR (UINTM, 0xF & ~UINT_ERROR);         // errors always reported
}

//
// ---------------------------------------------------------------------------
// Interrupt
// ---------------------------------------------------------------------------
//

BOOLEAN
EvtIsr (WDFINTERRUPT Interrupt, ULONG MessageId)
{
  PUART_CONTEXT  Ctx = GetUart (WdfInterruptGetDevice (Interrupt));
  ULONG          Pending;
  ULONG          Mask;

  UNREFERENCED_PARAMETER (MessageId);

  Pending = RD (UINTP) & 0xF;
  if (Pending == 0) {
    return FALSE;
  }
  // Ready notifications are one-shot: mask what fired until re-enabled.
  Mask = RD (UINTM);
  if ((Pending & UINT_RXD) != 0) {
    Mask |= UINT_RXD;
    if (Ctx->RxDma) {
      WR (UTRSTAT, UTRSTAT_TIMEOUT);     // DMA mode: the receive timeout, clear it
    }
  }
  if ((Pending & UINT_TXD) != 0) {
    Mask |= UINT_TXD;
  }
  if ((Pending & UINT_ERROR) != 0) {
    (VOID)RD (UERSTAT);                  // reading clears it
    Ctx->Errors++;
  }
  WR (UINTM, Mask);
  WR (UINTP, Pending);
  InterlockedOr (&Ctx->Events, (LONG)Pending);
  WdfInterruptQueueDpcForIsr (Interrupt);
  return TRUE;
}

VOID
EvtDpc (WDFINTERRUPT Interrupt, WDFOBJECT Device)
{
  PUART_CONTEXT  Ctx = GetUart (Device);
  LONG           Events = InterlockedExchange (&Ctx->Events, 0);
  BOOLEAN        Rx = FALSE;
  BOOLEAN        Tx = FALSE;
  BOOLEAN        NewData = FALSE;

  WdfInterruptAcquireLock (Interrupt);
  if (((Events & UINT_RXD) != 0) && Ctx->RxNotify) {
    Ctx->RxNotify = FALSE;
    Rx = TRUE;
  }
  if (((Events & UINT_RXD) != 0) && Ctx->NewDataNotify) {
    Ctx->NewDataNotify = FALSE;
    NewData = TRUE;
  }
  if (((Events & UINT_TXD) != 0) && Ctx->TxNotify) {
    Ctx->TxNotify = FALSE;
    Tx = TRUE;
  }
  WdfInterruptReleaseLock (Interrupt);
  if (Rx) {
    SerCx2PioReceiveReady (Ctx->PioReceive);
  }
  if (NewData) {
    SerCx2SystemDmaReceiveNewDataNotification (Ctx->DmaReceive);
  }
  if (Tx) {
    SerCx2PioTransmitReady (Ctx->PioTransmit);
  }
}

//
// ---------------------------------------------------------------------------
// PIO receive / transmit
// ---------------------------------------------------------------------------
//

static ULONG
RxCount (PUART_CONTEXT Ctx)
{
  ULONG  Stat = RD (UFSTAT);

  return ((Stat & UFSTAT_RXFULL) != 0) ? FIFO_SIZE : UFSTAT_RXCNT (Stat);
}

ULONG
EvtRxRead (SERCX2PIORECEIVE PioReceive, PUCHAR Buffer, ULONG Length)
{
  PUART_CONTEXT  Ctx = GetUart (GetPio (PioReceive)->Device);
  ULONG          Done = 0;
  ULONG          Avail = RxCount (Ctx);

  while ((Done < Length) && (Avail != 0)) {
    Buffer[Done++] = (UCHAR)RD (URXH);
    if (--Avail == 0) {
      Avail = RxCount (Ctx);
    }
  }
  return Done;
}

VOID
EvtRxEnable (SERCX2PIORECEIVE PioReceive)
{
  WDFDEVICE      Device = GetPio (PioReceive)->Device;
  PUART_CONTEXT  Ctx = GetUart (Device);
  BOOLEAN        Now;

  WdfInterruptAcquireLock (Ctx->Interrupt);
  Ctx->RxNotify = TRUE;
  WR (UINTM, RD (UINTM) & ~UINT_RXD);
  // Bytes below the trigger whose timeout already passed: report them now.
  Now = (RxCount (Ctx) != 0);
  WdfInterruptReleaseLock (Ctx->Interrupt);
  if (Now) {
    InterlockedOr (&Ctx->Events, UINT_RXD);
    WdfInterruptQueueDpcForIsr (Ctx->Interrupt);
  }
}

BOOLEAN
EvtRxCancel (SERCX2PIORECEIVE PioReceive)
{
  PUART_CONTEXT  Ctx = GetUart (GetPio (PioReceive)->Device);
  BOOLEAN        Was;

  WdfInterruptAcquireLock (Ctx->Interrupt);
  Was = Ctx->RxNotify;
  Ctx->RxNotify = FALSE;
  if (!Ctx->NewDataNotify) {
    WR (UINTM, RD (UINTM) | UINT_RXD);
  }
  WdfInterruptReleaseLock (Ctx->Interrupt);
  return Was;
}

ULONG
EvtTxWrite (SERCX2PIOTRANSMIT PioTransmit, PUCHAR Buffer, ULONG Length)
{
  PUART_CONTEXT  Ctx = GetUart (GetPio (PioTransmit)->Device);
  ULONG          Done = 0;

  while ((Done < Length) && ((RD (UFSTAT) & UFSTAT_TXFULL) == 0)) {
    WR (UTXH, Buffer[Done++]);
  }
  return Done;
}

VOID
EvtTxEnable (SERCX2PIOTRANSMIT PioTransmit)
{
  PUART_CONTEXT  Ctx = GetUart (GetPio (PioTransmit)->Device);

  WdfInterruptAcquireLock (Ctx->Interrupt);
  Ctx->TxNotify = TRUE;
  WR (UINTM, RD (UINTM) & ~UINT_TXD);    // level: fires at once if the FIFO has room
  WdfInterruptReleaseLock (Ctx->Interrupt);
}

BOOLEAN
EvtTxCancel (SERCX2PIOTRANSMIT PioTransmit)
{
  PUART_CONTEXT  Ctx = GetUart (GetPio (PioTransmit)->Device);
  BOOLEAN        Was;

  WdfInterruptAcquireLock (Ctx->Interrupt);
  Was = Ctx->TxNotify;
  Ctx->TxNotify = FALSE;
  WR (UINTM, RD (UINTM) | UINT_TXD);
  WdfInterruptReleaseLock (Ctx->Interrupt);
  return Was;
}

VOID
EvtPurgeFifos (WDFDEVICE Device, BOOLEAN PurgeRxFifo, BOOLEAN PurgeTxFifo)
{
  PUART_CONTEXT  Ctx = GetUart (Device);
  ULONG          Fcon = RD (UFCON);

  if (PurgeRxFifo) {
    Fcon |= UFCON_RXRESET;
  }
  if (PurgeTxFifo) {
    Fcon |= UFCON_TXRESET;
  }
  WR (UFCON, Fcon);
}

//
// ---------------------------------------------------------------------------
// System DMA (PL330 PDMA0 through halextpl330): the UART side
// ---------------------------------------------------------------------------
//

VOID
EvtDmaRxInit (SERCX2SYSTEMDMARECEIVE DmaReceive, ULONG Length)
{
  PUART_CONTEXT  Ctx = GetUart (GetPio (DmaReceive)->Device);

  UNREFERENCED_PARAMETER (Length);

  WdfInterruptAcquireLock (Ctx->Interrupt);
  Ctx->RxDma = TRUE;
  WR (UFCON, (RD (UFCON) & ~(UFCON_RXTRIG_MASK | UFCON_RXRESET | UFCON_TXRESET)) | UFCON_RXTRIG_1);
  WR (UCON, (RD (UCON) & ~UCON_RXMODE) | UCON_RX_DMA);
  WdfInterruptReleaseLock (Ctx->Interrupt);
  if (Ctx->DmaRx++ < 4) {
    OsLog ("exynosuart: DMA receive of %lu bytes", Length);
  }
  SerCx2SystemDmaReceiveInitializeTransactionComplete (DmaReceive, TRUE);
}

VOID
EvtDmaRxCleanup (SERCX2SYSTEMDMARECEIVE DmaReceive)
{
  PUART_CONTEXT  Ctx = GetUart (GetPio (DmaReceive)->Device);

  WdfInterruptAcquireLock (Ctx->Interrupt);
  WR (UCON, (RD (UCON) & ~UCON_RXMODE) | UCON_RX_IRQ);
  WR (UFCON, (RD (UFCON) & ~(UFCON_RXTRIG_MASK | UFCON_RXRESET | UFCON_TXRESET)) | UFCON_RXTRIG_32);
  Ctx->RxDma = FALSE;
  WdfInterruptReleaseLock (Ctx->Interrupt);
  SerCx2SystemDmaReceiveCleanupTransactionComplete (DmaReceive);
}

VOID
EvtDmaRxEnableNewData (SERCX2SYSTEMDMARECEIVE DmaReceive)
{
  PUART_CONTEXT  Ctx = GetUart (GetPio (DmaReceive)->Device);

  WdfInterruptAcquireLock (Ctx->Interrupt);
  Ctx->NewDataNotify = TRUE;
  WR (UINTM, RD (UINTM) & ~UINT_RXD);    // the receive timeout
  WdfInterruptReleaseLock (Ctx->Interrupt);
}

BOOLEAN
EvtDmaRxCancelNewData (SERCX2SYSTEMDMARECEIVE DmaReceive)
{
  PUART_CONTEXT  Ctx = GetUart (GetPio (DmaReceive)->Device);
  BOOLEAN        Was;

  WdfInterruptAcquireLock (Ctx->Interrupt);
  Was = Ctx->NewDataNotify;
  Ctx->NewDataNotify = FALSE;
  if (!Ctx->RxNotify) {
    WR (UINTM, RD (UINTM) | UINT_RXD);
  }
  WdfInterruptReleaseLock (Ctx->Interrupt);
  return Was;
}

VOID
EvtDmaTxInit (SERCX2SYSTEMDMATRANSMIT DmaTransmit, ULONG Length)
{
  PUART_CONTEXT  Ctx = GetUart (GetPio (DmaTransmit)->Device);

  UNREFERENCED_PARAMETER (Length);

  WdfInterruptAcquireLock (Ctx->Interrupt);
  WR (UCON, (RD (UCON) & ~UCON_TXMODE) | UCON_TX_DMA);
  WdfInterruptReleaseLock (Ctx->Interrupt);
  if (Ctx->DmaTx++ < 4) {
    OsLog ("exynosuart: DMA transmit of %lu bytes", Length);
  }
  SerCx2SystemDmaTransmitInitializeTransactionComplete (DmaTransmit, TRUE);
}

VOID
EvtDmaTxCleanup (SERCX2SYSTEMDMATRANSMIT DmaTransmit)
{
  PUART_CONTEXT  Ctx = GetUart (GetPio (DmaTransmit)->Device);

  WdfInterruptAcquireLock (Ctx->Interrupt);
  WR (UCON, (RD (UCON) & ~UCON_TXMODE) | UCON_TX_IRQ);
  WdfInterruptReleaseLock (Ctx->Interrupt);
  SerCx2SystemDmaTransmitCleanupTransactionComplete (DmaTransmit);
}

/**
  System DMA receive / transmit objects from the FixedDMA resources (RX
  first, TX second); without them (no PL330 extension / resources) the
  UART runs PIO only.
**/
static VOID
CreateDma (WDFDEVICE Device, PUART_CONTEXT Ctx, PCM_PARTIAL_RESOURCE_DESCRIPTOR RxDesc,
           PCM_PARTIAL_RESOURCE_DESCRIPTOR TxDesc)
{
  SERCX2_SYSTEM_DMA_RECEIVE_CONFIG   Rx;
  SERCX2_SYSTEM_DMA_TRANSMIT_CONFIG  Tx;
  WDF_OBJECT_ATTRIBUTES              Attributes;
  PHYSICAL_ADDRESS                   Pa;
  NTSTATUS                           Status;

  if ((RxDesc == NULL) || (TxDesc == NULL)) {
    OsLog ("exynosuart: no FixedDMA resources, PIO only");
    return;
  }
  if (Ctx->DmaReceive == NULL) {
    Pa.QuadPart = Ctx->RegsPhys.QuadPart + URXH;
    SERCX2_SYSTEM_DMA_RECEIVE_CONFIG_INIT_NEW_DATA_NOTIFICATION (&Rx, DMA_MAX_TRANSFER, Pa, Width8Bits, RxDesc,
                                                                 EvtDmaRxEnableNewData, EvtDmaRxCancelNewData);
    Rx.MinimumTransactionLength      = DMA_MIN_TRANSACTION;
    Rx.MaximumScatterGatherFragments = 16;
    Rx.EvtSerCx2SystemDmaReceiveInitializeTransaction = EvtDmaRxInit;
    Rx.EvtSerCx2SystemDmaReceiveCleanupTransaction    = EvtDmaRxCleanup;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, PIO_CONTEXT);
    Status = SerCx2SystemDmaReceiveCreate (Device, &Rx, &Attributes, &Ctx->DmaReceive);
    if (NT_SUCCESS (Status)) {
      GetPio (Ctx->DmaReceive)->Device = Device;
    } else {
      Ctx->DmaReceive = NULL;
    }
    OsLog ("exynosuart: DMA receive (request line %lu, channel %lu) - 0x%08lx", RxDesc->u.DmaV3.RequestLine,
           RxDesc->u.DmaV3.Channel, Status);
  }
  if (Ctx->DmaTransmit == NULL) {
    Pa.QuadPart = Ctx->RegsPhys.QuadPart + UTXH;
    SERCX2_SYSTEM_DMA_TRANSMIT_CONFIG_INIT (&Tx, DMA_MAX_TRANSFER, Pa, Width8Bits, TxDesc);
    Tx.MinimumTransactionLength      = DMA_MIN_TRANSACTION;
    Tx.MaximumScatterGatherFragments = 16;
    Tx.EvtSerCx2SystemDmaTransmitInitializeTransaction = EvtDmaTxInit;
    Tx.EvtSerCx2SystemDmaTransmitCleanupTransaction    = EvtDmaTxCleanup;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, PIO_CONTEXT);
    Status = SerCx2SystemDmaTransmitCreate (Device, &Tx, &Attributes, &Ctx->DmaTransmit);
    if (NT_SUCCESS (Status)) {
      GetPio (Ctx->DmaTransmit)->Device = Device;
    } else {
      Ctx->DmaTransmit = NULL;
    }
    OsLog ("exynosuart: DMA transmit (request line %lu, channel %lu) - 0x%08lx", TxDesc->u.DmaV3.RequestLine,
           TxDesc->u.DmaV3.Channel, Status);
  }
}

//
// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
//

/**
  The connection's UartSerialBus descriptor: default baud rate and flow
  control (ACPI 5.0 UART serial bus descriptor: type data = baud (ULONG),
  RX FIFO, TX FIFO (USHORT), parity, lines; type flags [1:0] flow control,
  [3:2] stop bits, [6:4] data bits).
**/
NTSTATUS
EvtApplyConfig (WDFDEVICE Device, PVOID ConnectionParameters)
{
  PUART_CONTEXT                                  Ctx = GetUart (Device);
  PRH_QUERY_CONNECTION_PROPERTIES_OUTPUT_BUFFER  Props = ConnectionParameters;
  PPNP_SERIAL_BUS_DESCRIPTOR                     Desc;
  PUCHAR                                         Data;
  ULONG                                          Baud;
  ULONG                                          Flags;

  if ((Props == NULL) || (Props->PropertiesLength < sizeof (PNP_SERIAL_BUS_DESCRIPTOR) + 4)) {
    return STATUS_SUCCESS;
  }
  Desc = (PPNP_SERIAL_BUS_DESCRIPTOR)Props->ConnectionProperties;
  if (Desc->SerialBusType != 3) {        // UART
    return STATUS_INVALID_PARAMETER;
  }
  Data  = (PUCHAR)(Desc + 1);
  Baud  = Data[0] | (Data[1] << 8) | (Data[2] << 16) | ((ULONG)Data[3] << 24);
  Flags = Desc->TypeSpecificFlags;
  Ctx->LineControl.WordLength = (UCHAR)(5 + ((Flags >> 4) & 7));
  if (Ctx->LineControl.WordLength > 8) {
    Ctx->LineControl.WordLength = 8;
  }
  Ctx->LineControl.StopBits = (((Flags >> 2) & 3) == 3) ? STOP_BITS_2 : STOP_BIT_1;
  Ctx->LineControl.Parity   = NO_PARITY;
  RtlZeroMemory (&Ctx->HandFlow, sizeof (Ctx->HandFlow));
  if ((Flags & 3) == 1) {
    Ctx->HandFlow.ControlHandShake = SERIAL_CTS_HANDSHAKE;
    Ctx->HandFlow.FlowReplace      = SERIAL_RTS_HANDSHAKE;
  }
  ApplyLine (Ctx);
  ApplyModem (Ctx);
  OsLog ("exynosuart: default configuration %lu baud, %u data bits, flow control %s", Baud,
         Ctx->LineControl.WordLength, ((Flags & 3) == 1) ? "RTS/CTS" : "none");
  return SetBaud (Ctx, Baud);
}

static NTSTATUS
InBuffer (WDFREQUEST Request, size_t Size, PVOID *Buffer)
{
  return WdfRequestRetrieveInputBuffer (Request, Size, Buffer, NULL);
}

static NTSTATUS
OutBuffer (WDFREQUEST Request, size_t Size, PVOID *Buffer)
{
  NTSTATUS  Status = WdfRequestRetrieveOutputBuffer (Request, Size, Buffer, NULL);

  if (NT_SUCCESS (Status)) {
    RtlZeroMemory (*Buffer, Size);
    WdfRequestSetInformation (Request, Size);
  }
  return Status;
}

NTSTATUS
EvtControl (WDFDEVICE Device, WDFREQUEST Request, size_t OutputBufferLength, size_t InputBufferLength,
            ULONG IoControlCode)
{
  PUART_CONTEXT  Ctx = GetUart (Device);
  PVOID          Buf;
  NTSTATUS       Status = STATUS_SUCCESS;

  UNREFERENCED_PARAMETER (OutputBufferLength);
  UNREFERENCED_PARAMETER (InputBufferLength);

  switch (IoControlCode) {
    case IOCTL_SERIAL_SET_BAUD_RATE:
      Status = InBuffer (Request, sizeof (SERIAL_BAUD_RATE), &Buf);
      if (NT_SUCCESS (Status)) {
        Status = SetBaud (Ctx, ((PSERIAL_BAUD_RATE)Buf)->BaudRate);
        OsLog ("exynosuart: baud %lu - 0x%08lx", ((PSERIAL_BAUD_RATE)Buf)->BaudRate, Status);
      }
      break;
    case IOCTL_SERIAL_GET_BAUD_RATE:
      Status = OutBuffer (Request, sizeof (SERIAL_BAUD_RATE), &Buf);
      if (NT_SUCCESS (Status)) {
        ((PSERIAL_BAUD_RATE)Buf)->BaudRate = Ctx->BaudRate;
      }
      break;
    case IOCTL_SERIAL_SET_LINE_CONTROL:
      Status = InBuffer (Request, sizeof (SERIAL_LINE_CONTROL), &Buf);
      if (NT_SUCCESS (Status)) {
        Ctx->LineControl = *(PSERIAL_LINE_CONTROL)Buf;
        ApplyLine (Ctx);
      }
      break;
    case IOCTL_SERIAL_GET_LINE_CONTROL:
      Status = OutBuffer (Request, sizeof (SERIAL_LINE_CONTROL), &Buf);
      if (NT_SUCCESS (Status)) {
        *(PSERIAL_LINE_CONTROL)Buf = Ctx->LineControl;
      }
      break;
    case IOCTL_SERIAL_SET_HANDFLOW:
      Status = InBuffer (Request, sizeof (SERIAL_HANDFLOW), &Buf);
      if (NT_SUCCESS (Status)) {
        Ctx->HandFlow = *(PSERIAL_HANDFLOW)Buf;
        ApplyModem (Ctx);
      }
      break;
    case IOCTL_SERIAL_GET_HANDFLOW:
      Status = OutBuffer (Request, sizeof (SERIAL_HANDFLOW), &Buf);
      if (NT_SUCCESS (Status)) {
        *(PSERIAL_HANDFLOW)Buf = Ctx->HandFlow;
      }
      break;
    case IOCTL_SERIAL_SET_RTS:
    case IOCTL_SERIAL_CLR_RTS:
      Ctx->Rts = (IoControlCode == IOCTL_SERIAL_SET_RTS);
      ApplyModem (Ctx);
      break;
    case IOCTL_SERIAL_SET_DTR:
    case IOCTL_SERIAL_CLR_DTR:
      Ctx->Dtr = (IoControlCode == IOCTL_SERIAL_SET_DTR);      // no DTR line
      break;
    case IOCTL_SERIAL_GET_DTRRTS:
      Status = OutBuffer (Request, sizeof (ULONG), &Buf);
      if (NT_SUCCESS (Status)) {
        *(PULONG)Buf = (Ctx->Dtr ? SERIAL_DTR_STATE : 0) | (Ctx->Rts ? SERIAL_RTS_STATE : 0);
      }
      break;
    case IOCTL_SERIAL_GET_MODEMSTATUS:
      Status = OutBuffer (Request, sizeof (ULONG), &Buf);
      if (NT_SUCCESS (Status)) {
        *(PULONG)Buf = ((RD (UMSTAT) & UMSTAT_CTS) != 0) ? SERIAL_CTS_STATE : 0;
      }
      break;
    case IOCTL_SERIAL_GET_PROPERTIES:
      Status = OutBuffer (Request, sizeof (SERIAL_COMMPROP), &Buf);
      if (NT_SUCCESS (Status)) {
        PSERIAL_COMMPROP  P = Buf;

        P->PacketLength       = sizeof (SERIAL_COMMPROP);
        P->PacketVersion      = 2;
        P->ServiceMask        = SERIAL_SP_SERIALCOMM;
        P->MaxTxQueue         = FIFO_SIZE;
        P->MaxRxQueue         = FIFO_SIZE;
        P->MaxBaud            = SERIAL_BAUD_USER;
        P->ProvSubType        = SERIAL_SP_RS232;
        P->ProvCapabilities   = SERIAL_PCF_RTSCTS | SERIAL_PCF_TOTALTIMEOUTS | SERIAL_PCF_INTTIMEOUTS;
        P->SettableParams     = SERIAL_SP_BAUD | SERIAL_SP_DATABITS | SERIAL_SP_STOPBITS | SERIAL_SP_PARITY |
                                SERIAL_SP_HANDSHAKING;
        P->SettableBaud       = SERIAL_BAUD_USER | SERIAL_BAUD_115200 | SERIAL_BAUD_57600 | SERIAL_BAUD_9600;
        P->SettableData       = SERIAL_DATABITS_5 | SERIAL_DATABITS_6 | SERIAL_DATABITS_7 | SERIAL_DATABITS_8;
        P->SettableStopParity = SERIAL_STOPBITS_10 | SERIAL_STOPBITS_20 | SERIAL_PARITY_NONE | SERIAL_PARITY_ODD |
                                SERIAL_PARITY_EVEN | SERIAL_PARITY_MARK | SERIAL_PARITY_SPACE;
        P->CurrentTxQueue     = FIFO_SIZE;
        P->CurrentRxQueue     = FIFO_SIZE;
      }
      break;
    case IOCTL_SERIAL_GET_COMMSTATUS:
      Status = OutBuffer (Request, sizeof (SERIAL_STATUS), &Buf);
      if (NT_SUCCESS (Status)) {
        ((PSERIAL_STATUS)Buf)->AmountInInQueue  = RxCount (Ctx);
        ((PSERIAL_STATUS)Buf)->AmountInOutQueue = UFSTAT_TXCNT (RD (UFSTAT));
      }
      break;
    case IOCTL_SERIAL_GET_CHARS:
      Status = OutBuffer (Request, sizeof (SERIAL_CHARS), &Buf);
      break;
    case IOCTL_SERIAL_SET_BREAK_ON:
      WR (UCON, RD (UCON) | UCON_SBREAK);
      break;
    case IOCTL_SERIAL_SET_BREAK_OFF:
      break;                             // the send-break bit clears itself
    case IOCTL_SERIAL_SET_CHARS:
    case IOCTL_SERIAL_SET_QUEUE_SIZE:
    case IOCTL_SERIAL_SET_XON:
    case IOCTL_SERIAL_SET_XOFF:
      break;
    default:
      Status = STATUS_NOT_SUPPORTED;
      break;
  }
  if (Ctx->Ioctls++ < 40) {
    OsLog ("exynosuart: ioctl 0x%08lx - 0x%08lx", IoControlCode, Status);
  }
  WdfRequestComplete (Request, Status);
  return Status;
}

//
// ---------------------------------------------------------------------------
// PnP / power
// ---------------------------------------------------------------------------
//

NTSTATUS
EvtPrepareHardware (WDFDEVICE Device, WDFCMRESLIST Raw, WDFCMRESLIST Translated)
{
  PUART_CONTEXT                    Ctx = GetUart (Device);
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Dma[2] = { NULL, NULL };
  ULONG                            DmaCount = 0;
  PHYSICAL_ADDRESS                 Pa;
  ULONG                            Index;

  UNREFERENCED_PARAMETER (Raw);

  for (Index = 0; Index < WdfCmResourceListGetCount (Translated); Index++) {
    Desc = WdfCmResourceListGetDescriptor (Translated, Index);
    if ((Desc->Type == CmResourceTypeMemory) && (Ctx->Regs == NULL)) {
      Ctx->RegsPhys = Desc->u.Memory.Start;
      Ctx->Regs = MmMapIoSpace (Desc->u.Memory.Start, 0x100, MmNonCached);
      OsLog ("exynosuart: registers at 0x%08lx", Desc->u.Memory.Start.LowPart);
    } else if ((Desc->Type == CmResourceTypeDma) && (DmaCount < 2)) {
      Dma[DmaCount++] = Desc;
    }
  }
  Pa.QuadPart = CMU_TOP_PHYS;
  Ctx->Cmu    = MmMapIoSpace (Pa, 0x1000, MmNonCached);
  Pa.QuadPart = GPIO_B_PHYS;
  Ctx->Gpio   = MmMapIoSpace (Pa, 0x1000, MmNonCached);
  if ((Ctx->Regs == NULL) || (Ctx->Cmu == NULL) || (Ctx->Gpio == NULL)) {
    OsLog ("exynosuart: mapping failed");
    return STATUS_INSUFFICIENT_RESOURCES;
  }
  CreateDma (Device, Ctx, Dma[0], Dma[1]);
  return STATUS_SUCCESS;
}

NTSTATUS
EvtReleaseHardware (WDFDEVICE Device, WDFCMRESLIST Translated)
{
  PUART_CONTEXT  Ctx = GetUart (Device);

  UNREFERENCED_PARAMETER (Translated);

  if (Ctx->Regs != NULL) { MmUnmapIoSpace (Ctx->Regs, 0x100); Ctx->Regs = NULL; }
  if (Ctx->Cmu != NULL) { MmUnmapIoSpace (Ctx->Cmu, 0x1000); Ctx->Cmu = NULL; }
  if (Ctx->Gpio != NULL) { MmUnmapIoSpace (Ctx->Gpio, 0x1000); Ctx->Gpio = NULL; }
  return STATUS_SUCCESS;
}

NTSTATUS
EvtD0Entry (WDFDEVICE Device, WDF_POWER_DEVICE_STATE PreviousState)
{
  PUART_CONTEXT  Ctx = GetUart (Device);

  UNREFERENCED_PARAMETER (PreviousState);

  HwInit (Ctx);
  OsLog ("exynosuart: D0, %lu baud, UBRDIV %lu UFRACVAL %lu, CTS %u", Ctx->BaudRate, RD (UBRDIV), RD (UFRACVAL),
         RD (UMSTAT) & UMSTAT_CTS);
  return STATUS_SUCCESS;
}

NTSTATUS
EvtD0Exit (WDFDEVICE Device, WDF_POWER_DEVICE_STATE TargetState)
{
  PUART_CONTEXT  Ctx = GetUart (Device);

  UNREFERENCED_PARAMETER (TargetState);

  WR (UINTM, 0xF);
  OsLog ("exynosuart: D%u (%lu receive errors)", (ULONG)TargetState - WdfPowerDeviceD0, Ctx->Errors);
  return STATUS_SUCCESS;
}

NTSTATUS
EvtDeviceAdd (WDFDRIVER Driver, PWDFDEVICE_INIT DeviceInit)
{
  WDF_PNPPOWER_EVENT_CALLBACKS  PnpPower;
  WDF_OBJECT_ATTRIBUTES         Attributes;
  WDF_INTERRUPT_CONFIG          InterruptConfig;
  SERCX2_CONFIG                 Config;
  SERCX2_PIO_RECEIVE_CONFIG     RxConfig;
  SERCX2_PIO_TRANSMIT_CONFIG    TxConfig;
  WDFDEVICE                     Device;
  PUART_CONTEXT                 Ctx;
  NTSTATUS                      Status;

  UNREFERENCED_PARAMETER (Driver);

  WDF_PNPPOWER_EVENT_CALLBACKS_INIT (&PnpPower);
  PnpPower.EvtDevicePrepareHardware = EvtPrepareHardware;
  PnpPower.EvtDeviceReleaseHardware = EvtReleaseHardware;
  PnpPower.EvtDeviceD0Entry         = EvtD0Entry;
  PnpPower.EvtDeviceD0Exit          = EvtD0Exit;
  WdfDeviceInitSetPnpPowerEventCallbacks (DeviceInit, &PnpPower);

  Status = SerCx2InitializeDeviceInit (DeviceInit);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosuart: SerCx2InitializeDeviceInit - 0x%08lx", Status);
    return Status;
  }

  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, UART_CONTEXT);
  Status = WdfDeviceCreate (&DeviceInit, &Attributes, &Device);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  Ctx = GetUart (Device);
  Ctx->BaudRate               = 115200;
  Ctx->LineControl.WordLength = 8;
  Ctx->LineControl.StopBits   = STOP_BIT_1;
  Ctx->LineControl.Parity     = NO_PARITY;

  WDF_INTERRUPT_CONFIG_INIT (&InterruptConfig, EvtIsr, EvtDpc);
  Status = WdfInterruptCreate (Device, &InterruptConfig, WDF_NO_OBJECT_ATTRIBUTES, &Ctx->Interrupt);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosuart: WdfInterruptCreate - 0x%08lx", Status);
    return Status;
  }

  SERCX2_CONFIG_INIT (&Config, EvtApplyConfig, EvtControl, EvtPurgeFifos);
  Status = SerCx2InitializeDevice (Device, &Config);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosuart: SerCx2InitializeDevice - 0x%08lx", Status);
    return Status;
  }

  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, PIO_CONTEXT);
  SERCX2_PIO_RECEIVE_CONFIG_INIT (&RxConfig, EvtRxRead, EvtRxEnable, EvtRxCancel);
  Status = SerCx2PioReceiveCreate (Device, &RxConfig, &Attributes, &Ctx->PioReceive);
  if (NT_SUCCESS (Status)) {
    GetPio (Ctx->PioReceive)->Device = Device;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, PIO_CONTEXT);
    SERCX2_PIO_TRANSMIT_CONFIG_INIT (&TxConfig, EvtTxWrite, EvtTxEnable, EvtTxCancel);
    Status = SerCx2PioTransmitCreate (Device, &TxConfig, &Attributes, &Ctx->PioTransmit);
  }
  if (NT_SUCCESS (Status)) {
    GetPio (Ctx->PioTransmit)->Device = Device;
  }
  OsLog ("exynosuart: device added - 0x%08lx", Status);
  return Status;
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG  Config;

  OsLogInit ();
  OsLog ("exynosuart: DriverEntry (built " __DATE__ " " __TIME__ ")");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  return WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, WDF_NO_HANDLE);
}
