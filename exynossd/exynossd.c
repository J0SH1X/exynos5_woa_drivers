/** @file
  microSD slot (Exynos5420 dw_mmc channel 2) as a Storport disk for Windows
  RT 8.1 on v1awifi: the SM-P900's card slot as one removable SCSI disk,
  with the MMC side done by the very files UEFI uses (Exynos5420Pkg
  Drivers/SdCardDxe: MmcCore.c, Dwmci.c, built unchanged against MmcEnv.h
  here).  Modelled on exynosemmc.

  Like exynosemmc it registers as a *virtual* miniport: a physical one has
  to claim bus-master DMA and Storport then builds HAL DMA mappings that
  fail for this ACPI device.  The controller's own DMA (IDMAC) moves the
  data through an uncached, physically contiguous bounce buffer that the
  MMC layer allocates; Storport only hands over the mapped DataBuffer.
  HwStartIo (DISPATCH) queues requests; a Storport worker runs them at
  PASSIVE level, polling the controller like UEFI does.  No interrupt.

  The card is optional, so unlike exynosemmc nothing happens until one is
  there:

  - HwFindAdapter maps dw_mmc2 (access range or known address) plus the
    CMU / GPIO pages, configures the card-detect pin and reads it.  With a
    card: power-cycle the slot (SdBoardInit), set up sclk_dwmci2
    (DwmciClockInit), DwmciInit, MmcInitCard (ACMD41), then the speed
    ladder below.  Without: leave the slot powered off.  Either way the
    adapter comes up and reports no media until a card shows up.
  - A 250 ms KTIMER/DPC polls the raw card-detect level (one MMIO read at
    DISPATCH level); when it differs from the confirmed level - or a failed
    init's retry time is up - a Storport worker debounces the pin UEFI
    style (1 ms + 5 reads 2 ms apart) and handles the insert or removal at
    PASSIVE level, then calls StorPortNotification (BusChangeDetected).
  - Speed: init leaves 4-bit at the 20 MHz request (16.65 MHz actual, the
    clock UEFI/kernel use), then 25 MHz (20.8 actual) proven by a read of
    LBA 0, then the SD high-speed switch (CMD6 check + CMD6 write, proven
    from the 64-byte switch status) with 50 MHz (41.6 actual).  Every step
    falls back on the previous proven rate; there is no CLKSEL change
    (0x03040000, CIU = CPLL 666 MHz / CLKSEL 4 = 166.5 MHz) for any SD mode.
  - No media: INQUIRY still answers (RMB = 1), TUR / READ CAPACITY / READ /
    WRITE / VERIFY come back CHECK CONDITION with NOT_READY 0x02 / 0x3A /
    0x00, both in the SRB sense buffer and via SCSIOP_REQUEST_SENSE.
  - A card that stops answering mid-session is brought back like exynosemmc's
    RecoverCard (controller FIFO/DMA reset, 400 kHz 1-bit, MmcInitCard,
    speed ladder again), at most every 2 s; if that fails the media goes
    away and the insert path retries (power cycle) up to 4 times per
    presence state.

  Windows cannot rely on UEFI having touched the slot: UEFI only initialises
  it when a card is present at boot.  The driver therefore does the CMU
  (sclk_dwmci2 = CPLL / 1, gate bit 14), the pins (GPC2[0,1,3-6] func 2,
  drive LV3, CLK LV4; card detect GPX2(4) low = inserted; slot power
  GPY2(0) high = on) and the power itself.  No Windows driver owns those
  pins (GPX2 1/2/5 belong to exynosbt/exynosbtn/exynosaudio, pin 4 is free).

  ACPI: _HID EXYN5017 (SDC0), dw_mmc2 at 0x12220000.

  With LOAD_TEST it bugchecks 0x53444D43 ("SDMC") in HwFindAdapter with
  P1 = register base, P2 = CIU clock (Hz), P3 = access range count.
  Bring-up failures (step STOR_STEP_*, status) are logged and fail the
  adapter.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#define OSLOG_GLOBAL
#include "../../uefi/Exynos5420Pkg/Drivers/SdCardDxe/MmcHost.h"
#include <storport.h>

#ifndef LOAD_TEST
#define LOAD_TEST 0
#endif
#define STOR_BUGCHECK  0x53444D43

enum {
  STOR_STEP_INITIALIZE = 1,
  STOR_STEP_MAP,
  STOR_STEP_MAP_CMU,
  STOR_STEP_MAP_GPIO,
  STOR_STEP_HOST,
  STOR_STEP_WORKER,
};

#define DWMCI2_BASE          0x12220000u
#define DWMCI2_SIZE          0x2000u
#define DWMCI_CIU_HZ         166500000u       // sclk_dwmci2 666 MHz / CLKSEL divider 4
#define DWMCI_SDR_TIMING     0x03040000u      // pdata sdr_timing, as UEFI (CLKSEL, no UHS)
#define MAX_TRANSFER_BLOCKS  256              // 128 KiB per request

//
// CMU page (0x10020000), regs-clock.h offsets: UEFI's CMU_BASE 0x10010000 +
// 0x10xxx.  mClk2 = {mux shift 16, div shift 20, gate bit 14}.
//
#define CMU_PHYS             0x10020000u
#define CMU_SIZE             0x1000u
#define CLK_CPLL_CON0        0x120
#define CLK_DPLL_CON0        0x128
#define CLK_EPLL_CON0        0x130
#define CLK_RPLL_CON0        0x140
#define CLK_IPLL_CON0        0x150
#define CLK_SPLL_CON0        0x160
#define CLK_MPLL_CON0        0x180
#define CLK_SRC_FSYS         0x244
#define CLK_SRC_MASK_FSYS    0x340
#define CLK_DIV_FSYS1        0x54C
#define CLK_DIV_STAT_FSYS1   0x64C
#define CLK_GATE_IP_FSYS     0x944
#define XXTI_HZ              24000000u
#define MUX_SRC_CPLL         1
#define SD_MUX_SHIFT         16
#define SD_DIV_SHIFT         20
#define SD_GATE_BIT          14
#define CLKSEL_CIU_DIV(x)    ((((x) >> 24) & 0x7) + 1)

//
// GPIO: controller "T" (0x13410000) has GPC2 and GPY2, controller "RT"
// (0x13400000) has the GPX2 card-detect bank.  Every bank is a CON/DAT/PUD
// triplet (0x00/0x04/0x08), 4 bits per pin in CON, 2 bits per pin in PUD.
//
#define GPIO_T_PHYS          0x13410000u
#define GPIO_T_SIZE          0x200u
#define GPC2_CON             0x040
#define GPC2_PUD             0x048
#define GPC2_DRV             0x04C           // bank + 0x0C, 2 bits per pin
#define GPY2_CON             0x100
#define GPY2_DAT             0x104
#define GPY2_PUD             0x108

#define GPIO_RT_PHYS         0x13400000u
#define GPIO_RT_SIZE         0x1000u
#define GPX2_CON             0xC40
#define GPX2_DAT             0xC44
#define GPX2_PUD             0xC48
#define SD_CD_PIN            4               // GPX2(4), low = card inserted
#define SD_POWER_PIN         0               // GPY2(0) = GPIO_TF_EN, high = on

#define GPIO_FUNC_INPUT       0
#define GPIO_FUNC_OUTPUT      1
#define GPIO_FUNC_SD          2               // GPC2 special function for dw_mmc2
#define GPIO_PULL_NONE        0
#define DRV_LV3              1
#define DRV_LV4              3               // CLK pin drive

//
// Card detect polling and init retry budget.
//
#define CD_POLL_MS           250
#define INIT_RETRY_MS        2000
#define MAX_INIT_TRIES       4                // initial + 3 retries per presence state

//
// SD switch (CMD6): group 0 = access mode, value 1 = high speed; the switch
// status reports support in byte 13 (SD_MODE_HIGH_SPEED = 1 << 1) and the
// mode actually set in byte 16 (low nibble 1 = high speed), as in the
// kernel's mmc_sd_switch / mmc_sd_switch_hs.
//
#define SD_SWITCH_FUNC       6
#define SD_MODE_HIGH_SPEED   0x02
#define SD_R1_ERROR_MASK     0xFDF90008U      // as MmcCore.c
#define SD_R1_SWITCH_ERROR   (1U << 18)       // SD R1 bit for CMD6 switch errors

typedef struct {
  LIST_ENTRY           Link;
  PSCSI_REQUEST_BLOCK  Srb;
} SRB_EXT, *PSRB_EXT;

typedef struct {
  PUCHAR      Base;             // dw_mmc2 registers
  ULONG       Size;
  PUCHAR      Cmu;              // CMU page (clocks for sclk_dwmci2)
  ULONG       CmuSize;
  PUCHAR      GpioT;            // GPC2 pins / GPY2 slot power
  ULONG       GpioTSize;
  PUCHAR      GpioRt;           // GPX2 card detect
  ULONG       GpioRtSize;
  MMC_HOST    Host;
  BOOLEAN     HostInited;       // DwmciInit ran once (its DMA pool is not re-allocatable)
  PVOID       Worker;
  KSPIN_LOCK  Lock;
  LIST_ENTRY  Pending;
  BOOLEAN     WorkerQueued;     // one work item covers the SRB queue and card-detect work
  BOOLEAN     CdPending;        // card-detect work waiting for the worker
  BOOLEAN     CdConfirmed;      // debounced level: TRUE = card in the slot
  BOOLEAN     MediaPresent;     // card identified, usable
  BOOLEAN     HwDone;           // HwFindAdapter finished (BusChangeDetected is safe)
  LONGLONG    NextRetry;        // KeQueryInterruptTime() of the next init attempt, 0 = none
  ULONG       InitTries;        // init attempts in the current presence state
  ULONG       Commands;         // SCSI commands seen (logging)
  ULONG       Transfers;        // reads/writes done (logging)
  ULONG       Recoveries;       // card re-initialisations after a dropout
  ULONG       Inserts;
  ULONG       Removals;
  LONGLONG    LastRecovery;     // KeQueryInterruptTime() of the last one
  KTIMER      CdTimer;
  KDPC        CdDpc;
} DEVICE_EXTENSION, *PDEVICE_EXTENSION;

//
// Defined below; the card-detect DPC hands work to it.
//
static VOID Worker (PVOID DeviceExtension, PVOID Context, PVOID WorkerHandle);

static VOID
BringUpFailed (ULONG Step, ULONG_PTR Status)
{
  OsLog ("exynossd: bring-up step %lu failed: 0x%08Ix", Step, Status);
}

//
// ---------------------------------------------------------------------------
// MMIO and board setup (pins, slot power, clock), at PASSIVE level.
// ---------------------------------------------------------------------------
//

static ULONG
RegRd (PUCHAR Page, ULONG Offset)
{
  return READ_REGISTER_ULONG ((PULONG)(Page + Offset));
}

static VOID
RegWr (PUCHAR Page, ULONG Offset, ULONG Value)
{
  WRITE_REGISTER_ULONG ((PULONG)(Page + Offset), Value);
}

static VOID
RegOr (PUCHAR Page, ULONG Offset, ULONG Value)
{
  RegWr (Page, Offset, RegRd (Page, Offset) | Value);
}

//
// 4-bit CON / 2-bit PUD fields, one call per pin (UEFI's Gpio->Config).
//
static VOID
GpioCon (PUCHAR Page, ULONG Offset, ULONG Pin, ULONG Function)
{
  ULONG Value = RegRd (Page, Offset);

  Value &= ~(0xFU << (Pin * 4));
  Value |= (Function & 0xF) << (Pin * 4);
  RegWr (Page, Offset, Value);
}

static VOID
GpioPud (PUCHAR Page, ULONG Offset, ULONG Pin, ULONG Pull)
{
  ULONG Value = RegRd (Page, Offset);

  Value &= ~(0x3U << (Pin * 2));
  Value |= (Pull & 0x3) << (Pin * 2);
  RegWr (Page, Offset, Value);
}

static VOID
GpioDrv (PUCHAR Page, ULONG Offset, ULONG Pin, ULONG Level)
{
  ULONG Value = RegRd (Page, Offset);

  Value &= ~(0x3U << (Pin * 2));
  Value |= (Level & 0x3) << (Pin * 2);
  RegWr (Page, Offset, Value);
}

static VOID
GpioSet (PUCHAR Page, ULONG Offset, ULONG Pin, BOOLEAN High)
{
  ULONG Value = RegRd (Page, Offset);

  if (High) {
    Value |= 1U << Pin;
  } else {
    Value &= ~(1U << Pin);
  }
  RegWr (Page, Offset, Value);
}

/**
  Route the slot pins (exynos_dwmci2_cfg_gpio) or put them back to inputs so
  an unpowered card is not back-fed through CMD/CLK/DAT.  Pin 2 is skipped
  like UEFI does (the kernel routes it, UEFI does not).
**/
static VOID
SdPins (PDEVICE_EXTENSION Dev, BOOLEAN Func)
{
  STATIC CONST ULONG  Pins[] = { 0, 1, 3, 4, 5, 6 };
  UINTN               Index;

  for (Index = 0; Index < ARRAY_SIZE (Pins); Index++) {
    GpioCon (Dev->GpioT, GPC2_CON, Pins[Index], Func ? GPIO_FUNC_SD : GPIO_FUNC_INPUT);
    GpioPud (Dev->GpioT, GPC2_PUD, Pins[Index], GPIO_PULL_NONE);
    if (Func) {
      GpioDrv (Dev->GpioT, GPC2_DRV, Pins[Index], (Pins[Index] == 0) ? DRV_LV4 : DRV_LV3);
    }
  }
}

static VOID
SdPower (PDEVICE_EXTENSION Dev, BOOLEAN On)
{
  GpioCon (Dev->GpioT, GPY2_CON, SD_POWER_PIN, GPIO_FUNC_OUTPUT);
  GpioPud (Dev->GpioT, GPY2_PUD, SD_POWER_PIN, GPIO_PULL_NONE);
  GpioSet (Dev->GpioT, GPY2_DAT, SD_POWER_PIN, On);
}

/**
  Slot off and pins tri-stated: the state for an empty slot and the first
  half of every power cycle.
**/
static VOID
SdPowerDown (PDEVICE_EXTENSION Dev)
{
  SdPins (Dev, FALSE);
  SdPower (Dev, FALSE);
}

/**
  Power-cycle the slot (exynos_dwmci2_cfg_gpio(4) + exynos_dwmci2_setpower),
  exactly as UEFI's SdBoardInit: pins are inputs while the card is unpowered,
  50 ms off, then routed with drive strength, on, 20 ms for the rail and the
  card's power-up.
**/
static VOID
SdBoardInit (PDEVICE_EXTENSION Dev)
{
  SdPins (Dev, FALSE);
  SdPower (Dev, FALSE);
  KeStallExecutionProcessor (50000);

  SdPins (Dev, TRUE);
  SdPower (Dev, TRUE);
  KeStallExecutionProcessor (20000);
}

//
// ---------------------------------------------------------------------------
// Clocks: sclk_dwmci2 = CPLL / 1 (UEFI's DwmciClockInit for mClk2).
// ---------------------------------------------------------------------------
//

/**
  Output of a PLL from its CON0: FOUT = MDIV * FIN / (PDIV << SDIV), 0 if the
  PLL is off (pll_35xx, as UEFI's PllRate).
**/
static UINT32
PllRate (PDEVICE_EXTENSION Dev, ULONG Offset)
{
  ULONG  Reg;
  ULONG  Mdiv;
  ULONG  Pdiv;
  ULONG  Sdiv;

  Reg = RegRd (Dev->Cmu, Offset);
  if ((Reg & (1U << 31)) == 0) {
    return 0;
  }
  Mdiv = (Reg >> 16) & 0x3FF;
  Pdiv = (Reg >> 8) & 0x3F;
  Sdiv = Reg & 0x7;
  if (Pdiv == 0) {
    return 0;
  }
  return (UINT32)(((UINT64)XXTI_HZ * Mdiv) / (Pdiv << Sdiv));
}

static UINT32
MuxSourceRate (PDEVICE_EXTENSION Dev, UINT32 Source)
{
  STATIC CONST ULONG  Pll[] = {
    0, CLK_CPLL_CON0, CLK_DPLL_CON0, CLK_MPLL_CON0,
    CLK_SPLL_CON0, CLK_IPLL_CON0, CLK_EPLL_CON0, CLK_RPLL_CON0
  };

  if (Source == 0) {
    return XXTI_HZ;
  }
  return (Source < ARRAY_SIZE (Pll)) ? PllRate (Dev, Pll[Source]) : 0;
}

/**
  Gate sclk_dwmci2, feed it from CPLL / 1 like the kernel (or keep the
  bootloader's source if CPLL is off) and return the CIU clock after the
  CLKSEL divider - DwmciClockInit (&mClk2, DWMCI_SDR_TIMING) from UEFI, with
  a wait for the divider change to land (as the kernel's exynos set_clksrc).
**/
static UINT32
SdClockInit (PDEVICE_EXTENSION Dev)
{
  UINT32  Source;
  UINT32  Div;
  UINT32  Rate;
  UINTN   Retry;

  RegOr (Dev->Cmu, CLK_GATE_IP_FSYS, 1U << SD_GATE_BIT);
  RegOr (Dev->Cmu, CLK_SRC_MASK_FSYS, 1U << SD_MUX_SHIFT);

  if (PllRate (Dev, CLK_CPLL_CON0) != 0) {
    ULONG  Reg;

    Reg = RegRd (Dev->Cmu, CLK_SRC_FSYS);
    Reg &= ~(0x7U << SD_MUX_SHIFT);
    Reg |= MUX_SRC_CPLL << SD_MUX_SHIFT;
    RegWr (Dev->Cmu, CLK_SRC_FSYS, Reg);
    RegWr (Dev->Cmu, CLK_DIV_FSYS1,
           RegRd (Dev->Cmu, CLK_DIV_FSYS1) & ~(0x3FFU << SD_DIV_SHIFT));

    for (Retry = 0; Retry < 1000; Retry++) {
      if ((RegRd (Dev->Cmu, CLK_DIV_STAT_FSYS1) & (1U << SD_DIV_SHIFT)) == 0) {
        break;
      }
      KeStallExecutionProcessor (10);
    }
  }

  Source = (RegRd (Dev->Cmu, CLK_SRC_FSYS) >> SD_MUX_SHIFT) & 0x7;
  Div    = ((RegRd (Dev->Cmu, CLK_DIV_FSYS1) >> SD_DIV_SHIFT) & 0x3FF) + 1;
  Rate   = MuxSourceRate (Dev, Source) / Div;
  return Rate / CLKSEL_CIU_DIV (DWMCI_SDR_TIMING);
}

//
// ---------------------------------------------------------------------------
// Card detect (GPX2(4), low = inserted).
// ---------------------------------------------------------------------------
//

static BOOLEAN
CdRaw (PDEVICE_EXTENSION Dev)
{
  return (RegRd (Dev->GpioRt, GPX2_DAT) & (1U << SD_CD_PIN)) == 0;
}

/**
  UEFI's SdCardPresent: the pin is a plain input (the board pulls it up);
  it counts as inserted when it stays low on 5 reads 2 ms apart after a
  1 ms settle.
**/
static BOOLEAN
SdCardPresent (PDEVICE_EXTENSION Dev)
{
  UINTN  Read;

  KeStallExecutionProcessor (1000);
  for (Read = 0; Read < 5; Read++) {
    if (!CdRaw (Dev)) {
      return FALSE;
    }
    KeStallExecutionProcessor (2000);
  }
  return TRUE;
}

//
// ---------------------------------------------------------------------------
// Card bring-up: init and the speed ladder (PASSIVE, in the worker or
// HwFindAdapter).
// ---------------------------------------------------------------------------
//

/**
  Actual card clock: CIU / (2 * ceil(CIU / (2 * Requested))), the divider
  DwmciSetClock programs (20 MHz -> 16.65, 25 -> 20.8, 50 -> 41.6).
**/
static UINT32
ActualClock (UINT32 Requested)
{
  UINT32  Div;

  if (Requested >= DWMCI_CIU_HZ) {
    return DWMCI_CIU_HZ;
  }
  Div = (DWMCI_CIU_HZ + 2 * Requested - 1) / (2 * Requested);
  if (Div == 0) {
    Div = 1;
  }
  if (Div > 0xFF) {
    Div = 0xFF;
  }
  return DWMCI_CIU_HZ / (2 * Div);
}

/**
  SD switch (CMD6): one 64-byte data read whose argument picks mode
  (0 = check, 1 = write), group and value - the kernel's mmc_sd_switch.
  BlockSize/ByteCount are the MMC_HOST overrides for the non-512-byte phase
  and are cleared again whatever happens.
**/
static EFI_STATUS
SdSwitch (PDEVICE_EXTENSION Dev, UINT32 Mode, UINT32 Group, UINT32 Value, UINT8 *Status)
{
  EFI_STATUS  Result;
  UINT32      Resp[4];
  UINT32      Arg;

  Arg = ((Mode != 0) ? 0x80000000U : 0) | 0x00FFFFFFU;
  Arg &= ~(0xFU << (Group * 4));
  Arg |= (Value & 0xF) << (Group * 4);

  Resp[0] = 0;
  Dev->Host.BlockSize = 64;
  Dev->Host.ByteCount = 64;

  Result = Dev->Host.PrepareData (&Dev->Host, TRUE, 1, Status);
  if (!EFI_ERROR (Result)) {
    Result = Dev->Host.SendCmd (&Dev->Host, SD_SWITCH_FUNC, Arg,
                                HOST_CMD_RESP_SHORT | HOST_CMD_DATA, Resp);
  }
  if (!EFI_ERROR (Result)) {
    Result = Dev->Host.XferBlock (&Dev->Host, TRUE, 1, Status);
  }

  Dev->Host.BlockSize = 0;
  Dev->Host.ByteCount = 0;

  if (!EFI_ERROR (Result) && ((Resp[0] & (SD_R1_ERROR_MASK | SD_R1_SWITCH_ERROR)) != 0)) {
    OsLog ("exynossd: CMD6 mode %lu group %lu: R1 0x%08lx", Mode, Group, Resp[0]);
    Result = EFI_DEVICE_ERROR;
  }
  return Result;
}

/**
  Prove a clock or mode by reading LBA 0, like the MMC core does for every
  bus mode change.
**/
static BOOLEAN
ProveRead (PDEVICE_EXTENSION Dev)
{
  STATIC UINT8  Block[BLOCK_SIZE];
  EFI_STATUS    Status;

  Status = MmcReadBlocks (&Dev->Host, 0, 1, Block);
  if (EFI_ERROR (Status)) {
    OsLog ("exynossd: proof read of LBA 0 failed - 0x%08Ix", Status);
    return FALSE;
  }
  return TRUE;
}

/**
  Speed ladder after MmcInitCard (which left 4-bit at the 20 MHz request,
  16.65 MHz actual): 25 MHz proven by a read, then the SD high-speed switch
  (CMD6 check, CMD6 write, both verified from the switch status) with a
  50 MHz request (41.6 MHz actual), each step proven by a read.  Every
  failure falls back to the last proven rate; a failed fallback re-runs
  MmcInitCard, which restores 4-bit / 20 MHz through SdTuneBus.  CLKSEL
  stays 0x03040000: no SD mode needs a different CIU phase or divider.
**/
static VOID
SelectSpeed (PDEVICE_EXTENSION Dev)
{
  EFI_STATUS  Status;
  UINT8       Switch[64];
  UINT32      Clock;

  //
  // 25 MHz (20.8 actual): the fastest clock guaranteed to work after init.
  //
  Status = Dev->Host.SetClock (&Dev->Host, 25000000);
  if (EFI_ERROR (Status) || !ProveRead (Dev)) {
    //
    // Back to the 20 MHz request MmcInitCard proved (MMC_TRANSFER_CLOCK_HZ
    // in MmcCore.c, 16.65 MHz actual).
    //
    Status = Dev->Host.SetClock (&Dev->Host, 20000000);
    if (EFI_ERROR (Status) || !ProveRead (Dev)) {
      OsLog ("exynossd: even the init clock failed - 0x%08Ix, re-initialising", Status);
      Status = MmcInitCard (&Dev->Host, FALSE);
      OsLog ("exynossd: re-init after clock failure - 0x%08Ix, %lu-bit, %lu Hz", Status,
             Dev->Host.Card.BusWidth, Dev->Host.ClockHz);
      return;
    }
    OsLog ("exynossd: staying at the 20 MHz request (%lu Hz actual)", ActualClock (20000000));
    return;
  }

  //
  // SD high-speed (CMD6 group 0, value 1): check the card supports it,
  // then ask it to switch, and verify from the 64-byte switch status that
  // mode 1 (bits 13 and 16) is what it did.
  //
  if (!EFI_ERROR (SdSwitch (Dev, 0, 0, 1, Switch)) && ((Switch[13] & SD_MODE_HIGH_SPEED) != 0)) {
    if (!EFI_ERROR (SdSwitch (Dev, 1, 0, 1, Switch)) && ((Switch[16] & 0xF) == 1)) {
      Status = Dev->Host.SetClock (&Dev->Host, 50000000);
      if (!EFI_ERROR (Status) && ProveRead (Dev)) {
        Clock  = 50000000;
        goto Logged;
      }
      OsLog ("exynossd: 50 MHz request failed - 0x%08Ix, back to 25 MHz", Status);
      Status = Dev->Host.SetClock (&Dev->Host, 25000000);
      if (EFI_ERROR (Status) || !ProveRead (Dev)) {
        OsLog ("exynossd: 25 MHz no longer proven - 0x%08Ix, re-initialising", Status);
        Status = MmcInitCard (&Dev->Host, FALSE);
        OsLog ("exynossd: re-init after switch failure - 0x%08Ix, %lu-bit, %lu Hz", Status,
               Dev->Host.Card.BusWidth, Dev->Host.ClockHz);
        return;
      }
      Clock  = 25000000;
      goto Logged;
    }
    OsLog ("exynossd: card refused the high-speed switch, staying at 25 MHz");
  } else {
    OsLog ("exynossd: card reports no high-speed support, staying at 25 MHz");
  }

  Clock = 25000000;

Logged:
  OsLog ("exynossd: speed = %lu MHz request (%lu Hz actual), %lu-bit, card %I64u blocks",
         Clock / 1000000, ActualClock (Clock), Dev->Host.Card.BusWidth,
         Dev->Host.Card.BlockCount);
}

/**
  The card stopped answering mid-session (every command times out: it has
  lost its RCA, as after a reset).  Bring it back like exynosemmc's
  RecoverCard: controller FIFO / DMA reset, 400 kHz 1-bit, the whole
  identification sequence, then the speed ladder.  At most every 2 s; when
  it fails the media goes away and the insert path retries from scratch.
**/
static BOOLEAN
RecoverCard (PDEVICE_EXTENSION Dev)
{
  LONGLONG    Now;
  EFI_STATUS  Status;

  Now = (LONGLONG)KeQueryInterruptTime ();
  if ((Dev->LastRecovery != 0) && (Now - Dev->LastRecovery < 20000000LL)) {
    return FALSE;
  }
  Dev->LastRecovery = Now;
  Dev->Recoveries++;

  WRITE_REGISTER_ULONG ((PULONG)(Dev->Base + 0x00), READ_REGISTER_ULONG ((PULONG)(Dev->Base + 0x00)) | 0x6);
  KeStallExecutionProcessor (100);
  Dev->Host.SetBusWidth (&Dev->Host, 1);
  Dev->Host.SetClock (&Dev->Host, 400000);
  Status = MmcInitCard (&Dev->Host, FALSE);
  OsLog ("exynossd: card stopped answering, re-init #%lu - 0x%08Ix (%lu-bit)", Dev->Recoveries,
         Status, Dev->Host.Card.BusWidth);
  if (!EFI_ERROR (Status)) {
    SelectSpeed (Dev);
    Dev->MediaPresent = TRUE;
    Dev->InitTries    = 0;
    Dev->NextRetry    = 0;
    return TRUE;
  }

  Dev->MediaPresent = FALSE;
  Dev->InitTries    = 0;                     // fresh budget: the worker re-inits from scratch
  Dev->NextRetry    = Now + (LONGLONG)INIT_RETRY_MS * 10000LL;
  return FALSE;
}

/**
  One insert attempt: power-cycle the slot, make sure sclk_dwmci2 runs,
  reset / arm the controller (DwmciInit once - it allocates the DMA pool
  and must not run twice - otherwise just clear whatever the last transfer
  left), identify the card (ACMD41) and pick the speed.

  On failure the slot is powered down and the attempt counts against the
  budget (MAX_INIT_TRIES per presence state); NextRetry schedules the next
  one through the card-detect DPC.
**/
static BOOLEAN
SdTryInit (PDEVICE_EXTENSION Dev)
{
  EFI_STATUS  Status;
  UINT32      CiuHz;
  BOOLEAN     First;

  First = !Dev->HostInited;
  Dev->InitTries++;
  Dev->NextRetry = (LONGLONG)KeQueryInterruptTime () + (LONGLONG)INIT_RETRY_MS * 10000LL;

  SdBoardInit (Dev);
  CiuHz = SdClockInit (Dev);
  if (CiuHz == 0) {
    OsLog ("exynossd: sclk_dwmci2 is off (mux source?), attempt %lu", Dev->InitTries);
    SdPowerDown (Dev);
    goto Budget;
  }

  if (First) {
    Status = DwmciInit (&Dev->Host, (UINT32)(ULONG_PTR)Dev->Base, CiuHz, 4,
                        DWMCI_SDR_TIMING, FALSE);
    if (EFI_ERROR (Status)) {
      BringUpFailed (STOR_STEP_HOST, Status);
      SdPowerDown (Dev);
      goto Budget;
    }
    Dev->HostInited = TRUE;
  } else {
    WRITE_REGISTER_ULONG ((PULONG)(Dev->Base + 0x00), READ_REGISTER_ULONG ((PULONG)(Dev->Base + 0x00)) | 0x6);
    KeStallExecutionProcessor (100);
    Dev->Host.SetBusWidth (&Dev->Host, 1);
    Dev->Host.SetClock (&Dev->Host, 400000);
  }

  Status = MmcInitCard (&Dev->Host, FALSE);
  if (EFI_ERROR (Status)) {
    OsLog ("exynossd: card init attempt %lu failed - 0x%08Ix", Dev->InitTries, Status);
    SdPowerDown (Dev);
    goto Budget;
  }

  SelectSpeed (Dev);
  Dev->MediaPresent = TRUE;
  Dev->NextRetry    = 0;
  OsLog ("exynossd: microSD ready, %I64u blocks (%I64u MiB), %lu-bit, %lu Hz",
         Dev->Host.Card.BlockCount, Dev->Host.Card.BlockCount >> 11,
         Dev->Host.Card.BusWidth, Dev->Host.ClockHz);
  return TRUE;

Budget:
  if (Dev->InitTries >= MAX_INIT_TRIES) {
    OsLog ("exynossd: %lu init attempts failed, giving up until the card is re-inserted",
           Dev->InitTries);
    Dev->NextRetry = 0;
  }
  return FALSE;
}

/**
  Card-detect work, at PASSIVE level in the worker: debounce the pin, then
  act on insert / removal / retry.  BusChangeDetected (PathId 0) makes
  storport re-enumerate the bus with INQUIRY, which is how Windows finds
  the disk arriving or going away.
**/
static VOID
HandleCardDetect (PDEVICE_EXTENSION Dev)
{
  BOOLEAN  Present;

  Present = SdCardPresent (Dev);
  Dev->CdConfirmed = Present;

  if (Present) {
    if (!Dev->MediaPresent) {
      BOOLEAN  Added;

      if (Dev->InitTries >= MAX_INIT_TRIES) {
        return;                             // budget exhausted, wait for a re-insert
      }
      Added = SdTryInit (Dev);
      if (Added) {
        Dev->Inserts++;
        OsLog ("exynossd: card present (insertion #%lu)", Dev->Inserts);
        if (Dev->HwDone) {
          StorPortNotification (BusChangeDetected, Dev, 0);   // PathId 0
        }
      }
    }
    return;
  }

  if (Dev->MediaPresent) {
    Dev->MediaPresent = FALSE;
    SdPowerDown (Dev);
    Dev->Removals++;
    OsLog ("exynossd: card removed (removal #%lu)", Dev->Removals);
    if (Dev->HwDone) {
      StorPortNotification (BusChangeDetected, Dev, 0);       // PathId 0
    }
  } else {
    SdPowerDown (Dev);                      // empty slot: keep the rail off
  }
  Dev->InitTries = 0;                       // a future insert starts a fresh budget
  Dev->NextRetry = 0;
}

/**
  Card-detect timer callback (DISPATCH): one MMIO read of GPX2(4).  A
  changed level, or an init retry whose time has come, hands the work to
  the Storport worker (debounce + init + notifications all run there, at
  PASSIVE level).
**/
static VOID
CdTimerDpc (PKDPC Dpc, PVOID DeferredContext, PVOID SystemArgument1, PVOID SystemArgument2)
{
  PDEVICE_EXTENSION  Dev;
  BOOLEAN            Raw;
  BOOLEAN            Need;
  BOOLEAN            Kick;
  KIRQL              Irql;

  UNREFERENCED_PARAMETER (Dpc);
  UNREFERENCED_PARAMETER (SystemArgument1);
  UNREFERENCED_PARAMETER (SystemArgument2);

  Dev  = DeferredContext;
  Raw  = CdRaw (Dev);
  Need = (Raw != Dev->CdConfirmed);
  if (!Need && Raw && !Dev->MediaPresent && (Dev->NextRetry != 0) &&
      ((LONGLONG)KeQueryInterruptTime () >= Dev->NextRetry)) {
    Need = TRUE;                            // failed init, retry time reached
  }
  if (!Need) {
    return;
  }

  KeAcquireSpinLock (&Dev->Lock, &Irql);
  Dev->CdPending = TRUE;
  Kick = !Dev->WorkerQueued;
  Dev->WorkerQueued = TRUE;
  KeReleaseSpinLock (&Dev->Lock, Irql);

  if (Kick && (StorPortQueueWorkItem (Dev, Worker, Dev->Worker, NULL) != STOR_STATUS_SUCCESS)) {
    OsLog ("exynossd: card-detect work item unavailable (worker %p), leaving it queued", Dev->Worker);
    KeAcquireSpinLock (&Dev->Lock, &Irql);
    Dev->WorkerQueued = FALSE;
    KeReleaseSpinLock (&Dev->Lock, Irql);
  }
}

//
// ---------------------------------------------------------------------------
// SCSI commands, at PASSIVE level in the worker.
// ---------------------------------------------------------------------------
//

static UCHAR
CopyOut (PSCSI_REQUEST_BLOCK Srb, const VOID *Data, ULONG Length)
{
  Length = min (Length, Srb->DataTransferLength);
  if ((Length != 0) && (Srb->DataBuffer == NULL)) {
    return SRB_STATUS_ERROR;
  }
  RtlCopyMemory (Srb->DataBuffer, Data, Length);
  Srb->DataTransferLength = Length;
  return SRB_STATUS_SUCCESS;
}

/**
  Fixed-format sense for a failed command.  Filled into the SRB's own sense
  buffer (AutoRequestSense) so the class driver sees it whether or not it
  follows up with SCSIOP_REQUEST_SENSE - that one is handled below too.
**/
static UCHAR
FillSense (PSCSI_REQUEST_BLOCK Srb, UCHAR Key, UCHAR Asc, UCHAR Ascq)
{
  SENSE_DATA  Sense;
  ULONG       Length;

  RtlZeroMemory (&Sense, sizeof (Sense));
  Sense.ErrorCode                    = SCSI_SENSE_ERRORCODE_FIXED_CURRENT;
  Sense.SenseKey                     = Key;
  Sense.AdditionalSenseLength        = sizeof (Sense) - 8;
  Sense.AdditionalSenseCode          = Asc;
  Sense.AdditionalSenseCodeQualifier = Ascq;
  Srb->DataTransferLength            = 0;

  if ((Srb->SenseInfoBuffer == NULL) || (Srb->SenseInfoBufferLength == 0)) {
    return SRB_STATUS_ERROR;
  }
  Length = min ((ULONG)Srb->SenseInfoBufferLength, (ULONG)sizeof (SENSE_DATA));
  RtlCopyMemory (Srb->SenseInfoBuffer, &Sense, Length);
  return SRB_STATUS_ERROR | SRB_STATUS_AUTOSENSE_VALID;
}

/**
  No card: NOT_READY / "medium not present" (0x02 / 0x3A / 0x00).
**/
static UCHAR
NoMedia (PSCSI_REQUEST_BLOCK Srb)
{
  return FillSense (Srb, 0x02, 0x3A, 0x00);
}

static UCHAR
ScsiInquiry (PDEVICE_EXTENSION Dev, PSCSI_REQUEST_BLOCK Srb)
{
  INQUIRYDATA  Inquiry;
  UCHAR        Pages[6];

  UNREFERENCED_PARAMETER (Dev);

  if ((Srb->Cdb[1] & 1) != 0) {
    // EVPD: only the supported-pages page.
    if (Srb->Cdb[2] != 0) {
      return SRB_STATUS_INVALID_REQUEST;
    }
    RtlZeroMemory (Pages, sizeof (Pages));
    Pages[3] = 1;                    // one page: 0x00
    return CopyOut (Srb, Pages, 5);
  }

  RtlZeroMemory (&Inquiry, sizeof (Inquiry));
  Inquiry.DeviceType          = DIRECT_ACCESS_DEVICE;
  Inquiry.RemovableMedia      = 1;
  Inquiry.Versions            = 5;
  Inquiry.ResponseDataFormat  = 2;
  Inquiry.AdditionalLength    = sizeof (Inquiry) - 5;
  RtlCopyMemory (Inquiry.VendorId, "Samsung ", 8);
  RtlCopyMemory (Inquiry.ProductId, "SM-P900 microSD ", 16);
  RtlCopyMemory (Inquiry.ProductRevisionLevel, "1.0 ", 4);
  return CopyOut (Srb, &Inquiry, sizeof (Inquiry));
}

static UCHAR
ScsiReadCapacity (PDEVICE_EXTENSION Dev, PSCSI_REQUEST_BLOCK Srb, BOOLEAN Long)
{
  UCHAR   Data[32];
  UINT64  Last;
  ULONG   Index;

  Last = Dev->Host.Card.BlockCount - 1;
  RtlZeroMemory (Data, sizeof (Data));
  if (Long) {
    for (Index = 0; Index < 8; Index++) {
      Data[Index] = (UCHAR)(Last >> (56 - 8 * Index));
    }
    Data[10] = BLOCK_SIZE >> 8;        // bytes 8-11: block length, big endian
    return CopyOut (Srb, Data, 32);
  }

  if (Last > 0xFFFFFFFF) {
    Last = 0xFFFFFFFF;                 // tells the host to use READ CAPACITY 16
  }
  Data[0] = (UCHAR)(Last >> 24);
  Data[1] = (UCHAR)(Last >> 16);
  Data[2] = (UCHAR)(Last >> 8);
  Data[3] = (UCHAR)Last;
  Data[6] = BLOCK_SIZE >> 8;
  return CopyOut (Srb, Data, 8);
}

/**
  Mode parameter header only (no pages): not write-protected, no cache page.
**/
static UCHAR
ScsiModeSense (PSCSI_REQUEST_BLOCK Srb, BOOLEAN Ten)
{
  UCHAR  Header[8];

  RtlZeroMemory (Header, sizeof (Header));
  if (Ten) {
    Header[1] = 6;                     // mode data length after this field
    return CopyOut (Srb, Header, 8);
  }
  Header[0] = 3;
  return CopyOut (Srb, Header, 4);
}

/**
  REQUEST_SENSE: what a follow-up after CHECK CONDITION would get - the
  current medium state (no card: NOT_READY 0x02 / 0x3A / 0x00, else no
  sense).
**/
static UCHAR
ScsiRequestSense (PDEVICE_EXTENSION Dev, PSCSI_REQUEST_BLOCK Srb)
{
  SENSE_DATA  Sense;

  RtlZeroMemory (&Sense, sizeof (Sense));
  Sense.ErrorCode             = SCSI_SENSE_ERRORCODE_FIXED_CURRENT;
  Sense.AdditionalSenseLength = sizeof (Sense) - 8;
  if (!Dev->MediaPresent) {
    Sense.SenseKey                    = 0x02;
    Sense.AdditionalSenseCode         = 0x3A;
    Sense.AdditionalSenseCodeQualifier = 0x00;
  }
  return CopyOut (Srb, &Sense, sizeof (Sense));
}

static UCHAR
ScsiReadWrite (PDEVICE_EXTENSION Dev, PSCSI_REQUEST_BLOCK Srb, BOOLEAN Write, UINT64 Lba, ULONG Blocks)
{
  EFI_STATUS  Status;

  if (Blocks == 0) {
    return SRB_STATUS_SUCCESS;
  }
  if ((Lba + Blocks > Dev->Host.Card.BlockCount) ||
      (Srb->DataTransferLength < Blocks * BLOCK_SIZE) || (Srb->DataBuffer == NULL)) {
    return SRB_STATUS_INVALID_REQUEST;
  }

  Status = Write ? MmcWriteBlocks (&Dev->Host, Lba, Blocks, Srb->DataBuffer)
                 : MmcReadBlocks (&Dev->Host, Lba, Blocks, Srb->DataBuffer);
  if ((Status == EFI_TIMEOUT) && RecoverCard (Dev)) {
    Status = Write ? MmcWriteBlocks (&Dev->Host, Lba, Blocks, Srb->DataBuffer)
                   : MmcReadBlocks (&Dev->Host, Lba, Blocks, Srb->DataBuffer);
    OsLog ("exynossd: retry after re-init: %s LBA %I64u x%lu - 0x%08Ix", Write ? "write" : "read", Lba, Blocks, Status);
  }
  if (EFI_ERROR (Status)) {
    OsLog ("exynossd: %s LBA %I64u x%lu failed 0x%08Ix", Write ? "write" : "read", Lba, Blocks, Status);
    return Dev->MediaPresent ? SRB_STATUS_ERROR : NoMedia (Srb);
  }
  Srb->DataTransferLength = Blocks * BLOCK_SIZE;
  return SRB_STATUS_SUCCESS;
}

static UINT64
Be (const UCHAR *P, ULONG Bytes)
{
  UINT64  Value;

  for (Value = 0; Bytes > 0; Bytes--, P++) {
    Value = (Value << 8) | *P;
  }
  return Value;
}

static UCHAR
ExecuteScsi (PDEVICE_EXTENSION Dev, PSCSI_REQUEST_BLOCK Srb)
{
  const UCHAR  *Cdb = Srb->Cdb;

  switch (Cdb[0]) {
    case SCSIOP_START_STOP_UNIT:
    case SCSIOP_MEDIUM_REMOVAL:
    case SCSIOP_SYNCHRONIZE_CACHE:
    case SCSIOP_SYNCHRONIZE_CACHE16:
      Srb->DataTransferLength = 0;
      return SRB_STATUS_SUCCESS;

    case SCSIOP_TEST_UNIT_READY:
      Srb->DataTransferLength = 0;
      return Dev->MediaPresent ? SRB_STATUS_SUCCESS : NoMedia (Srb);

    case SCSIOP_VERIFY:
      Srb->DataTransferLength = 0;
      return Dev->MediaPresent ? SRB_STATUS_SUCCESS : NoMedia (Srb);

    case SCSIOP_REQUEST_SENSE:
      return ScsiRequestSense (Dev, Srb);

    case SCSIOP_INQUIRY:
      return ScsiInquiry (Dev, Srb);

    case SCSIOP_READ_CAPACITY:
      return Dev->MediaPresent ? ScsiReadCapacity (Dev, Srb, FALSE) : NoMedia (Srb);

    case SCSIOP_SERVICE_ACTION_IN16:
      if ((Cdb[1] & 0x1F) == SERVICE_ACTION_READ_CAPACITY16) {
        return Dev->MediaPresent ? ScsiReadCapacity (Dev, Srb, TRUE) : NoMedia (Srb);
      }
      return SRB_STATUS_INVALID_REQUEST;

    case SCSIOP_MODE_SENSE:
      return ScsiModeSense (Srb, FALSE);

    case SCSIOP_MODE_SENSE10:
      return ScsiModeSense (Srb, TRUE);

    case SCSIOP_READ:
    case SCSIOP_WRITE:
      if (!Dev->MediaPresent) {
        return NoMedia (Srb);
      }
      return ScsiReadWrite (Dev, Srb, Cdb[0] == SCSIOP_WRITE, Be (&Cdb[2], 4), (ULONG)Be (&Cdb[7], 2));

    case SCSIOP_READ16:
    case SCSIOP_WRITE16:
      if (!Dev->MediaPresent) {
        return NoMedia (Srb);
      }
      return ScsiReadWrite (Dev, Srb, Cdb[0] == SCSIOP_WRITE16, Be (&Cdb[2], 8), (ULONG)Be (&Cdb[10], 4));

    default:
      return SRB_STATUS_INVALID_REQUEST;
  }
}

//
// ---------------------------------------------------------------------------
// Storport worker (PASSIVE): card-detect work first, then queued requests.
// ---------------------------------------------------------------------------
//

static VOID
Worker (PVOID DeviceExtension, PVOID Context, PVOID WorkerHandle)
{
  PDEVICE_EXTENSION    Dev;
  PLIST_ENTRY          Entry;
  PSCSI_REQUEST_BLOCK  Srb;
  BOOLEAN              CardWork;
  KIRQL                Irql;

  UNREFERENCED_PARAMETER (Context);
  UNREFERENCED_PARAMETER (WorkerHandle);

  Dev = DeviceExtension;
  for ( ; ; ) {
    KeAcquireSpinLock (&Dev->Lock, &Irql);
    if (Dev->CdPending) {
      Dev->CdPending = FALSE;
      CardWork       = TRUE;
      Srb            = NULL;
    } else if (!IsListEmpty (&Dev->Pending)) {
      Entry = RemoveHeadList (&Dev->Pending);
      Srb   = CONTAINING_RECORD (Entry, SRB_EXT, Link)->Srb;
      CardWork = FALSE;
    } else {
      Dev->WorkerQueued = FALSE;
      KeReleaseSpinLock (&Dev->Lock, Irql);
      return;
    }
    KeReleaseSpinLock (&Dev->Lock, Irql);

    if (CardWork) {
      HandleCardDetect (Dev);
      continue;
    }

    Srb->SrbStatus = ExecuteScsi (Dev, Srb);
    Srb->ScsiStatus = (Srb->SrbStatus == SRB_STATUS_SUCCESS) ? SCSISTAT_GOOD : SCSISTAT_CHECK_CONDITION;

    //
    // The first commands (the disk class bring-up), every failure, and a
    // sign of life every 500 reads/writes.
    //
    Dev->Commands++;
    if ((Dev->Commands <= 60) || (Srb->SrbStatus != SRB_STATUS_SUCCESS)) {
      OsLog ("exynossd: SCSI 0x%02x len %lu -> srb 0x%02x (cmd %lu)", Srb->Cdb[0],
             Srb->DataTransferLength, Srb->SrbStatus, Dev->Commands);
    }
    if ((Srb->Cdb[0] == SCSIOP_READ) || (Srb->Cdb[0] == SCSIOP_WRITE) ||
        (Srb->Cdb[0] == SCSIOP_READ16) || (Srb->Cdb[0] == SCSIOP_WRITE16)) {
      if ((Dev->Transfers++ % 500) == 0) {
        OsLog ("exynossd: %lu transfers, last 0x%02x LBA %I64u", Dev->Transfers, Srb->Cdb[0],
               (Srb->Cdb[0] == SCSIOP_READ || Srb->Cdb[0] == SCSIOP_WRITE) ? Be (&Srb->Cdb[2], 4) : Be (&Srb->Cdb[2], 8));
      }
    }
    StorPortNotification (RequestComplete, Dev, Srb);
  }
}

//
// ---------------------------------------------------------------------------
// Storport callbacks.
// ---------------------------------------------------------------------------
//

static BOOLEAN
HwStartIo (PVOID DeviceExtension, PSCSI_REQUEST_BLOCK Srb)
{
  PDEVICE_EXTENSION  Dev;
  PSRB_EXT           Ext;
  KIRQL              Irql;
  BOOLEAN            Kick;

  Dev = DeviceExtension;

  switch (Srb->Function) {
    case SRB_FUNCTION_EXECUTE_SCSI:
      if ((Srb->PathId != 0) || (Srb->TargetId != 0) || (Srb->Lun != 0)) {
        Srb->SrbStatus = SRB_STATUS_SELECTION_TIMEOUT;
        break;
      }
      Ext      = Srb->SrbExtension;
      Ext->Srb = Srb;
      KeAcquireSpinLock (&Dev->Lock, &Irql);
      InsertTailList (&Dev->Pending, &Ext->Link);
      Kick = !Dev->WorkerQueued;
      Dev->WorkerQueued = TRUE;
      KeReleaseSpinLock (&Dev->Lock, Irql);
      if (Kick) {
        ULONG  QueueStatus = STOR_STATUS_UNSUCCESSFUL;

        if (Dev->Worker != NULL) {
          QueueStatus = StorPortQueueWorkItem (Dev, Worker, Dev->Worker, NULL);
        }
        if (QueueStatus != STOR_STATUS_SUCCESS) {
          //
          // No worker (yet): run the request here, at DISPATCH level.
          //
          if (Dev->Commands < 60) {
            OsLog ("exynossd: worker %p unavailable (0x%08lx), running inline", Dev->Worker, QueueStatus);
          }
          Worker (Dev, NULL, NULL);
        }
      }
      return TRUE;

    case SRB_FUNCTION_FLUSH:
    case SRB_FUNCTION_SHUTDOWN:
    case SRB_FUNCTION_RESET_BUS:
    case SRB_FUNCTION_RESET_DEVICE:
    case SRB_FUNCTION_RESET_LOGICAL_UNIT:
    case SRB_FUNCTION_PNP:
    case SRB_FUNCTION_POWER:
      Srb->SrbStatus = SRB_STATUS_SUCCESS;
      break;

    default:
      Srb->SrbStatus = SRB_STATUS_INVALID_REQUEST;
      break;
  }

  if (Srb->Function == SRB_FUNCTION_PNP) {
    OsLog ("exynossd: PnP action %d", ((PSCSI_PNP_REQUEST_BLOCK)Srb)->PnPAction);
  } else if (Dev->Commands < 60) {
    OsLog ("exynossd: SRB function 0x%02x -> 0x%02x", Srb->Function, Srb->SrbStatus);
  }
  StorPortNotification (RequestComplete, Dev, Srb);
  return TRUE;
}

static BOOLEAN
HwResetBus (PVOID DeviceExtension, ULONG PathId)
{
  UNREFERENCED_PARAMETER (DeviceExtension);
  UNREFERENCED_PARAMETER (PathId);
  return TRUE;
}

static BOOLEAN
HwPassiveInitialize (PVOID DeviceExtension)
{
  PDEVICE_EXTENSION  Dev;
  ULONG              Status;

  Dev    = DeviceExtension;
  Status = StorPortInitializeWorker (Dev, &Dev->Worker);
  OsLog ("exynossd: passive init, worker %p (0x%08lx)", Dev->Worker, Status);
  if (Status != STOR_STATUS_SUCCESS) {
    BringUpFailed (STOR_STEP_WORKER, Status);
    Dev->Worker = NULL;
  }
  return TRUE;
}

static BOOLEAN
HwInitialize (PVOID DeviceExtension)
{
  BOOLEAN  Enabled;

  Enabled = StorPortEnablePassiveInitialization (DeviceExtension, HwPassiveInitialize);
  OsLog ("exynossd: HwInitialize, passive init %s", Enabled ? "enabled" : "not available");
  return TRUE;
}

static SCSI_ADAPTER_CONTROL_STATUS
HwAdapterControl (PVOID DeviceExtension, SCSI_ADAPTER_CONTROL_TYPE ControlType, PVOID Parameters)
{
  PSCSI_SUPPORTED_CONTROL_TYPE_LIST  List;
  PDEVICE_EXTENSION                  Dev;

  Dev = DeviceExtension;

  switch (ControlType) {
    case ScsiQuerySupportedControlTypes:
      List = Parameters;
      if (List->MaxControlType > ScsiStopAdapter) {
        List->SupportedTypeList[ScsiStopAdapter] = TRUE;
      }
      if (List->MaxControlType > ScsiRestartAdapter) {
        List->SupportedTypeList[ScsiRestartAdapter] = TRUE;
      }
      List->SupportedTypeList[ScsiQuerySupportedControlTypes] = TRUE;
      return ScsiAdapterControlSuccess;
    case ScsiStopAdapter:
      (VOID)KeCancelTimer (&Dev->CdTimer);
      return ScsiAdapterControlSuccess;
    case ScsiRestartAdapter:
      return ScsiAdapterControlSuccess;
    default:
      return ScsiAdapterControlUnsuccessful;
  }
}

static ULONG
HwFindAdapter (PVOID DeviceExtension, PVOID HwContext, PVOID BusInformation, PVOID LowerDevice,
               PCHAR ArgumentString, PPORT_CONFIGURATION_INFORMATION ConfigInfo, PBOOLEAN Again)
{
  PDEVICE_EXTENSION       Dev;
  PHYSICAL_ADDRESS        Phys;
  LARGE_INTEGER           DueTime;

  UNREFERENCED_PARAMETER (HwContext);
  UNREFERENCED_PARAMETER (BusInformation);
  UNREFERENCED_PARAMETER (ArgumentString);
  UNREFERENCED_PARAMETER (LowerDevice);
  UNREFERENCED_PARAMETER (Again);

  Dev = DeviceExtension;
  OsLog ("exynossd: HwFindAdapter, interface %d, %lu access ranges",
         ConfigInfo->AdapterInterfaceType, ConfigInfo->NumberOfAccessRanges);
  KeInitializeSpinLock (&Dev->Lock);
  InitializeListHead (&Dev->Pending);
  KeInitializeTimer (&Dev->CdTimer);
  KeInitializeDpc (&Dev->CdDpc, CdTimerDpc, Dev);

  //
  // A virtual miniport may or may not see the ACPI memory resource; the
  // register block is known either way.
  //
  {
    ULONG             Size;

    Phys.QuadPart = DWMCI2_BASE;
    Size          = DWMCI2_SIZE;
    if ((ConfigInfo->NumberOfAccessRanges >= 1) && (ConfigInfo->AccessRanges != NULL) &&
        ((*ConfigInfo->AccessRanges)[0].RangeLength >= DWMCI2_SIZE)) {
      Phys = (*ConfigInfo->AccessRanges)[0].RangeStart;
      Size = (*ConfigInfo->AccessRanges)[0].RangeLength;
    }
    Dev->Base = MmMapIoSpace (Phys, Size, MmNonCached);
    Dev->Size = Size;
    if (Dev->Base == NULL) {
      BringUpFailed (STOR_STEP_MAP, Phys.LowPart);
      return SP_RETURN_ERROR;
    }
  }

  //
  // The CMU page (sclk_dwmci2) and both GPIO pages: UEFI only sets them up
  // when a card was present at boot, so the driver always needs them.
  //
  Phys.QuadPart = CMU_PHYS;
  Dev->Cmu = MmMapIoSpace (Phys, CMU_SIZE, MmNonCached);
  Dev->CmuSize = CMU_SIZE;
  if (Dev->Cmu == NULL) {
    BringUpFailed (STOR_STEP_MAP_CMU, CMU_PHYS);
    return SP_RETURN_ERROR;
  }
  Phys.QuadPart = GPIO_T_PHYS;
  Dev->GpioT = MmMapIoSpace (Phys, GPIO_T_SIZE, MmNonCached);
  Dev->GpioTSize = GPIO_T_SIZE;
  Phys.QuadPart = GPIO_RT_PHYS;
  Dev->GpioRt = MmMapIoSpace (Phys, GPIO_RT_SIZE, MmNonCached);
  Dev->GpioRtSize = GPIO_RT_SIZE;
  if ((Dev->GpioT == NULL) || (Dev->GpioRt == NULL)) {
    BringUpFailed (STOR_STEP_MAP_GPIO, GPIO_T_PHYS);
    return SP_RETURN_ERROR;
  }
  OsLog ("exynossd: dw_mmc2 mapped at %p, CIU %lu Hz", Dev->Base, DWMCI_CIU_HZ);

#if LOAD_TEST
  KeBugCheckEx (STOR_BUGCHECK, DWMCI2_BASE, DWMCI_CIU_HZ, ConfigInfo->NumberOfAccessRanges, 0);
#endif

  //
  // Card detect: plain input, no pull (the board pulls it up), then read it
  // before touching anything else - an empty slot stays powered off.
  //
  GpioCon (Dev->GpioRt, GPX2_CON, SD_CD_PIN, GPIO_FUNC_INPUT);
  GpioPud (Dev->GpioRt, GPX2_PUD, SD_CD_PIN, GPIO_PULL_NONE);
  Dev->CdConfirmed = SdCardPresent (Dev);
  if (Dev->CdConfirmed && !CdRaw (Dev)) {
    Dev->CdConfirmed = FALSE;
  }

  if (Dev->CdConfirmed) {
    if (SdTryInit (Dev)) {
      OsLog ("exynossd: card present at boot, %I64u blocks, %lu-bit, %lu Hz",
             Dev->Host.Card.BlockCount, Dev->Host.Card.BusWidth, Dev->Host.ClockHz);
    } else {
      OsLog ("exynossd: card present at boot but not responding (attempt %lu), will retry",
             Dev->InitTries);
    }
  } else {
    SdPowerDown (Dev);
    OsLog ("exynossd: no card in the slot, adapter stays up without media");
  }

  ConfigInfo->VirtualDevice               = TRUE;
  ConfigInfo->MaximumTransferLength       = MAX_TRANSFER_BLOCKS * BLOCK_SIZE;
  ConfigInfo->NumberOfPhysicalBreaks      = MAX_TRANSFER_BLOCKS + 1;
  ConfigInfo->AlignmentMask               = 3;
  ConfigInfo->NumberOfBuses               = 1;
  ConfigInfo->MaximumNumberOfTargets      = 1;
  ConfigInfo->MaximumNumberOfLogicalUnits = 1;
  ConfigInfo->InitiatorBusId[0]           = 1;
  //
  // Storport only starts adapters that claim scatter/gather bus-master DMA
  // (without it: StorRemoveDevice right after HwFindAdapter).  For a virtual
  // miniport it builds the S/G list itself from the request's MDL, with no
  // HAL DMA involved; the data still moves by PIO through the mapped
  // DataBuffer.
  //
  ConfigInfo->ScatterGather               = TRUE;
  ConfigInfo->Master                      = TRUE;
  ConfigInfo->Dma32BitAddresses           = TRUE;
  ConfigInfo->CachesData                  = FALSE;
  ConfigInfo->MapBuffers                  = STOR_MAP_ALL_BUFFERS_INCLUDING_READ_WRITE;

  //
  // Poll the card-detect pin: first tick in 1 s, then every 250 ms.
  //
  Dev->HwDone = TRUE;
  DueTime.QuadPart = -10000000LL;
  KeSetTimerEx (&Dev->CdTimer, DueTime, CD_POLL_MS, &Dev->CdDpc);
  OsLog ("exynossd: card-detect polling started (%d ms), media %s", CD_POLL_MS,
         Dev->MediaPresent ? "present" : "absent");
  return SP_RETURN_FOUND;
}

//
// Virtual miniport callbacks with nothing to do here.
//
static VOID
HwFreeAdapterResources (PVOID DeviceExtension)
{
  PDEVICE_EXTENSION  Dev;

  Dev = DeviceExtension;
  OsLog ("exynossd: free adapter resources");
  (VOID)KeCancelTimer (&Dev->CdTimer);
  if (Dev->GpioRt != NULL) {
    MmUnmapIoSpace (Dev->GpioRt, Dev->GpioRtSize);
    Dev->GpioRt = NULL;
  }
  if (Dev->GpioT != NULL) {
    MmUnmapIoSpace (Dev->GpioT, Dev->GpioTSize);
    Dev->GpioT = NULL;
  }
  if (Dev->Cmu != NULL) {
    MmUnmapIoSpace (Dev->Cmu, Dev->CmuSize);
    Dev->Cmu = NULL;
  }
  if (Dev->Base != NULL) {
    MmUnmapIoSpace (Dev->Base, Dev->Size);
    Dev->Base = NULL;
  }
}

static VOID
HwProcessServiceRequest (PVOID DeviceExtension, PVOID Irp)
{
  ((PIRP)Irp)->IoStatus.Status      = STATUS_NOT_SUPPORTED;
  ((PIRP)Irp)->IoStatus.Information = 0;
  StorPortExtendedFunction (ExtFunctionCompleteServiceIrp, DeviceExtension, Irp);
}

static VOID
HwCompleteServiceIrp (PVOID DeviceExtension)
{
  UNREFERENCED_PARAMETER (DeviceExtension);
}

ULONG
DriverEntry (PVOID DriverObject, PVOID RegistryPath)
{
  static const INTERFACE_TYPE     Types[] = { Internal, PNPBus, ACPIBus };
  VIRTUAL_HW_INITIALIZATION_DATA  Init;
  ULONG                           Status;
  ULONG                           Result;
  ULONG                           Index;

  OsLogInit ();
  OsLog ("exynossd: DriverEntry");

  RtlZeroMemory (&Init, sizeof (Init));
  Init.HwInitializationDataSize = sizeof (Init);
  Init.HwInitialize             = HwInitialize;
  Init.HwStartIo                = HwStartIo;
  Init.HwFindAdapter            = HwFindAdapter;
  Init.HwResetBus               = HwResetBus;
  Init.HwAdapterControl         = HwAdapterControl;
  Init.HwFreeAdapterResources   = HwFreeAdapterResources;
  Init.HwProcessServiceRequest  = HwProcessServiceRequest;
  Init.HwCompleteServiceIrp     = HwCompleteServiceIrp;
  Init.DeviceExtensionSize      = sizeof (DEVICE_EXTENSION);
  Init.SrbExtensionSize         = sizeof (SRB_EXT);
  Init.MapBuffers               = STOR_MAP_ALL_BUFFERS_INCLUDING_READ_WRITE;
  Init.NeedPhysicalAddresses    = TRUE;
  Init.AutoRequestSense         = TRUE;
  Init.TaggedQueuing            = FALSE;
  Init.MultipleRequestPerLu     = FALSE;

  //
  // The ACPI device shows up as ACPIBus here; register for the other bus
  // types an ACPI device can present too (INF Parameters\PnpInterface).
  //
  Result = (ULONG)STATUS_UNSUCCESSFUL;
  for (Index = 0; Index < ARRAYSIZE (Types); Index++) {
    Init.AdapterInterfaceType = Types[Index];
    Status = StorPortInitialize (DriverObject, RegistryPath, (PHW_INITIALIZATION_DATA)&Init, NULL);
    OsLog ("exynossd: StorPortInitialize (virtual, interface %d) 0x%08lx", Types[Index], Status);
    if (Status == STATUS_SUCCESS) {
      Result = STATUS_SUCCESS;
    }
  }
  if (Result != STATUS_SUCCESS) {
    BringUpFailed (STOR_STEP_INITIALIZE, Status);
  }
  return Result;
}
