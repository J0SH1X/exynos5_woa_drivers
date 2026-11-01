/** @file
  USB log + debug channel for Windows RT 8.1 on v1awifi (SM-P900): the
  Exynos5420 USB3.0 DRD0 (Synopsys DWC3) in peripheral mode, presenting the
  same 1209:0001 device as UEFI's UsbLogDxe, so Tools/usblog.py and
  Tools/usbdbg.py work against running Windows.

  - Interface 0, bulk IN 0x81: the OS log ring (ExynosOsLog.h, written via
    common/oslog.h), from the oldest byte still in the ring, then live.
  - Interface 1, bulk OUT 0x02 / IN 0x82: USBDBG_PING, READ, WRITE32 (on
    physical addresses, mapped with MmMapIoSpace) and RESET (normal reboot
    through the firmware).  The UEFI file commands are not available.

  ACPI: _HID EXYN5001 (UFN0), the controller's registers and SPI 72 (GSI
  104).  UEFI leaves clocks and the DRD PHY up and only stops the core at
  ExitBootServices, so this resets just the device core and reprograms it.

  Ported from Exynos5420Pkg UsbFnDxe (Dwc3.c: core bring-up, event buffer,
  EP0 control state machine, TRB transfers) and, for the function side,
  from Exynos441x exynosusb / UsbLogDxe.  The ISR only masks the event
  interrupt and queues the DPC; events are processed at DISPATCH_LEVEL
  under a spin lock, by the DPC and by a 10 ms timer (a lost interrupt must
  not wedge the channel).  DMA memory (event buffer, TRBs, bounce buffers)
  is one physically contiguous uncached block, so there is no cache upkeep.

  With LOAD_TEST it bugchecks 0x55534233 ("USB3") in PrepareHardware with
  P1 = memory base, P2 = interrupt (raw GSI), P3 = resource count.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <ntddk.h>
#include <wdf.h>
#include "oslog.h"

#ifndef LOAD_TEST
#define LOAD_TEST 0
#endif
#define USB_BUGCHECK  0x55534233

//
// DWC3 global registers.
//
#define GSBUSCFG0             0xC100
#define GSBUSCFG1             0xC104
#define GRXTHRCFG             0xC10C
#define GCTL                  0xC110
#define GSNPSID               0xC120
#define GUCTL                 0xC12C
#define GUSB2PHYCFG           0xC200
#define GUSB3PIPECTL          0xC2C0
#define GEVNTADRLO            0xC400
#define GEVNTADRHI            0xC404
#define GEVNTSIZ              0xC408
#define GEVNTCOUNT            0xC40C

#define GCTL_PRTCAPDIR_MASK   (0x3u << 12)
#define GCTL_PRTCAPDIR_DEVICE (0x2u << 12)
#define GUCTL_HSTINAUTORETRY  (1u << 14)
#define GRXTHRCFG_PKTCNTSEL   (1u << 29)
#define GRXTHRCFG_MASK        ((0xFu << 24) | (0x1Fu << 19))
#define GRXTHRCFG_VALUE       (GRXTHRCFG_PKTCNTSEL | (3u << 24) | (3u << 19))
#define GUSB2PHYCFG_TRDTIM_MASK (0xFu << 10)
#define GUSB2PHYCFG_TRDTIM(x) ((x) << 10)
#define GUSB2PHYCFG_ENBLSLPM  (1u << 8)
#define GUSB2PHYCFG_SUSPHY    (1u << 6)
#define GUSB2PHYCFG_PHYIF     (1u << 3)
#define GUSB3PIPECTL_DELAY_MASK (0x7u << 19)
#define GUSB3PIPECTL_DELAY(x) ((x) << 19)
#define GUSB3PIPECTL_SUSPSSPHY (1u << 17)
#define GEVNTSIZ_INTMASK      (1u << 31)

//
// DWC3 device registers.
//
#define DCFG                  0xC700
#define DCTL                  0xC704
#define DEVTEN                0xC708
#define DSTS                  0xC70C
#define DALEPENA              0xC720
#define DEPCMDPAR2(n)         (0xC800 + (n) * 0x10)
#define DEPCMDPAR1(n)         (0xC804 + (n) * 0x10)
#define DEPCMDPAR0(n)         (0xC808 + (n) * 0x10)
#define DEPCMD(n)             (0xC80C + (n) * 0x10)

#define DCFG_NUMP(x)          ((x) << 17)
#define DCFG_PERFRINT(x)      ((x) << 10)
#define DCFG_DEVADDR_MASK     (0x7Fu << 3)
#define DCFG_DEVADDR(x)       ((x) << 3)
#define DCFG_DEVSPD_HS        0

#define DCTL_RUNSTOP          (1u << 31)
#define DCTL_CSFTRST          (1u << 30)
#define DCTL_TSTCTL_MASK      (0xFu << 1)

#define DEVTEN_ULSTCNG        (1u << 3)
#define DEVTEN_CONNECTDONE    (1u << 2)
#define DEVTEN_USBRST         (1u << 1)
#define DEVTEN_DISCONN        (1u << 0)

#define DSTS_DEVCTRLHLT       (1u << 22)
#define DSTS_CONNECTSPD(x)    ((x) & 0x7)

#define DEPCMD_PARAM(x)       ((x) << 16)
#define DEPCMD_XFERRSCIDX(x)  (((x) >> 16) & 0x7F)
#define DEPCMD_STATUS(x)      (((x) >> 12) & 0xF)
#define DEPCMD_HIPRI_FORCERM  (1u << 11)
#define DEPCMD_CMDACT         (1u << 10)
#define DEPCMD_DEPCFG         0x1
#define DEPCMD_DEPXFERCFG     0x2
#define DEPCMD_DEPSSTALL      0x4
#define DEPCMD_DEPCSTALL      0x5
#define DEPCMD_DEPSTRTXFER    0x6
#define DEPCMD_DEPENDXFER     0x8
#define DEPCMD_DEPSTARTCFG    0x9

#define DEPCFG0_CFGACTION(x)  ((x) << 30)
#define DEPCFG0_FIFONUM(x)    ((x) << 17)
#define DEPCFG0_MPS(x)        ((x) << 3)
#define DEPCFG0_EPTYPE(x)     ((x) << 1)
#define DEPCFG1_EPNUM(x)      ((x) << 26)
#define DEPCFG1_EPDIR         (1u << 25)
#define DEPCFG1_XFERNRDYEN    (1u << 10)
#define DEPCFG1_XFERCMPLEN    (1u << 8)

//
// TRB.
//
typedef struct {
  ULONG  BufLo;
  ULONG  BufHi;
  ULONG  Size;
  ULONG  Ctrl;
} DWC3_TRB;

#define TRB_SIZE_MASK         0x00FFFFFFu
#define TRB_CTRL_IOC          (1u << 11)
#define TRB_CTRL_ISP_IMI      (1u << 10)
#define TRB_CTRL_TRBCTL(x)    ((x) << 4)
#define TRB_CTRL_CHN          (1u << 2)
#define TRB_CTRL_LST          (1u << 1)
#define TRB_CTRL_HWO          (1u << 0)
#define TRBCTL_NORMAL         1
#define TRBCTL_CONTROL_SETUP  2
#define TRBCTL_CONTROL_STATUS2 3
#define TRBCTL_CONTROL_STATUS3 4
#define TRBCTL_CONTROL_DATA   5

//
// Events.
//
#define EVT_IS_DEVICE(e)      (((e) & 1) != 0)
#define EVT_DEVT_TYPE(e)      (((e) >> 1) & 0x7F)
#define DEPEVT_EPNUM(e)       (((e) >> 1) & 0x1F)
#define DEPEVT_TYPE(e)        (((e) >> 6) & 0xF)
#define DEPEVT_XFERCOMPLETE   1
#define DEPEVT_XFERNOTREADY   3
#define DEPEVT_CTL_STATUS(e)  (((e) >> 12) & 0x3)
#define CTL_STATUS_SETUP      0
#define CTL_STATUS_DATA       1
#define CTL_STATUS_STATUS     2
#define DEVT_TYPE(e)          (((e) >> 8) & 0xF)
#define DEVT_DISCONN          0
#define DEVT_USBRST           1
#define DEVT_CONNECTDONE      2

//
// Endpoints and DMA pool.
//
#define MAX_EPS        4
#define PHYS_EPS       (MAX_EPS * 2)
#define PHYS_EP(Ep, In) ((UCHAR)(((Ep) << 1) | ((In) ? 1 : 0)))
#define EP0_MPS        64
#define BULK_MPS       512
#define EP_BULK        2
#define LOG_EP         1
#define DBG_EP         2

#define EVENT_BUFFER_SIZE  4096
#define TRB_SLOT           64
#define EP0_BUFFER         512
#define BOUNCE_SIZE        8192
#define POOL_SIZE          (EVENT_BUFFER_SIZE + PAGE_SIZE + PAGE_SIZE + (PHYS_EPS - 2) * BOUNCE_SIZE)

//
// USB requests and descriptor types.
//
#define REQ_GET_STATUS     0x00
#define REQ_CLEAR_FEATURE  0x01
#define REQ_SET_FEATURE    0x03
#define REQ_SET_ADDRESS    0x05
#define REQ_GET_DESCRIPTOR 0x06
#define REQ_GET_CONFIG     0x08
#define REQ_SET_CONFIG     0x09
#define DESC_DEVICE        0x01
#define DESC_CONFIG        0x02
#define DESC_STRING        0x03

//
// Debug channel wire format (UsbLogDxe.c).
//
#define USBDBG_REQ_MAGIC   0x47424455      // 'UDBG'
#define USBDBG_RSP_MAGIC   0x52424455      // 'UDBR'
#define USBDBG_MAX_READ    4096
#define USBDBG_PING        1
#define USBDBG_READ        2
#define USBDBG_WRITE32     3
#define USBDBG_RESET       4
#define USBDBG_OK          0
#define USBDBG_EBADCMD     1
#define USBDBG_EBADARG     2

#define LOG_CHUNK          4096
#define POLL_MS            10

#define DWC3_ROUND_UP(x, a)     (((x) + (a) - 1) & ~((a) - 1))

#include <pshpack1.h>
typedef struct {
  UCHAR   RequestType;
  UCHAR   Request;
  USHORT  Value;
  USHORT  Index;
  USHORT  Length;
} USB_SETUP;

typedef struct {
  ULONG  Magic;
  ULONG  Command;
  ULONG  Arg0;
  ULONG  Arg1;
} USBDBG_HEADER;

typedef struct {
  ULONG  Magic;
  ULONG  Status;
  ULONG  Length;
  ULONG  Reserved;
} USBDBG_REPLY;
#include <poppack.h>

static const UCHAR mDeviceDescriptor[18] = {
  18, DESC_DEVICE, 0x00, 0x02, 0xFF, 0x00, 0x00, EP0_MPS,
  0x09, 0x12, 0x01, 0x00,          // 1209:0001
  0x00, 0x01, 1, 2, 3, 1
};

static const UCHAR mConfigDescriptor[48] = {
  9, DESC_CONFIG, 48, 0, 2, 1, 0, 0x80, 250,
  9, 0x04, 0, 0, 1, 0xFF, 0x00, 0x00, 0,                 // interface 0: log
  7, 0x05, 0x80 | LOG_EP, EP_BULK, BULK_MPS & 0xFF, BULK_MPS >> 8, 0,
  9, 0x04, 1, 0, 2, 0xFF, 0x00, 0x00, 0,                 // interface 1: debug
  7, 0x05, DBG_EP, EP_BULK, BULK_MPS & 0xFF, BULK_MPS >> 8, 0,
  7, 0x05, 0x80 | DBG_EP, EP_BULK, BULK_MPS & 0xFF, BULK_MPS >> 8, 0,
};

//
// Physical endpoints the configuration uses: EP1 IN, EP2 OUT, EP2 IN.
//
static const UCHAR mBulkPhys[] = { PHYS_EP (LOG_EP, TRUE), PHYS_EP (DBG_EP, FALSE), PHYS_EP (DBG_EP, TRUE) };

typedef enum {
  Ep0Idle = 0,
  Ep0SetupStage,
  Ep0DataStage,
  Ep0WaitStatus,
  Ep0StatusStage
} EP0_STATE;

typedef struct {
  BOOLEAN  Active;
  BOOLEAN  Internal;      // EP0 reply owned by the driver
  BOOLEAN  SendZlp;
  BOOLEAN  Started;       // TRB handed to the core
  PUCHAR   Buffer;
  ULONG    Length;
  ULONG    Programmed;
  ULONG    Done;
  UCHAR    Resource;
} XFER;

typedef enum {
  EvReset = 1,
  EvSetup,
  EvTxDone,
  EvRxDone,
} USB_EVENT_TYPE;

typedef struct {
  USB_EVENT_TYPE  Type;
  UCHAR           Ep;
  ULONG           Bytes;
  USB_SETUP       Setup;
} USB_EVENT;

#define EVENT_QUEUE  32

typedef struct {
  PUCHAR        Base;
  SIZE_T        Size;
  WDFINTERRUPT  Interrupt;
  WDFSPINLOCK   Lock;
  WDFTIMER      Timer;
  WDFWORKITEM   ResetWork;
  BOOLEAN       Running;
  USHORT        Release;

  //
  // DMA pool.
  //
  PUCHAR        Pool;
  ULONG         PoolPhys;
  PULONG        EventBuffer;
  ULONG         EventIndex;
  DWC3_TRB      *Trb[PHYS_EPS];
  PUCHAR        Bounce[PHYS_EPS];
  PUCHAR        SetupBuffer;

  //
  // Controller state.
  //
  UCHAR         Configuration;
  BOOLEAN       EpsConfigured;
  BOOLEAN       Halted[PHYS_EPS];
  XFER          In[MAX_EPS];
  XFER          Out[MAX_EPS];
  EP0_STATE     Ep0State;
  BOOLEAN       Ep0ThreeStage;
  BOOLEAN       Ep0DirIn;
  BOOLEAN       Ep0StatusReady;
  BOOLEAN       Ep0StatusPending;
  UCHAR         Ep0StatusPhys;
  UCHAR         Ep0Buffer[256];

  USB_EVENT     Queue[EVENT_QUEUE];
  ULONG         QueueHead;
  ULONG         QueueTail;

  //
  // Function side (UsbLogDxe).
  //
  BOOLEAN       TxBusy;
  ULONG         ReadPos;
  ULONG         TxEnd;
  UCHAR         Tx[LOG_CHUNK];
  UCHAR         Ep0Reply[64];
  BOOLEAN       DbgRxPosted;
  BOOLEAN       DbgTxBusy;
  BOOLEAN       PendingReset;
  UCHAR         DbgRx[sizeof (USBDBG_HEADER) + 64];
  UCHAR         DbgTx[sizeof (USBDBG_REPLY) + USBDBG_MAX_READ];
} DEVICE_CONTEXT, *PDEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (DEVICE_CONTEXT, GetDeviceContext)

//
// HAL export without a public prototype: reboot through the firmware's
// ResetSystem.
//
typedef enum {
  HalHaltRoutine,
  HalPowerDownRoutine,
  HalRestartRoutine,
  HalRebootRoutine,
} FIRMWARE_REENTRY;

NTHALAPI VOID HalReturnToFirmware (FIRMWARE_REENTRY Routine);

DRIVER_INITIALIZE                DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD        EvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE  EvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE  EvtReleaseHardware;
EVT_WDF_DEVICE_D0_ENTRY          EvtD0Entry;
EVT_WDF_DEVICE_D0_EXIT           EvtD0Exit;
EVT_WDF_INTERRUPT_ISR            EvtIsr;
EVT_WDF_INTERRUPT_DPC            EvtDpc;
EVT_WDF_TIMER                    EvtTimer;
EVT_WDF_WORKITEM                 EvtResetWork;

static ULONG Rd (PDEVICE_CONTEXT D, ULONG R)          { return READ_REGISTER_ULONG ((PULONG)(D->Base + R)); }
static VOID  Wr (PDEVICE_CONTEXT D, ULONG R, ULONG V) { WRITE_REGISTER_ULONG ((PULONG)(D->Base + R), V); }
static VOID  Or (PDEVICE_CONTEXT D, ULONG R, ULONG V) { Wr (D, R, Rd (D, R) | V); }
static VOID  And (PDEVICE_CONTEXT D, ULONG R, ULONG V){ Wr (D, R, Rd (D, R) & V); }

static ULONG
PoolPhys (PDEVICE_CONTEXT D, const VOID *Va)
{
  return D->PoolPhys + (ULONG)((const UCHAR *)Va - D->Pool);
}

static XFER *
XferOf (PDEVICE_CONTEXT D, UCHAR Phys)
{
  return ((Phys & 1) != 0) ? &D->In[Phys >> 1] : &D->Out[Phys >> 1];
}

static BOOLEAN
PollClear (PDEVICE_CONTEXT D, ULONG Reg, ULONG Mask, ULONG TimeoutUs)
{
  while ((Rd (D, Reg) & Mask) != 0) {
    if (TimeoutUs-- == 0) {
      return FALSE;
    }
    KeStallExecutionProcessor (1);
  }
  return TRUE;
}

static BOOLEAN
PollSet (PDEVICE_CONTEXT D, ULONG Reg, ULONG Mask, ULONG TimeoutUs)
{
  while ((Rd (D, Reg) & Mask) == 0) {
    if (TimeoutUs-- == 0) {
      return FALSE;
    }
    KeStallExecutionProcessor (1);
  }
  return TRUE;
}

//
// ---------------------------------------------------------------------------
// DWC3 core (UsbFnDxe Dwc3.c), under D->Lock.
// ---------------------------------------------------------------------------
//

static ULONG
DepCmd (PDEVICE_CONTEXT D, UCHAR Phys, ULONG Cmd, ULONG P0, ULONG P1, ULONG P2)
{
  ULONG  Reg;

  Wr (D, DEPCMDPAR2 (Phys), P2);
  Wr (D, DEPCMDPAR1 (Phys), P1);
  Wr (D, DEPCMDPAR0 (Phys), P0);
  Wr (D, DEPCMD (Phys), Cmd | DEPCMD_CMDACT);
  if (!PollClear (D, DEPCMD (Phys), DEPCMD_CMDACT, 1000)) {
    OsLog ("exynosdwc3: DEPCMD %lx on ep%u timed out", Cmd, Phys);
    return MAXULONG;
  }
  Reg = Rd (D, DEPCMD (Phys));
  if (DEPCMD_STATUS (Reg) != 0) {
    OsLog ("exynosdwc3: DEPCMD %lx on ep%u status %lx", Cmd, Phys, DEPCMD_STATUS (Reg));
  }
  return Reg;
}

static BOOLEAN
StartTrb (PDEVICE_CONTEXT D, UCHAR Phys, PUCHAR Buffer, ULONG Length, ULONG TrbCtl, BOOLEAN Zlp)
{
  DWC3_TRB  *Trb;
  XFER      *X;
  ULONG     Ctrl;
  ULONG     Reg;

  Trb  = D->Trb[Phys];
  X    = XferOf (D, Phys);
  Ctrl = TRB_CTRL_TRBCTL (TrbCtl) | TRB_CTRL_HWO;
  if ((Phys & 1) == 0) {
    Ctrl |= TRB_CTRL_ISP_IMI;
  }

  Trb[0].BufLo = PoolPhys (D, Buffer);
  Trb[0].BufHi = 0;
  Trb[0].Size  = Length & TRB_SIZE_MASK;
  if (Zlp) {
    Trb[1].BufLo = PoolPhys (D, Buffer);
    Trb[1].BufHi = 0;
    Trb[1].Size  = 0;
    Trb[1].Ctrl  = Ctrl | TRB_CTRL_LST | TRB_CTRL_IOC;
    Trb[0].Ctrl  = Ctrl | TRB_CTRL_CHN;
  } else {
    Trb[0].Ctrl = Ctrl | TRB_CTRL_LST | TRB_CTRL_IOC;
  }
  __dmb (0xF);                                   // TRB in memory before the doorbell

  Reg = DepCmd (D, Phys, DEPCMD_DEPSTRTXFER, 0, PoolPhys (D, Trb), 0);
  if ((Reg == MAXULONG) || (DEPCMD_STATUS (Reg) != 0)) {
    Trb[0].Ctrl = 0;
    return FALSE;
  }
  X->Resource   = (UCHAR)DEPCMD_XFERRSCIDX (Reg);
  X->Programmed = Length;
  X->Started    = TRUE;
  return TRUE;
}

static ULONG
TrbDone (PDEVICE_CONTEXT D, UCHAR Phys)
{
  ULONG  Left;
  XFER   *X;

  X    = XferOf (D, Phys);
  Left = D->Trb[Phys][0].Size & TRB_SIZE_MASK;
  return (Left <= X->Programmed) ? (X->Programmed - Left) : 0;
}

static VOID
EndXfer (PDEVICE_CONTEXT D, UCHAR Phys)
{
  XFER  *X;

  X = XferOf (D, Phys);
  if (!X->Started) {
    return;
  }
  DepCmd (D, Phys, DEPCMD_DEPENDXFER | DEPCMD_PARAM (X->Resource) | DEPCMD_HIPRI_FORCERM, 0, 0, 0);
  KeStallExecutionProcessor (100);
  X->Started = FALSE;
}

static VOID
QueueEvent (PDEVICE_CONTEXT D, USB_EVENT_TYPE Type, ULONG Ep, ULONG Bytes, const USB_SETUP *Setup)
{
  ULONG  Next;

  Next = (D->QueueHead + 1) % EVENT_QUEUE;
  if (Next == D->QueueTail) {
    return;
  }
  RtlZeroMemory (&D->Queue[D->QueueHead], sizeof (USB_EVENT));
  D->Queue[D->QueueHead].Type  = Type;
  D->Queue[D->QueueHead].Ep    = (UCHAR)Ep;
  D->Queue[D->QueueHead].Bytes = Bytes;
  if (Setup != NULL) {
    D->Queue[D->QueueHead].Setup = *Setup;
  }
  D->QueueHead = Next;
}

//
// EP0.
//

static VOID
Ep0ArmSetup (PDEVICE_CONTEXT D)
{
  D->Ep0State         = Ep0SetupStage;
  D->Ep0StatusReady   = FALSE;
  D->Ep0StatusPending = FALSE;
  if (!StartTrb (D, 0, D->SetupBuffer, 8, TRBCTL_CONTROL_SETUP, FALSE)) {
    OsLog ("exynosdwc3: cannot arm SETUP");
  }
}

static VOID
Ep0StartStatus (PDEVICE_CONTEXT D)
{
  UCHAR  Phys;

  Phys        = D->Ep0StatusPhys;
  D->Ep0State = Ep0StatusStage;
  StartTrb (D, Phys, D->Bounce[Phys], 0,
            D->Ep0ThreeStage ? TRBCTL_CONTROL_STATUS3 : TRBCTL_CONTROL_STATUS2, FALSE);
}

static VOID
Ep0Abort (PDEVICE_CONTEXT D)
{
  EndXfer (D, 0);
  EndXfer (D, 1);
  RtlZeroMemory (&D->In[0], sizeof (D->In[0]));
  RtlZeroMemory (&D->Out[0], sizeof (D->Out[0]));
}

static VOID
Ep0Stall (PDEVICE_CONTEXT D)
{
  Ep0Abort (D);
  DepCmd (D, 0, DEPCMD_DEPSSTALL, 0, 0, 0);
  Ep0ArmSetup (D);
}

static VOID
Ep0StatusReady (PDEVICE_CONTEXT D)
{
  D->Ep0StatusReady = TRUE;
  if ((D->Ep0State == Ep0WaitStatus) && D->Ep0StatusPending) {
    Ep0StartStatus (D);
  }
}

/**
  Start the transfer described by D->In/Out[Ep] (Active, Buffer, Length,
  Internal and SendZlp set by the caller).
**/
static BOOLEAN
StartTransfer (PDEVICE_CONTEXT D, ULONG Ep, BOOLEAN In)
{
  XFER   *X;
  UCHAR  Phys;
  ULONG  Length;

  X    = In ? &D->In[Ep] : &D->Out[Ep];
  Phys = PHYS_EP (Ep, In);
  if (X->Started) {
    return FALSE;
  }

  if (Ep == 0) {
    if ((D->Ep0State == Ep0WaitStatus) && !D->Ep0ThreeStage && In && (X->Length == 0)) {
      X->Active = FALSE;
      Ep0StatusReady (D);
      return TRUE;
    }
    if ((D->Ep0State != Ep0DataStage) || (In != D->Ep0DirIn)) {
      return FALSE;
    }
    Length = min (X->Length, (ULONG)EP0_BUFFER);
    if (In) {
      RtlCopyMemory (D->Bounce[Phys], X->Buffer, Length);
    } else {
      Length = min (DWC3_ROUND_UP (max (Length, 1), EP0_MPS), (ULONG)EP0_BUFFER);
    }
    return StartTrb (D, Phys, D->Bounce[Phys], Length, TRBCTL_CONTROL_DATA, In && X->SendZlp);
  }

  if (!D->EpsConfigured) {
    return FALSE;
  }
  Length = min (X->Length, (ULONG)BOUNCE_SIZE);
  if (In) {
    RtlCopyMemory (D->Bounce[Phys], X->Buffer, Length);
  } else {
    Length = min (DWC3_ROUND_UP (max (Length, 1), BULK_MPS), (ULONG)BOUNCE_SIZE);
  }
  return StartTrb (D, Phys, D->Bounce[Phys], Length, TRBCTL_NORMAL, FALSE);
}

static VOID
Ep0Reply (PDEVICE_CONTEXT D, const VOID *Data, ULONG Length, USHORT WLength)
{
  XFER  *X;

  Length = min (min (Length, (ULONG)WLength), sizeof (D->Ep0Buffer));
  if (Length != 0) {
    RtlCopyMemory (D->Ep0Buffer, Data, Length);
  }
  X = &D->In[0];
  RtlZeroMemory (X, sizeof (*X));
  X->Active   = TRUE;
  X->Internal = TRUE;
  X->Buffer   = D->Ep0Buffer;
  X->Length   = Length;
  X->SendZlp  = (Length < WLength) && (Length != 0) && ((Length % EP0_MPS) == 0);
  if (!StartTransfer (D, 0, TRUE)) {
    X->Active = FALSE;
  }
}

static VOID
SetStall (PDEVICE_CONTEXT D, ULONG Ep, BOOLEAN In, BOOLEAN Stall)
{
  UCHAR  Phys;

  if (Ep == 0) {
    if (Stall) {
      Ep0Stall (D);
    }
    return;
  }
  Phys = PHYS_EP (Ep, In);
  DepCmd (D, Phys, Stall ? DEPCMD_DEPSSTALL : DEPCMD_DEPCSTALL, 0, 0, 0);
  D->Halted[Phys] = Stall;
}

static VOID
ActivateEndpoints (PDEVICE_CONTEXT D)
{
  ULONG  Index;
  UCHAR  Phys;
  ULONG  Ep;
  BOOLEAN In;

  if (D->EpsConfigured) {
    return;
  }
  DepCmd (D, 0, DEPCMD_DEPSTARTCFG | DEPCMD_PARAM (2), 0, 0, 0);
  for (Index = 0; Index < ARRAYSIZE (mBulkPhys); Index++) {
    Phys = mBulkPhys[Index];
    Ep   = Phys >> 1;
    In   = (Phys & 1) != 0;
    DepCmd (D, Phys, DEPCMD_DEPCFG,
            DEPCFG0_EPTYPE (EP_BULK) | DEPCFG0_MPS (BULK_MPS) | (In ? DEPCFG0_FIFONUM (Ep) : 0),
            DEPCFG1_EPNUM (Ep) | (In ? DEPCFG1_EPDIR : 0) | DEPCFG1_XFERCMPLEN, 0);
    DepCmd (D, Phys, DEPCMD_DEPXFERCFG, 1, 0, 0);
    Or (D, DALEPENA, 1u << Phys);
    D->Halted[Phys] = FALSE;
  }
  D->EpsConfigured = TRUE;
}

static VOID
DeactivateEndpoints (PDEVICE_CONTEXT D)
{
  UCHAR  Phys;

  for (Phys = 2; Phys < PHYS_EPS; Phys++) {
    EndXfer (D, Phys);
    if (D->Halted[Phys]) {
      DepCmd (D, Phys, DEPCMD_DEPCSTALL, 0, 0, 0);
      D->Halted[Phys] = FALSE;
    }
  }
  Wr (D, DALEPENA, 0x3);
  RtlZeroMemory (&D->In[1], sizeof (XFER) * (MAX_EPS - 1));
  RtlZeroMemory (&D->Out[1], sizeof (XFER) * (MAX_EPS - 1));
  D->EpsConfigured = FALSE;
  D->Configuration = 0;
}

/**
  Standard requests the driver answers itself.
**/
static BOOLEAN
Ep0Standard (PDEVICE_CONTEXT D, const USB_SETUP *Req)
{
  ULONG   Ep;
  USHORT  Status;

  if ((Req->RequestType & 0x60) != 0) {
    return FALSE;
  }

  switch (Req->Request) {
    case REQ_SET_ADDRESS:
      Wr (D, DCFG, (Rd (D, DCFG) & ~DCFG_DEVADDR_MASK) | DCFG_DEVADDR (Req->Value & 0x7F));
      Ep0StatusReady (D);
      return TRUE;

    case REQ_GET_DESCRIPTOR:
      switch (Req->Value >> 8) {
        case DESC_DEVICE:
          Ep0Reply (D, mDeviceDescriptor, sizeof (mDeviceDescriptor), Req->Length);
          return TRUE;
        case DESC_CONFIG:
          Ep0Reply (D, mConfigDescriptor, sizeof (mConfigDescriptor), Req->Length);
          return TRUE;
        case DESC_STRING:
          return FALSE;
        default:
          Ep0Stall (D);
          return TRUE;
      }

    case REQ_SET_CONFIG:
      D->Configuration = (UCHAR)Req->Value;
      if (D->Configuration != 0) {
        ActivateEndpoints (D);
      }
      Ep0StatusReady (D);
      return TRUE;

    case REQ_GET_CONFIG:
      Ep0Reply (D, &D->Configuration, 1, Req->Length);
      return TRUE;

    case REQ_GET_STATUS:
      Status = 0;
      if ((Req->RequestType & 0x1F) == 2) {
        Ep     = Req->Index & 0xF;
        Status = ((Ep != 0) && (Ep < MAX_EPS) && D->Halted[PHYS_EP (Ep, (Req->Index & 0x80) != 0)]) ? 1 : 0;
      }
      Ep0Reply (D, &Status, sizeof (Status), Req->Length);
      return TRUE;

    case REQ_CLEAR_FEATURE:
    case REQ_SET_FEATURE:
      if (((Req->RequestType & 0x1F) == 2) && (Req->Value == 0)) {
        Ep = Req->Index & 0xF;
        if ((Ep != 0) && (Ep < MAX_EPS)) {
          SetStall (D, Ep, (Req->Index & 0x80) != 0, Req->Request == REQ_SET_FEATURE);
        }
      }
      Ep0StatusReady (D);
      return TRUE;

    default:
      return FALSE;
  }
}

static VOID
Ep0XferComplete (PDEVICE_CONTEXT D, UCHAR Phys)
{
  USB_SETUP  Req;
  XFER       *X;
  ULONG      Done;

  X          = XferOf (D, Phys);
  X->Started = FALSE;

  switch (D->Ep0State) {
    case Ep0SetupStage:
      RtlCopyMemory (&Req, D->SetupBuffer, sizeof (Req));
      RtlZeroMemory (&D->In[0], sizeof (D->In[0]));
      RtlZeroMemory (&D->Out[0], sizeof (D->Out[0]));
      D->Ep0ThreeStage    = (Req.Length != 0);
      D->Ep0DirIn         = (Req.RequestType & 0x80) != 0;
      D->Ep0StatusReady   = FALSE;
      D->Ep0StatusPending = FALSE;
      D->Ep0State         = D->Ep0ThreeStage ? Ep0DataStage : Ep0WaitStatus;
      if (!Ep0Standard (D, &Req)) {
        QueueEvent (D, EvSetup, 0, 0, &Req);
      }
      break;

    case Ep0DataStage:
      X    = D->Ep0DirIn ? &D->In[0] : &D->Out[0];
      Done = TrbDone (D, Phys);
      if (!D->Ep0DirIn) {
        Done = min (Done, X->Length);
        RtlCopyMemory (X->Buffer, D->Bounce[Phys], Done);
      }
      X->Done   = Done;
      X->Active = FALSE;
      D->Ep0State = Ep0WaitStatus;
      Ep0StatusReady (D);
      break;

    default:
      Ep0ArmSetup (D);
      break;
  }
}

static VOID
Ep0XferNotReady (PDEVICE_CONTEXT D, UCHAR Phys, ULONG Event)
{
  ULONG  Stage;

  Stage = DEPEVT_CTL_STATUS (Event);

  //
  // A new SETUP while the last request is unfinished: take the new one.
  //
  if ((Stage == CTL_STATUS_SETUP) && (D->Ep0State != Ep0SetupStage)) {
    Ep0Abort (D);
    Ep0ArmSetup (D);
    return;
  }

  switch (D->Ep0State) {
    case Ep0SetupStage:
      if (Stage != CTL_STATUS_SETUP) {
        Ep0Stall (D);
      }
      break;

    case Ep0DataStage:
      if ((Stage == CTL_STATUS_DATA) && (((Phys & 1) != 0) != D->Ep0DirIn)) {
        Ep0Stall (D);
      }
      break;

    case Ep0WaitStatus:
      if (Stage == CTL_STATUS_DATA) {
        Ep0Stall (D);
        break;
      }
      if (Stage == CTL_STATUS_STATUS) {
        D->Ep0StatusPending = TRUE;
        D->Ep0StatusPhys    = Phys;
        if (D->Ep0StatusReady) {
          Ep0StartStatus (D);
        }
      }
      break;

    default:
      break;
  }
}

static VOID
EpXferComplete (PDEVICE_CONTEXT D, UCHAR Phys)
{
  XFER     *X;
  ULONG    Done;
  BOOLEAN  In;

  X  = XferOf (D, Phys);
  In = (Phys & 1) != 0;
  if (!X->Started) {
    return;
  }
  X->Started = FALSE;

  Done = TrbDone (D, Phys);
  if (!In) {
    Done = min (Done, X->Length);
    RtlCopyMemory (X->Buffer, D->Bounce[Phys], Done);
  }
  X->Done = Done;
  if (X->Active) {
    X->Active = FALSE;
    QueueEvent (D, In ? EvTxDone : EvRxDone, Phys >> 1, Done, NULL);
  }
}

static VOID
OnBusReset (PDEVICE_CONTEXT D)
{
  And (D, DCTL, ~DCTL_TSTCTL_MASK);
  And (D, GUSB2PHYCFG, ~GUSB2PHYCFG_SUSPHY);
  And (D, GUSB3PIPECTL, ~GUSB3PIPECTL_SUSPSSPHY);
  DeactivateEndpoints (D);
  if (!((D->Ep0State == Ep0SetupStage) && D->Out[0].Started)) {
    Ep0Abort (D);
    Ep0ArmSetup (D);
  }
  And (D, DCFG, ~DCFG_DEVADDR_MASK);
  QueueEvent (D, EvReset, 0, 0, NULL);
}

static VOID
OnConnectDone (PDEVICE_CONTEXT D)
{
  OsLog ("exynosdwc3: connected, speed %lu", DSTS_CONNECTSPD (Rd (D, DSTS)));
  Or (D, GUSB3PIPECTL, GUSB3PIPECTL_SUSPSSPHY);
  DepCmd (D, 0, DEPCMD_DEPCFG, DEPCFG0_MPS (EP0_MPS) | DEPCFG0_CFGACTION (2),
          DEPCFG1_XFERNRDYEN | DEPCFG1_XFERCMPLEN, 0);
  DepCmd (D, 1, DEPCMD_DEPCFG, DEPCFG0_MPS (EP0_MPS) | DEPCFG0_CFGACTION (2),
          DEPCFG1_EPDIR | DEPCFG1_XFERNRDYEN | DEPCFG1_XFERCMPLEN, 0);
}

static VOID
HandleEvent (PDEVICE_CONTEXT D, ULONG Event)
{
  UCHAR  Phys;

  if (!EVT_IS_DEVICE (Event)) {
    Phys = (UCHAR)DEPEVT_EPNUM (Event);
    if (Phys >= PHYS_EPS) {
      return;
    }
    switch (DEPEVT_TYPE (Event)) {
      case DEPEVT_XFERCOMPLETE:
        if (Phys < 2) {
          Ep0XferComplete (D, Phys);
        } else {
          EpXferComplete (D, Phys);
        }
        break;
      case DEPEVT_XFERNOTREADY:
        if (Phys < 2) {
          Ep0XferNotReady (D, Phys, Event);
        }
        break;
    }
    return;
  }

  if (EVT_DEVT_TYPE (Event) != 0) {
    return;
  }
  switch (DEVT_TYPE (Event)) {
    case DEVT_USBRST:
      OnBusReset (D);
      break;
    case DEVT_CONNECTDONE:
      OnConnectDone (D);
      break;
    case DEVT_DISCONN:
      DeactivateEndpoints (D);
      QueueEvent (D, EvReset, 0, 0, NULL);
      break;
  }
}

static VOID
ServiceEvents (PDEVICE_CONTEXT D)
{
  ULONG  Count;
  ULONG  Rounds;

  for (Rounds = 0; Rounds < 8; Rounds++) {
    Count = Rd (D, GEVNTCOUNT) & 0xFFFC;
    if (Count == 0) {
      return;
    }
    while (Count != 0) {
      HandleEvent (D, D->EventBuffer[D->EventIndex / 4]);
      D->EventIndex = (D->EventIndex + 4) % EVENT_BUFFER_SIZE;
      Wr (D, GEVNTCOUNT, 4);
      Count -= 4;
    }
  }
}

/**
  Device core reset and device-mode setup (Dwc3CoreInit without the clock
  and PHY bring-up UEFI already did); EP0 armed, not connected.
**/
static NTSTATUS
CoreInit (PDEVICE_CONTEXT D)
{
  ULONG  Id;
  ULONG  Reg;

  Id = Rd (D, GSNPSID);
  if ((Id >> 16) != 0x5533) {
    OsLog ("exynosdwc3: no DWC3 (GSNPSID %08lx)", Id);
    return STATUS_DEVICE_CONFIGURATION_ERROR;
  }
  D->Release = (USHORT)Id;

  And (D, DCTL, ~DCTL_RUNSTOP);
  PollSet (D, DSTS, DSTS_DEVCTRLHLT, 10000);

  Wr (D, GCTL, (Rd (D, GCTL) & ~GCTL_PRTCAPDIR_MASK) | GCTL_PRTCAPDIR_DEVICE);
  Or (D, DCTL, DCTL_CSFTRST);
  if (!PollClear (D, DCTL, DCTL_CSFTRST, 100000)) {
    OsLog ("exynosdwc3: DCTL.CSftRst stuck");
    return STATUS_DEVICE_NOT_READY;
  }

  //
  // exynos_drd_config() / exynos_drd_phy_set() core-side settings.
  //
  Wr (D, GSBUSCFG0, (1u << 3) | (1u << 0));      // INCR16 | INCR
  Wr (D, GSBUSCFG1, 3u << 8);                    // BREQLIMIT 3
  if ((D->Release >= 0x190a) && (D->Release <= 0x210a)) {
    Or (D, GUCTL, GUCTL_HSTINAUTORETRY);
    Wr (D, GRXTHRCFG, (Rd (D, GRXTHRCFG) & ~GRXTHRCFG_MASK) | GRXTHRCFG_VALUE);
  }
  Reg  = Rd (D, GUSB2PHYCFG);
  Reg &= ~(GUSB2PHYCFG_SUSPHY | GUSB2PHYCFG_ENBLSLPM | GUSB2PHYCFG_TRDTIM_MASK);
  Reg |= GUSB2PHYCFG_TRDTIM (9);
  if (D->Release <= 0x210a) {
    Reg |= GUSB2PHYCFG_PHYIF;
    Wr (D, GUSB3PIPECTL, (Rd (D, GUSB3PIPECTL) & ~GUSB3PIPECTL_DELAY_MASK) | GUSB3PIPECTL_DELAY (1));
  }
  Wr (D, GUSB2PHYCFG, Reg);
  And (D, GUSB3PIPECTL, ~GUSB3PIPECTL_SUSPSSPHY);

  Wr (D, DCFG, DCFG_NUMP (3) | DCFG_PERFRINT (2) | DCFG_DEVSPD_HS);

  RtlZeroMemory (D->EventBuffer, EVENT_BUFFER_SIZE);
  D->EventIndex = 0;
  Wr (D, GEVNTADRHI, 0);
  Wr (D, GEVNTADRLO, PoolPhys (D, D->EventBuffer));
  Wr (D, GEVNTSIZ, GEVNTSIZ_INTMASK | EVENT_BUFFER_SIZE);
  Wr (D, GEVNTCOUNT, Rd (D, GEVNTCOUNT));
  Wr (D, GEVNTSIZ, EVENT_BUFFER_SIZE);
  Wr (D, DEVTEN, DEVTEN_ULSTCNG | DEVTEN_CONNECTDONE | DEVTEN_USBRST | DEVTEN_DISCONN);

  DepCmd (D, 0, DEPCMD_DEPSTARTCFG, 0, 0, 0);
  DepCmd (D, 0, DEPCMD_DEPCFG, DEPCFG0_MPS (EP0_MPS), DEPCFG1_XFERNRDYEN | DEPCFG1_XFERCMPLEN, 0);
  DepCmd (D, 1, DEPCMD_DEPCFG, DEPCFG0_MPS (EP0_MPS),
          DEPCFG1_EPDIR | DEPCFG1_XFERNRDYEN | DEPCFG1_XFERCMPLEN, 0);
  DepCmd (D, 0, DEPCMD_DEPXFERCFG, 1, 0, 0);
  DepCmd (D, 1, DEPCMD_DEPXFERCFG, 1, 0, 0);
  Wr (D, DALEPENA, 0x3);

  RtlZeroMemory (D->In, sizeof (D->In));
  RtlZeroMemory (D->Out, sizeof (D->Out));
  RtlZeroMemory (D->Halted, sizeof (D->Halted));
  D->EpsConfigured = FALSE;
  D->Configuration = 0;
  Ep0ArmSetup (D);

  OsLog ("exynosdwc3: DWC3 release %x up", D->Release);
  return STATUS_SUCCESS;
}

static VOID
CoreStop (PDEVICE_CONTEXT D)
{
  ULONG  Try;

  And (D, DCTL, ~DCTL_RUNSTOP);
  for (Try = 0; Try < 10; Try++) {
    Wr (D, GEVNTCOUNT, Rd (D, GEVNTCOUNT) & 0xFFFC);
    if (PollSet (D, DSTS, DSTS_DEVCTRLHLT, 100)) {
      break;
    }
  }
  Wr (D, DEVTEN, 0);
  Wr (D, GEVNTSIZ, GEVNTSIZ_INTMASK);
  Wr (D, GEVNTCOUNT, Rd (D, GEVNTCOUNT) & 0xFFFC);
  Wr (D, GEVNTADRLO, 0);
}

//
// ---------------------------------------------------------------------------
// Interrupt: mask and hand over to the DPC.
// ---------------------------------------------------------------------------
//

BOOLEAN
EvtIsr (WDFINTERRUPT Interrupt, ULONG MessageId)
{
  PDEVICE_CONTEXT  D;

  UNREFERENCED_PARAMETER (MessageId);

  D = GetDeviceContext (WdfInterruptGetDevice (Interrupt));
  if (!D->Running || ((Rd (D, GEVNTCOUNT) & 0xFFFC) == 0)) {
    return FALSE;
  }
  Wr (D, GEVNTSIZ, GEVNTSIZ_INTMASK | EVENT_BUFFER_SIZE);
  WdfInterruptQueueDpcForIsr (Interrupt);
  return TRUE;
}

//
// ---------------------------------------------------------------------------
// Function side (UsbLogDxe.c), at DISPATCH_LEVEL.
// ---------------------------------------------------------------------------
//

static BOOLEAN
Transfer (PDEVICE_CONTEXT D, ULONG Ep, BOOLEAN In, PUCHAR Buffer, ULONG Length)
{
  XFER     *X;
  BOOLEAN  Started;

  Started = FALSE;
  WdfSpinLockAcquire (D->Lock);
  X = In ? &D->In[Ep] : &D->Out[Ep];
  if (D->Running && (D->Configuration != 0) && !X->Active && !X->Started) {
    RtlZeroMemory (X, sizeof (*X));
    X->Active = TRUE;
    X->Buffer = Buffer;
    X->Length = Length;
    Started   = StartTransfer (D, Ep, In);
    if (!Started) {
      X->Active = FALSE;
    }
  }
  WdfSpinLockRelease (D->Lock);
  return Started;
}

/**
  String descriptors, like UsbLogDxe; anything else stalls.
**/
static VOID
HandleSetup (PDEVICE_CONTEXT D, const USB_SETUP *Req)
{
  const char  *Str;
  ULONG       Size;

  WdfSpinLockAcquire (D->Lock);
  if ((Req->Request != REQ_GET_DESCRIPTOR) || ((Req->Value >> 8) != DESC_STRING)) {
    Ep0Stall (D);
    WdfSpinLockRelease (D->Lock);
    return;
  }

  switch (Req->Value & 0xFF) {
    case 0:  Str = NULL;                                  break;
    case 1:  Str = "Samsung";                             break;
    case 2:  Str = "Galaxy Note Pro 12.2 Windows log";    break;
    default: Str = "SM-P900";                             break;
  }

  D->Ep0Reply[1] = DESC_STRING;
  if (Str == NULL) {
    D->Ep0Reply[2] = 0x09;
    D->Ep0Reply[3] = 0x04;
    Size = 4;
  } else {
    for (Size = 2; (*Str != 0) && (Size + 2 <= sizeof (D->Ep0Reply)); Str++) {
      D->Ep0Reply[Size++] = (UCHAR)*Str;
      D->Ep0Reply[Size++] = 0;
    }
  }
  D->Ep0Reply[0] = (UCHAR)Size;
  Ep0Reply (D, D->Ep0Reply, Size, Req->Length);
  WdfSpinLockRelease (D->Lock);
}

static VOID
LogKick (PDEVICE_CONTEXT D)
{
  ULONG  Head;
  ULONG  Pending;
  ULONG  Index;

  if (D->TxBusy || (mOsLog == NULL) || (mOsLog->Magic != OS_LOG_MAGIC)) {
    return;
  }

  Head = mOsLog->Head;
  if (Head - D->ReadPos > OS_LOG_DATA_SIZE) {
    D->ReadPos = Head - OS_LOG_DATA_SIZE;
  }
  Pending = min (Head - D->ReadPos, LOG_CHUNK);
  if (Pending == 0) {
    return;
  }
  // Every transfer ends in a short packet, so the host read returns.
  if ((Pending % BULK_MPS) == 0) {
    Pending--;
  }
  for (Index = 0; Index < Pending; Index++) {
    D->Tx[Index] = mOsLog->Data[(D->ReadPos + Index) % OS_LOG_DATA_SIZE];
  }

  if (Transfer (D, LOG_EP, TRUE, D->Tx, Pending)) {
    D->TxEnd  = D->ReadPos + Pending;
    D->TxBusy = TRUE;
  }
}

static VOID
DbgPostRx (PDEVICE_CONTEXT D)
{
  if (!D->DbgRxPosted && Transfer (D, DBG_EP, FALSE, D->DbgRx, sizeof (D->DbgRx))) {
    D->DbgRxPosted = TRUE;
  }
}

static PVOID
MapPhysical (ULONG Address, ULONG Length, PULONG Offset)
{
  PHYSICAL_ADDRESS  Phys;

  Phys.QuadPart = Address & ~(PAGE_SIZE - 1);
  *Offset       = Address & (PAGE_SIZE - 1);
  return MmMapIoSpace (Phys, *Offset + Length, MmNonCached);
}

static VOID
DbgHandle (PDEVICE_CONTEXT D, ULONG Length)
{
  USBDBG_HEADER  Req;
  USBDBG_REPLY   *Rsp;
  PUCHAR         Data;
  PUCHAR         Va;
  ULONG          Offset;
  ULONG          Index;

  Rsp  = (USBDBG_REPLY *)D->DbgTx;
  Data = D->DbgTx + sizeof (USBDBG_REPLY);
  RtlZeroMemory (Rsp, sizeof (*Rsp));
  Rsp->Magic  = USBDBG_RSP_MAGIC;
  Rsp->Status = USBDBG_OK;

  RtlCopyMemory (&Req, D->DbgRx, min (Length, sizeof (Req)));
  if ((Length < sizeof (Req)) || (Req.Magic != USBDBG_REQ_MAGIC)) {
    Rsp->Status = USBDBG_EBADCMD;
  } else {
    switch (Req.Command) {
      case USBDBG_PING:
        RtlCopyMemory (Data, "PONG", 4);
        Rsp->Length = 4;
        break;

      case USBDBG_READ:
        if ((Req.Arg1 == 0) || (Req.Arg1 > USBDBG_MAX_READ)) {
          Rsp->Status = USBDBG_EBADARG;
          break;
        }
        Va = MapPhysical (Req.Arg0, Req.Arg1, &Offset);
        if (Va == NULL) {
          Rsp->Status = USBDBG_EBADARG;
          break;
        }
        if (((Req.Arg0 | Req.Arg1) & 3) == 0) {
          for (Index = 0; Index < Req.Arg1; Index += 4) {
            *(PULONG)(Data + Index) = READ_REGISTER_ULONG ((PULONG)(Va + Offset + Index));
          }
        } else {
          for (Index = 0; Index < Req.Arg1; Index++) {
            Data[Index] = READ_REGISTER_UCHAR (Va + Offset + Index);
          }
        }
        MmUnmapIoSpace (Va, Offset + Req.Arg1);
        Rsp->Length = Req.Arg1;
        break;

      case USBDBG_WRITE32:
        if ((Req.Arg0 & 3) != 0) {
          Rsp->Status = USBDBG_EBADARG;
          break;
        }
        Va = MapPhysical (Req.Arg0, 4, &Offset);
        if (Va == NULL) {
          Rsp->Status = USBDBG_EBADARG;
          break;
        }
        WRITE_REGISTER_ULONG ((PULONG)(Va + Offset), Req.Arg1);
        MmUnmapIoSpace (Va, Offset + 4);
        break;

      case USBDBG_RESET:
        // Windows reboots through the firmware's ResetSystem: normal mode only.
        D->PendingReset = TRUE;
        break;

      default:
        Rsp->Status = USBDBG_EBADCMD;
        break;
    }
  }

  if (Transfer (D, DBG_EP, TRUE, D->DbgTx, sizeof (USBDBG_REPLY) + Rsp->Length)) {
    D->DbgTxBusy = TRUE;
  } else if (D->PendingReset) {
    WdfWorkItemEnqueue (D->ResetWork);
  }
}

static VOID
Poll (PDEVICE_CONTEXT D)
{
  USB_EVENT  Ev;
  BOOLEAN    Have;

  for ( ; ; ) {
    WdfSpinLockAcquire (D->Lock);
    if (D->Running) {
      ServiceEvents (D);
      Wr (D, GEVNTSIZ, EVENT_BUFFER_SIZE);        // unmask (the ISR masked it)
    }
    Have = (D->QueueTail != D->QueueHead);
    if (Have) {
      Ev           = D->Queue[D->QueueTail];
      D->QueueTail = (D->QueueTail + 1) % EVENT_QUEUE;
    }
    WdfSpinLockRelease (D->Lock);
    if (!Have) {
      break;
    }

    switch (Ev.Type) {
      case EvReset:
        D->TxBusy      = FALSE;
        D->DbgTxBusy   = FALSE;
        D->DbgRxPosted = FALSE;
        break;
      case EvSetup:
        HandleSetup (D, &Ev.Setup);
        break;
      case EvTxDone:
        if (Ev.Ep == LOG_EP) {
          D->ReadPos = D->TxEnd;
          D->TxBusy  = FALSE;
        } else if (Ev.Ep == DBG_EP) {
          D->DbgTxBusy = FALSE;
          if (D->PendingReset) {
            WdfWorkItemEnqueue (D->ResetWork);
          }
        }
        break;
      case EvRxDone:
        if (Ev.Ep == DBG_EP) {
          D->DbgRxPosted = FALSE;
          if (!D->DbgTxBusy) {
            DbgHandle (D, Ev.Bytes);
          }
        }
        break;
    }
  }

  DbgPostRx (D);
  LogKick (D);
}

VOID
EvtDpc (WDFINTERRUPT Interrupt, WDFOBJECT AssociatedObject)
{
  UNREFERENCED_PARAMETER (AssociatedObject);
  Poll (GetDeviceContext (WdfInterruptGetDevice (Interrupt)));
}

VOID
EvtTimer (WDFTIMER Timer)
{
  Poll (GetDeviceContext ((WDFDEVICE)WdfTimerGetParentObject (Timer)));
}

VOID
EvtResetWork (WDFWORKITEM WorkItem)
{
  UNREFERENCED_PARAMETER (WorkItem);
  OsLog ("exynosdwc3: reboot requested over USB");
  HalReturnToFirmware (HalRebootRoutine);
}

//
// ---------------------------------------------------------------------------
// PnP / power.
// ---------------------------------------------------------------------------
//

NTSTATUS
EvtD0Entry (WDFDEVICE Device, WDF_POWER_DEVICE_STATE PreviousState)
{
  PDEVICE_CONTEXT  D;
  NTSTATUS         Status;

  UNREFERENCED_PARAMETER (PreviousState);

  //
  // KMDF connects the interrupt after D0Entry: nothing else touches the
  // controller yet.
  //
  D = GetDeviceContext (Device);
  D->QueueHead = D->QueueTail = 0;
  Status = CoreInit (D);
  if (NT_SUCCESS (Status)) {
    D->Running = TRUE;
    Or (D, DCTL, DCTL_RUNSTOP);                  // connect
    if (!PollClear (D, DSTS, DSTS_DEVCTRLHLT, 1000)) {
      OsLog ("exynosdwc3: controller did not start");
    }
  }
  WdfTimerStart (D->Timer, WDF_REL_TIMEOUT_IN_MS (POLL_MS));
  return Status;
}

NTSTATUS
EvtD0Exit (WDFDEVICE Device, WDF_POWER_DEVICE_STATE TargetState)
{
  PDEVICE_CONTEXT  D;

  UNREFERENCED_PARAMETER (TargetState);

  D = GetDeviceContext (Device);
  WdfTimerStop (D->Timer, TRUE);
  WdfSpinLockAcquire (D->Lock);
  if (D->Running) {
    D->Running = FALSE;
    CoreStop (D);
  }
  WdfSpinLockRelease (D->Lock);
  return STATUS_SUCCESS;
}

NTSTATUS
EvtPrepareHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
{
  PDEVICE_CONTEXT                  D;
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  PHYSICAL_ADDRESS                 Low;
  PHYSICAL_ADDRESS                 High;
  PHYSICAL_ADDRESS                 Skip;
  ULONG                            Index;
  ULONG                            Count;
  ULONG                            Phys;
  ULONG_PTR                        Gsi;
  PUCHAR                           Ptr;

  D     = GetDeviceContext (Device);
  Gsi   = 0;
  Count = WdfCmResourceListGetCount (ResourcesTranslated);
  for (Index = 0; Index < Count; Index++) {
    Desc = WdfCmResourceListGetDescriptor (ResourcesTranslated, Index);
    if ((Desc->Type == CmResourceTypeMemory) && (D->Base == NULL)) {
      D->Size = Desc->u.Memory.Length;
      D->Base = MmMapIoSpace (Desc->u.Memory.Start, D->Size, MmNonCached);
      if (D->Base == NULL) {
        return STATUS_INSUFFICIENT_RESOURCES;
      }
    } else if (Desc->Type == CmResourceTypeInterrupt) {
      Gsi = WdfCmResourceListGetDescriptor (ResourcesRaw, Index)->u.Interrupt.Vector;
    }
  }
  if (D->Base == NULL) {
    OsLog ("exynosdwc3: no memory resource");
    return STATUS_DEVICE_CONFIGURATION_ERROR;
  }
  OsLog ("exynosdwc3: controller mapped, GSI %Iu, %lu resources", Gsi, Count);

#if LOAD_TEST
  KeBugCheckEx (USB_BUGCHECK,
                (ULONG_PTR)WdfCmResourceListGetDescriptor (ResourcesRaw, 0)->u.Memory.Start.QuadPart,
                Gsi, Count, 0);
#endif

  //
  // DMA pool: physically contiguous, uncached, below 4 GiB (32-bit TRBs).
  //
  if (D->Pool == NULL) {
    Low.QuadPart  = 0;
    High.QuadPart = 0xFFFFFFFF;
    Skip.QuadPart = 0;
    D->Pool = MmAllocateContiguousMemorySpecifyCache (POOL_SIZE, Low, High, Skip, MmNonCached);
    if (D->Pool == NULL) {
      OsLog ("exynosdwc3: no DMA memory");
      return STATUS_INSUFFICIENT_RESOURCES;
    }
    RtlZeroMemory (D->Pool, POOL_SIZE);
    D->PoolPhys = (ULONG)MmGetPhysicalAddress (D->Pool).QuadPart;

    Ptr            = D->Pool;
    D->EventBuffer = (PULONG)Ptr;
    Ptr           += EVENT_BUFFER_SIZE;
    for (Phys = 0; Phys < PHYS_EPS; Phys++) {
      D->Trb[Phys] = (DWC3_TRB *)(Ptr + Phys * TRB_SLOT);
    }
    D->SetupBuffer = Ptr + PHYS_EPS * TRB_SLOT;
    Ptr           += PAGE_SIZE;
    D->Bounce[0]   = Ptr;
    D->Bounce[1]   = Ptr + EP0_BUFFER;
    Ptr           += PAGE_SIZE;
    for (Phys = 2; Phys < PHYS_EPS; Phys++) {
      D->Bounce[Phys] = Ptr;
      Ptr            += BOUNCE_SIZE;
    }
  }
  return STATUS_SUCCESS;
}

NTSTATUS
EvtReleaseHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesTranslated)
{
  PDEVICE_CONTEXT  D;

  UNREFERENCED_PARAMETER (ResourcesTranslated);

  D = GetDeviceContext (Device);
  if (D->Pool != NULL) {
    MmFreeContiguousMemorySpecifyCache (D->Pool, POOL_SIZE, MmNonCached);
    D->Pool = NULL;
  }
  if (D->Base != NULL) {
    MmUnmapIoSpace (D->Base, D->Size);
    D->Base = NULL;
  }
  return STATUS_SUCCESS;
}

NTSTATUS
EvtDeviceAdd (WDFDRIVER Driver, PWDFDEVICE_INIT DeviceInit)
{
  WDF_PNPPOWER_EVENT_CALLBACKS  PnpPower;
  WDF_OBJECT_ATTRIBUTES         Attributes;
  WDF_INTERRUPT_CONFIG          InterruptConfig;
  WDF_TIMER_CONFIG              TimerConfig;
  WDF_WORKITEM_CONFIG           WorkConfig;
  PDEVICE_CONTEXT               D;
  WDFDEVICE                     Device;
  NTSTATUS                      Status;

  UNREFERENCED_PARAMETER (Driver);

  WDF_PNPPOWER_EVENT_CALLBACKS_INIT (&PnpPower);
  PnpPower.EvtDevicePrepareHardware = EvtPrepareHardware;
  PnpPower.EvtDeviceReleaseHardware = EvtReleaseHardware;
  PnpPower.EvtDeviceD0Entry         = EvtD0Entry;
  PnpPower.EvtDeviceD0Exit          = EvtD0Exit;
  WdfDeviceInitSetPnpPowerEventCallbacks (DeviceInit, &PnpPower);

  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, DEVICE_CONTEXT);
  Status = WdfDeviceCreate (&DeviceInit, &Attributes, &Device);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  D = GetDeviceContext (Device);

  WDF_OBJECT_ATTRIBUTES_INIT (&Attributes);
  Attributes.ParentObject = Device;
  Status = WdfSpinLockCreate (&Attributes, &D->Lock);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  WDF_INTERRUPT_CONFIG_INIT (&InterruptConfig, EvtIsr, EvtDpc);
  Status = WdfInterruptCreate (Device, &InterruptConfig, WDF_NO_OBJECT_ATTRIBUTES, &D->Interrupt);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  WDF_TIMER_CONFIG_INIT_PERIODIC (&TimerConfig, EvtTimer, POLL_MS);
  WDF_OBJECT_ATTRIBUTES_INIT (&Attributes);
  Attributes.ParentObject = Device;
  Status = WdfTimerCreate (&TimerConfig, &Attributes, &D->Timer);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  WDF_WORKITEM_CONFIG_INIT (&WorkConfig, EvtResetWork);
  WDF_OBJECT_ATTRIBUTES_INIT (&Attributes);
  Attributes.ParentObject = Device;
  return WdfWorkItemCreate (&WorkConfig, &Attributes, &D->ResetWork);
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG  Config;

  OsLogInit ();
  OsLog ("exynosdwc3: DriverEntry");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  return WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, WDF_NO_HANDLE);
}
