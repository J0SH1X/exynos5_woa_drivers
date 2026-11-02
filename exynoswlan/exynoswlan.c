/** @file
  v1awifi (SM-P900) Wi-Fi for Windows RT 8.1: Broadcom BCM4354 (FullMAC) on
  dw_mmc channel 1, as an NDIS 6.30 miniport.

  First version: the adapter is an 802.3 (Ethernet) adapter.  The firmware
  is FullMAC - it scans, associates and does the WPA2 handshake itself - so
  the driver joins the network named by the "Ssid" / "Passphrase" values
  (Device Manager, Advanced) and moves Ethernet frames.  No Native Wi-Fi
  (network list, WLAN API) yet.

  The Broadcom side (SDIO, backplane, firmware download, SDPCM, BCDC) is the
  UEFI tree's Drivers/BcmWlan/BcmSdio.c, built unchanged (bcmsdio.c,
  dwmci.c, MmcEnv.h).  It is strictly one caller at a time, so one system
  thread owns the bus.  The chip signals frames with its in-band SDIO
  interrupt (DAT1), which dw_mmc1 raises on GSI 108: the ISR masks it, the
  DPC wakes the thread, the thread drains the chip, then acknowledges and
  unmasks.  Sends wake the thread too.  A 1 s timeout is only a safety
  net for a lost interrupt.

  Board (board-universal5420-wlan.c, board-universal5420-mmc.c): WLAN_EN =
  GPY7(7); SDIO pins GPC1(0,1,3-6) function 2 (GPC1(2) is GPS_EN, not
  touched); sclk_dwmci1 from CPLL (CLK_SRC_FSYS[14:12], CLK_DIV_FSYS1[19:10],
  CLK_GATE_IP_FSYS bit 13).  The clock and GPIO registers are mapped here
  without being claimed (exynosgpio owns the GPIO window in ACPI); only the
  WLAN bits are changed.

  ACPI: _HID EXYN5009, Memory32Fixed 0x12210000 + 0x1000 (dw_mmc1),
  Interrupt 108 (dw_mmc1, SPI 76).
  Firmware: %SystemRoot%\System32\drivers\bcmdhd_sta.bin, nvram_net.txt.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#define NDIS_MINIPORT_DRIVER  1
#define NDIS630_MINIPORT      1
#define OSLOG_GLOBAL
#include "MmcEnv.h"
#include <ndis.h>

#include "../../uefi/Exynos5420Pkg/Drivers/BcmWlan/BcmSdio.h"

#define WLAN_TAG            'nlwE'
#define SAFETY_POLL_MS      1000        // with the card interrupt working
#define FALLBACK_POLL_MS    20          // until the first card interrupt
#define DRAIN_ROUNDS        16
#define RX_FRAME_MAX        1600
#define TX_QUEUE_MAX        64

//
// Physical registers the driver maps itself.
//
#define CMU_PAGE            0x10020000         // CMU_BASE + 0x10000
#define CLK_CPLL_CON0       0x120
#define CLK_SRC_FSYS        0x244
#define CLK_SRC_MASK_FSYS   0x340
#define CLK_DIV_FSYS1       0x54C
#define CLK_GATE_IP_FSYS    0x944
#define GPIO_RT_PAGE        0x13400000          // GPY7 at +0x000
#define GPIO_T_PAGE         0x13410000          // GPC1 at +0x020
#define GPY7_CON            0x000
#define GPY7_DAT            0x004
#define GPC1_CON            0x020
#define GPC1_PUD            0x028
#define GPC1_DRV            0x02C
#define WLAN_EN_PIN         7
#define DWMCI_SDR_TIMING    0x03040000U

typedef struct {
  NDIS_HANDLE        Handle;
  PUCHAR             Mmc;
  PUCHAR             Cmu;
  PUCHAR             GpioRt;
  PUCHAR             GpioT;
  ULONG              MmcPhys;

  BCM_SDIO           *Bcm;
  UCHAR              Mac[6];
  BOOLEAN            Up;               // firmware running
  BOOLEAN            LinkUp;
  BOOLEAN            LinkChanged;
  BOOLEAN            Paused;

  PETHREAD           Thread;
  KEVENT             Wake;
  BOOLEAN            Stop;
  NDIS_HANDLE        Interrupt;
  volatile LONG      IrqPending;
  ULONG              Interrupts;

  NDIS_SPIN_LOCK     Lock;
  PNET_BUFFER_LIST   TxHead;
  PNET_BUFFER_LIST   TxTail;
  ULONG              TxQueued;

  NDIS_HANDLE        NblPool;
  PNET_BUFFER_LIST   RxNbl;
  PMDL               RxMdl;
  PUCHAR             RxBuffer;

  ULONG              PacketFilter;
  ULONG              Lookahead;
  ULONG64            TxFrames;
  ULONG64            RxFrames;
  ULONG64            TxErrors;

  CHAR               Ssid[33];
  CHAR               Passphrase[65];
  UCHAR              TxFrame[RX_FRAME_MAX + 8];
} ADAPTER, *PADAPTER;

static NDIS_HANDLE  mDriverHandle;

static const NDIS_OID mSupportedOids[] = {
  OID_GEN_SUPPORTED_LIST, OID_GEN_HARDWARE_STATUS, OID_GEN_MEDIA_SUPPORTED, OID_GEN_MEDIA_IN_USE,
  OID_GEN_MAXIMUM_FRAME_SIZE, OID_GEN_MAXIMUM_TOTAL_SIZE, OID_GEN_LINK_SPEED, OID_GEN_TRANSMIT_BLOCK_SIZE,
  OID_GEN_RECEIVE_BLOCK_SIZE, OID_GEN_VENDOR_ID, OID_GEN_VENDOR_DESCRIPTION, OID_GEN_VENDOR_DRIVER_VERSION,
  OID_GEN_CURRENT_PACKET_FILTER, OID_GEN_CURRENT_LOOKAHEAD, OID_GEN_DRIVER_VERSION, OID_GEN_MAXIMUM_SEND_PACKETS,
  OID_GEN_MEDIA_CONNECT_STATUS, OID_GEN_XMIT_OK, OID_GEN_RCV_OK, OID_GEN_XMIT_ERROR, OID_GEN_RCV_ERROR,
  OID_GEN_RCV_NO_BUFFER, OID_802_3_PERMANENT_ADDRESS, OID_802_3_CURRENT_ADDRESS, OID_802_3_MULTICAST_LIST,
  OID_802_3_MAXIMUM_LIST_SIZE, OID_802_3_RCV_ERROR_ALIGNMENT, OID_802_3_XMIT_ONE_COLLISION,
  OID_802_3_XMIT_MORE_COLLISIONS, OID_GEN_STATISTICS,
};

MINIPORT_INITIALIZE                mpInitialize;
MINIPORT_HALT                      mpHalt;
MINIPORT_UNLOAD                    mpUnload;
MINIPORT_PAUSE                     mpPause;
MINIPORT_RESTART                   mpRestart;
MINIPORT_OID_REQUEST               mpOidRequest;
MINIPORT_SEND_NET_BUFFER_LISTS     mpSend;
MINIPORT_RETURN_NET_BUFFER_LISTS   mpReturn;
MINIPORT_CANCEL_SEND               mpCancelSend;
MINIPORT_DEVICE_PNP_EVENT_NOTIFY   mpPnpEvent;
MINIPORT_SHUTDOWN                  mpShutdown;
MINIPORT_CANCEL_OID_REQUEST        mpCancelOid;
MINIPORT_ISR                       mpIsr;
MINIPORT_INTERRUPT_DPC             mpDpc;
MINIPORT_ENABLE_INTERRUPT          mpEnableInterrupt;
MINIPORT_DISABLE_INTERRUPT         mpDisableInterrupt;
MINIPORT_SYNCHRONIZE_INTERRUPT     SyncInterruptDone;
DRIVER_INITIALIZE                  DriverEntry;

//
// ---------------------------------------------------------------------------
// Board.
// ---------------------------------------------------------------------------
//

static ULONG
Reg (PUCHAR Base, ULONG Off)
{
  return READ_REGISTER_ULONG ((PULONG)(Base + Off));
}

static VOID
RegWrite (PUCHAR Base, ULONG Off, ULONG Value)
{
  WRITE_REGISTER_ULONG ((PULONG)(Base + Off), Value);
}

static VOID
WlanPower (PADAPTER A, BOOLEAN On)
{
  ULONG  Dat;

  Dat = Reg (A->GpioRt, GPY7_DAT);
  RegWrite (A->GpioRt, GPY7_DAT, On ? (Dat | (1u << WLAN_EN_PIN)) : (Dat & ~(1u << WLAN_EN_PIN)));
  RegWrite (A->GpioRt, GPY7_CON, (Reg (A->GpioRt, GPY7_CON) & ~(0xFu << (WLAN_EN_PIN * 4))) | (1u << (WLAN_EN_PIN * 4)));
}

static VOID
PinsInit (PADAPTER A)
{
  ULONG  Pin;

  for (Pin = 0; Pin <= 6; Pin++) {
    if (Pin == 2) {
      continue;                         // GPS_EN
    }
    RegWrite (A->GpioT, GPC1_CON, (Reg (A->GpioT, GPC1_CON) & ~(0xFu << (Pin * 4))) | (2u << (Pin * 4)));
    RegWrite (A->GpioT, GPC1_PUD, (Reg (A->GpioT, GPC1_PUD) & ~(3u << (Pin * 2))) | ((Pin == 0) ? 0u : (3u << (Pin * 2))));
    RegWrite (A->GpioT, GPC1_DRV, (Reg (A->GpioT, GPC1_DRV) & ~(3u << (Pin * 2))) | (1u << (Pin * 2)));
  }
}

/**
  sclk_dwmci1 from CPLL / 1; returns the CIU clock (CLKSEL divides by 4).
**/
static ULONG
ClockInit (PADAPTER A)
{
  ULONG  Con;
  ULONG  Rate;

  RegWrite (A->Cmu, CLK_GATE_IP_FSYS, Reg (A->Cmu, CLK_GATE_IP_FSYS) | (1u << 13));
  RegWrite (A->Cmu, CLK_SRC_MASK_FSYS, Reg (A->Cmu, CLK_SRC_MASK_FSYS) | (1u << 12));
  Con = Reg (A->Cmu, CLK_CPLL_CON0);
  if (((Con & 0x80000000u) == 0) || (((Con >> 8) & 0x3F) == 0)) {
    return 0;
  }
  Rate = (ULONG)((24000000ULL * ((Con >> 16) & 0x3FF)) / (((Con >> 8) & 0x3F) << (Con & 7)));
  RegWrite (A->Cmu, CLK_SRC_FSYS, (Reg (A->Cmu, CLK_SRC_FSYS) & ~(7u << 12)) | (1u << 12));
  RegWrite (A->Cmu, CLK_DIV_FSYS1, Reg (A->Cmu, CLK_DIV_FSYS1) & ~(0x3FFu << 10));
  return Rate / 4;
}

//
// ---------------------------------------------------------------------------
// Files and configuration.
// ---------------------------------------------------------------------------
//

static NTSTATUS
LoadFile (PCWSTR Path, PUCHAR *Data, ULONG *Size)
{
  UNICODE_STRING             Name;
  OBJECT_ATTRIBUTES          Attr;
  IO_STATUS_BLOCK            Io;
  FILE_STANDARD_INFORMATION  Info;
  HANDLE                     File;
  NTSTATUS                   Status;

  *Data = NULL;
  RtlInitUnicodeString (&Name, Path);
  InitializeObjectAttributes (&Attr, &Name, OBJ_CASE_INSENSITIVE | OBJ_KERNEL_HANDLE, NULL, NULL);
  Status = ZwCreateFile (&File, GENERIC_READ | SYNCHRONIZE, &Attr, &Io, NULL, FILE_ATTRIBUTE_NORMAL, FILE_SHARE_READ,
                         FILE_OPEN, FILE_SYNCHRONOUS_IO_NONALERT | FILE_NON_DIRECTORY_FILE, NULL, 0);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  Status = ZwQueryInformationFile (File, &Io, &Info, sizeof (Info), FileStandardInformation);
  if (NT_SUCCESS (Status) && ((Info.EndOfFile.HighPart != 0) || (Info.EndOfFile.LowPart > 4 * 1024 * 1024))) {
    Status = STATUS_FILE_TOO_LARGE;
  }
  if (NT_SUCCESS (Status)) {
    *Size = Info.EndOfFile.LowPart;
    *Data = ExAllocatePoolWithTag (NonPagedPool, *Size + 8, WLAN_TAG);
    if (*Data == NULL) {
      Status = STATUS_INSUFFICIENT_RESOURCES;
    }
  }
  if (NT_SUCCESS (Status)) {
    RtlZeroMemory (*Data, *Size + 8);
    Status = ZwReadFile (File, NULL, NULL, NULL, &Io, *Data, *Size, NULL, NULL);
  }
  ZwClose (File);
  if (!NT_SUCCESS (Status) && (*Data != NULL)) {
    ExFreePoolWithTag (*Data, WLAN_TAG);
    *Data = NULL;
  }
  return Status;
}

static VOID
ReadString (NDIS_HANDLE Config, PCWSTR Key, PCHAR Out, ULONG OutSize)
{
  NDIS_STRING                     Name;
  PNDIS_CONFIGURATION_PARAMETER   Param;
  NDIS_STATUS                     Status;
  ULONG                           Index;

  Out[0] = '\0';
  NdisInitUnicodeString (&Name, Key);
  NdisReadConfiguration (&Status, &Param, Config, &Name, NdisParameterString);
  if ((Status != NDIS_STATUS_SUCCESS) || (Param->ParameterType != NdisParameterString)) {
    return;
  }
  for (Index = 0; (Index + 1 < OutSize) && (Index < Param->ParameterData.StringData.Length / sizeof (WCHAR)); Index++) {
    Out[Index] = (CHAR)Param->ParameterData.StringData.Buffer[Index];
  }
  Out[Index] = '\0';
}

static VOID
ReadConfig (PADAPTER A)
{
  NDIS_CONFIGURATION_OBJECT  Object;
  NDIS_HANDLE                Config;

  RtlZeroMemory (&Object, sizeof (Object));
  Object.Header.Type     = NDIS_OBJECT_TYPE_CONFIGURATION_OBJECT;
  Object.Header.Revision = NDIS_CONFIGURATION_OBJECT_REVISION_1;
  Object.Header.Size     = NDIS_SIZEOF_CONFIGURATION_OBJECT_REVISION_1;
  Object.NdisHandle      = A->Handle;
  if (NdisOpenConfigurationEx (&Object, &Config) != NDIS_STATUS_SUCCESS) {
    return;
  }
  ReadString (Config, L"Ssid", A->Ssid, sizeof (A->Ssid));
  ReadString (Config, L"Passphrase", A->Passphrase, sizeof (A->Passphrase));
  NdisCloseConfiguration (Config);
  OsLog ("exynoswlan: network \"%s\" (%s)", A->Ssid, (A->Passphrase[0] != '\0') ? "WPA2-PSK" : "open");
}

//
// ---------------------------------------------------------------------------
// Receive (called by BcmSdioPoll() in the bus thread).
// ---------------------------------------------------------------------------
//

static ULONG
Be32 (const UCHAR *P)
{
  return ((ULONG)P[0] << 24) | ((ULONG)P[1] << 16) | ((ULONG)P[2] << 8) | P[3];
}

static VOID
RxHandler (VOID *Context, UINT8 Channel, UINT8 *Payload, UINT32 Length)
{
  PADAPTER  A = Context;
  PUCHAR    E;
  ULONG     Type;
  ULONG     Status;
  USHORT    Flags;

  if (Channel == BCM_CHANNEL_EVENT) {
    if (Length < BCM_EVENT_MSG_OFFSET + 40) {
      return;
    }
    E      = Payload + BCM_EVENT_MSG_OFFSET;
    Flags  = (USHORT)((E[2] << 8) | E[3]);
    Type   = Be32 (E + 4);
    Status = Be32 (E + 8);
    if (Type == WLC_E_LINK) {
      if ((Flags & WLC_EVENT_MSG_LINK) == 0) {
        BcmWpaLinkDown (A->Bcm);        // new handshake on the next association
      }
      if (A->LinkUp != ((Flags & WLC_EVENT_MSG_LINK) != 0)) {
        A->LinkUp      = (Flags & WLC_EVENT_MSG_LINK) != 0;
        A->LinkChanged = TRUE;
      }
      OsLog ("exynoswlan: link %s", A->LinkUp ? "up" : "down");
    } else if ((Type == WLC_E_SET_SSID) || (Type == WLC_E_PSK_SUP) || (Type == WLC_E_DEAUTH_IND) ||
               (Type == WLC_E_DISASSOC_IND)) {
      OsLog ("exynoswlan: event %lu status %lu reason %lu", Type, Status, Be32 (E + 12));
    }
    return;
  }

  if ((Channel != BCM_CHANNEL_DATA) || (Length < 14) || (Length > RX_FRAME_MAX) || A->Paused || !A->LinkUp) {
    return;
  }

  //
  // One frame at a time, indicated with NDIS_RECEIVE_FLAGS_RESOURCES so the
  // buffer is ours again when the call returns.
  //
  RtlCopyMemory (A->RxBuffer, Payload, Length);
  NET_BUFFER_DATA_LENGTH (NET_BUFFER_LIST_FIRST_NB (A->RxNbl)) = Length;
  NET_BUFFER_DATA_OFFSET (NET_BUFFER_LIST_FIRST_NB (A->RxNbl)) = 0;
  NET_BUFFER_CURRENT_MDL_OFFSET (NET_BUFFER_LIST_FIRST_NB (A->RxNbl)) = 0;
  NET_BUFFER_LIST_STATUS (A->RxNbl) = NDIS_STATUS_SUCCESS;
  A->RxNbl->SourceHandle = A->Handle;
  A->RxFrames++;
  NdisMIndicateReceiveNetBufferLists (A->Handle, A->RxNbl, NDIS_DEFAULT_PORT_NUMBER, 1,
                                      NDIS_RECEIVE_FLAGS_RESOURCES);
}

//
// ---------------------------------------------------------------------------
// Interrupt.
// ---------------------------------------------------------------------------
//

BOOLEAN
mpIsr (NDIS_HANDLE Context, PBOOLEAN QueueDefaultInterruptDpc, PULONG TargetProcessors)
{
  PADAPTER  A = Context;

  *QueueDefaultInterruptDpc = FALSE;
  *TargetProcessors         = 0;
  if (!BcmSdioInterruptIsr ((UINT32)(ULONG_PTR)A->Mmc)) {
    return FALSE;
  }
  *QueueDefaultInterruptDpc = TRUE;
  return TRUE;
}

VOID
mpDpc (NDIS_HANDLE Context, PVOID DpcContext, PVOID ReceiveThrottleParameters, PVOID NdisReserved2)
{
  PADAPTER  A = Context;

  UNREFERENCED_PARAMETER (DpcContext);
  UNREFERENCED_PARAMETER (ReceiveThrottleParameters);
  UNREFERENCED_PARAMETER (NdisReserved2);

  InterlockedExchange (&A->IrqPending, 1);
  A->Interrupts++;
  KeSetEvent (&A->Wake, 0, FALSE);
}

VOID
mpEnableInterrupt (NDIS_HANDLE Context)
{
  PADAPTER  A = Context;

  WRITE_REGISTER_ULONG ((PULONG)(A->Mmc + 0x24), READ_REGISTER_ULONG ((PULONG)(A->Mmc + 0x24)) | (1u << 16));
}

VOID
mpDisableInterrupt (NDIS_HANDLE Context)
{
  PADAPTER  A = Context;

  WRITE_REGISTER_ULONG ((PULONG)(A->Mmc + 0x24), READ_REGISTER_ULONG ((PULONG)(A->Mmc + 0x24)) & ~(1u << 16));
}

BOOLEAN
SyncInterruptDone (NDIS_HANDLE Context)
{
  PADAPTER  A = Context;

  BcmSdioInterruptDone (A->Bcm);
  return TRUE;
}

static NDIS_STATUS
RegisterInterrupt (PADAPTER A)
{
  NDIS_MINIPORT_INTERRUPT_CHARACTERISTICS  Chars;
  NDIS_STATUS                              Status;

  RtlZeroMemory (&Chars, sizeof (Chars));
  Chars.Header.Type     = NDIS_OBJECT_TYPE_MINIPORT_INTERRUPT;
  Chars.Header.Revision = NDIS_MINIPORT_INTERRUPT_REVISION_1;
  Chars.Header.Size     = NDIS_SIZEOF_MINIPORT_INTERRUPT_CHARACTERISTICS_REVISION_1;
  Chars.InterruptHandler        = mpIsr;
  Chars.InterruptDpcHandler     = mpDpc;
  Chars.EnableInterruptHandler  = mpEnableInterrupt;
  Chars.DisableInterruptHandler = mpDisableInterrupt;
  Chars.MsiSupported            = FALSE;
  Status = NdisMRegisterInterruptEx (A->Handle, A, &Chars, &A->Interrupt);
  if (Status != NDIS_STATUS_SUCCESS) {
    A->Interrupt = NULL;
    OsLog ("exynoswlan: NdisMRegisterInterruptEx failed: 0x%08lx", Status);
  }
  return Status;
}

//
// ---------------------------------------------------------------------------
// Bus thread.
// ---------------------------------------------------------------------------
//

static VOID
IndicateLink (PADAPTER A)
{
  NDIS_STATUS_INDICATION  Ind;
  NDIS_LINK_STATE         Link;
  BOOLEAN                 Connected;

  RtlZeroMemory (&Link, sizeof (Link));
  Link.Header.Type     = NDIS_OBJECT_TYPE_DEFAULT;
  Link.Header.Revision = NDIS_LINK_STATE_REVISION_1;
  Link.Header.Size     = NDIS_SIZEOF_LINK_STATE_REVISION_1;
  //
  // WPA2: connected once the handshake has installed the keys.
  //
  Connected = A->LinkUp && (!A->Bcm->WpaActive || A->Bcm->WpaDone);
  Link.MediaConnectState = Connected ? MediaConnectStateConnected : MediaConnectStateDisconnected;
  Link.MediaDuplexState  = MediaDuplexStateFull;
  Link.XmitLinkSpeed     = Connected ? 54000000ULL : NDIS_LINK_SPEED_UNKNOWN;
  Link.RcvLinkSpeed      = Link.XmitLinkSpeed;

  RtlZeroMemory (&Ind, sizeof (Ind));
  Ind.Header.Type     = NDIS_OBJECT_TYPE_STATUS_INDICATION;
  Ind.Header.Revision = NDIS_STATUS_INDICATION_REVISION_1;
  Ind.Header.Size     = NDIS_SIZEOF_STATUS_INDICATION_REVISION_1;
  Ind.SourceHandle    = A->Handle;
  Ind.StatusCode      = NDIS_STATUS_LINK_STATE;
  Ind.StatusBuffer    = &Link;
  Ind.StatusBufferSize = sizeof (Link);
  NdisMIndicateStatusEx (A->Handle, &Ind);
}

static PNET_BUFFER_LIST
TakeQueue (PADAPTER A)
{
  PNET_BUFFER_LIST  List;

  NdisAcquireSpinLock (&A->Lock);
  List       = A->TxHead;
  A->TxHead  = NULL;
  A->TxTail  = NULL;
  A->TxQueued = 0;
  NdisReleaseSpinLock (&A->Lock);
  return List;
}

/**
  Every NET_BUFFER of the list as one data frame: a 4-byte BDC header
  (version 2, priority 0), then the 802.3 frame.
**/
static VOID
SendList (PADAPTER A, PNET_BUFFER_LIST List)
{
  PNET_BUFFER_LIST  Nbl;
  PNET_BUFFER_LIST  Next;
  PNET_BUFFER       Nb;
  PVOID             Data;
  ULONG             Length;
  NDIS_STATUS       Status;

  for (Nbl = List; Nbl != NULL; Nbl = Next) {
    Next = NET_BUFFER_LIST_NEXT_NBL (Nbl);
    NET_BUFFER_LIST_NEXT_NBL (Nbl) = NULL;
    Status = A->Paused ? NDIS_STATUS_PAUSED : (A->LinkUp ? NDIS_STATUS_SUCCESS : NDIS_STATUS_MEDIA_DISCONNECTED);
    for (Nb = NET_BUFFER_LIST_FIRST_NB (Nbl); (Nb != NULL) && (Status == NDIS_STATUS_SUCCESS); Nb = NET_BUFFER_NEXT_NB (Nb)) {
      Length = NET_BUFFER_DATA_LENGTH (Nb);
      if ((Length < 14) || (Length > RX_FRAME_MAX)) {
        Status = NDIS_STATUS_INVALID_LENGTH;
        break;
      }
      Data = NdisGetDataBuffer (Nb, Length, A->TxFrame + 4, 1, 0);
      if (Data == NULL) {
        Status = NDIS_STATUS_RESOURCES;
        break;
      }
      if (Data != A->TxFrame + 4) {
        RtlCopyMemory (A->TxFrame + 4, Data, Length);
      }
      A->TxFrame[0] = 2 << 4;           // BDC protocol version 2
      A->TxFrame[1] = 0;                // priority
      A->TxFrame[2] = 0;
      A->TxFrame[3] = 0;                // data offset (words)
      if (EFI_ERROR (BcmSdioSend (A->Bcm, BCM_CHANNEL_DATA, A->TxFrame, Length + 4))) {
        Status = NDIS_STATUS_FAILURE;
        A->TxErrors++;
      } else {
        A->TxFrames++;
      }
    }
    NET_BUFFER_LIST_STATUS (Nbl) = Status;
    NdisMSendNetBufferListsComplete (A->Handle, Nbl, 0);
  }
}

static VOID
BusThread (PVOID Context)
{
  PADAPTER          A = Context;
  LARGE_INTEGER     Timeout;
  PNET_BUFFER_LIST  List;
  EFI_STATUS        Status;
  ULONG             Round;
  ULONG             Before;
  ULONG             Missed = 0;

  //
  // Join, then serve the bus.
  //
  if (A->Ssid[0] != '\0') {
    Status = BcmJoin (A->Bcm, A->Ssid, (A->Passphrase[0] != '\0') ? A->Passphrase : NULL);
    OsLog ("exynoswlan: join \"%s\" - 0x%08Ix", A->Ssid, Status);
  } else {
    OsLog ("exynoswlan: no SSID configured (Device Manager > Advanced)");
  }

  Timeout.QuadPart = -(LONGLONG)SAFETY_POLL_MS * 10000;
  while (!A->Stop) {
    Timeout.QuadPart = -(LONGLONG)((A->Interrupts != 0) ? SAFETY_POLL_MS : FALLBACK_POLL_MS) * 10000;
    KeWaitForSingleObject (&A->Wake, Executive, KernelMode, FALSE, &Timeout);
    if (A->Stop) {
      break;
    }

    //
    // Drain what the chip has (also after the safety timeout), then give
    // the interrupt back if it was what woke us.
    //
    Before = A->Bcm->Frames;
    for (Round = 0; (Round < DRAIN_ROUNDS) && (BcmSdioPoll (A->Bcm) > 0); Round++) {
    }
    if ((A->Bcm->Frames != Before) && (A->IrqPending == 0) && (Missed < 5)) {
      //
      // Frames found by the safety poll, not by an interrupt: what does
      // the controller say?
      //
      Missed++;
      OsLog ("exynoswlan: frames without interrupt (%lu so far): CCCR INTPEND 0x%02x RINTSTS 0x%08lx MINTSTS 0x%08lx INTMASK 0x%08lx CTRL 0x%08lx",
             A->Interrupts, BcmSdioIntPending (A->Bcm), READ_REGISTER_ULONG ((PULONG)(A->Mmc + 0x44)), READ_REGISTER_ULONG ((PULONG)(A->Mmc + 0x40)),
             READ_REGISTER_ULONG ((PULONG)(A->Mmc + 0x24)), READ_REGISTER_ULONG ((PULONG)(A->Mmc + 0x00)));
    }
    if (InterlockedExchange (&A->IrqPending, 0) != 0) {
      NdisMSynchronizeWithInterruptEx (A->Interrupt, 0, SyncInterruptDone, A);
    }
    if (BcmWpaService (A->Bcm)) {
      //
      // The handshake ran, so we are associated, events or not.
      //
      OsLog ("exynoswlan: WPA2 keys installed, link up");
      A->LinkUp      = TRUE;
      A->LinkChanged = TRUE;
    }

    if (A->LinkChanged) {
      A->LinkChanged = FALSE;
      IndicateLink (A);
    }
    List = TakeQueue (A);
    if (List != NULL) {
      SendList (A, List);
    }
  }

  List = TakeQueue (A);
  if (List != NULL) {
    A->Paused = TRUE;
    SendList (A, List);
  }
  PsTerminateSystemThread (STATUS_SUCCESS);
}

//
// ---------------------------------------------------------------------------
// Bring-up.
// ---------------------------------------------------------------------------
//

static NDIS_STATUS
StartChip (PADAPTER A)
{
  PUCHAR      Firmware;
  PUCHAR      Nvram;
  ULONG       FirmwareSize;
  ULONG       NvramSize;
  ULONG       CiuHz;
  CHAR8       Ver[128];
  EFI_STATUS  Status;
  NTSTATUS    Nt;

  Nt = LoadFile (L"\\SystemRoot\\System32\\drivers\\bcmdhd_sta.bin", &Firmware, &FirmwareSize);
  if (!NT_SUCCESS (Nt)) {
    OsLog ("exynoswlan: cannot read bcmdhd_sta.bin: 0x%08lx", Nt);
    return NDIS_STATUS_FILE_NOT_FOUND;
  }
  Nt = LoadFile (L"\\SystemRoot\\System32\\drivers\\nvram_net.txt", &Nvram, &NvramSize);
  if (!NT_SUCCESS (Nt)) {
    OsLog ("exynoswlan: cannot read nvram_net.txt: 0x%08lx", Nt);
    ExFreePoolWithTag (Firmware, WLAN_TAG);
    return NDIS_STATUS_FILE_NOT_FOUND;
  }

  PinsInit (A);
  WlanPower (A, FALSE);
  NdisMSleep (20000);
  WlanPower (A, TRUE);
  NdisMSleep (200000);

  CiuHz  = ClockInit (A);
  Status = (CiuHz != 0) ? BcmSdioAttach (A->Bcm, (UINT32)(ULONG_PTR)A->Mmc, CiuHz) : EFI_DEVICE_ERROR;
  if (!EFI_ERROR (Status)) {
    Status = BcmSdioDownload (A->Bcm, Firmware, FirmwareSize, (CONST CHAR8 *)Nvram, NvramSize);
  }
  ExFreePoolWithTag (Firmware, WLAN_TAG);
  ExFreePoolWithTag (Nvram, WLAN_TAG);
  if (!EFI_ERROR (Status)) {
    Status = BcmSdioBusInit (A->Bcm);
  }
  if (!EFI_ERROR (Status)) {
    RtlZeroMemory (Ver, sizeof (Ver));
    BcmIovarGet (A->Bcm, "ver", Ver, sizeof (Ver) - 1);
    OsLog ("exynoswlan: firmware %s", Ver);
    Status = BcmIovarGet (A->Bcm, "cur_etheraddr", A->Mac, sizeof (A->Mac));
  }
  if (!EFI_ERROR (Status)) {
    OsLog ("exynoswlan: MAC %02x:%02x:%02x:%02x:%02x:%02x", A->Mac[0], A->Mac[1], A->Mac[2], A->Mac[3], A->Mac[4], A->Mac[5]);
    BcmIoctlSetInt (A->Bcm, WLC_SET_PM, 0);
    Status = BcmIoctl (A->Bcm, WLC_UP, TRUE, NULL, 0, 2000);
  }
  if (EFI_ERROR (Status)) {
    OsLog ("exynoswlan: chip bring-up failed: 0x%08Ix", Status);
    BcmLogConsole (A->Bcm);
    WlanPower (A, FALSE);
    return NDIS_STATUS_ADAPTER_NOT_FOUND;
  }
  A->Up = TRUE;
  return NDIS_STATUS_SUCCESS;
}

static VOID
FreeAdapter (PADAPTER A)
{
  if (A->RxNbl != NULL) {
    NdisFreeNetBufferList (A->RxNbl);
  }
  if (A->RxMdl != NULL) {
    NdisFreeMdl (A->RxMdl);
  }
  if (A->RxBuffer != NULL) {
    ExFreePoolWithTag (A->RxBuffer, WLAN_TAG);
  }
  if (A->NblPool != NULL) {
    NdisFreeNetBufferListPool (A->NblPool);
  }
  if (A->Mmc != NULL) {
    MmUnmapIoSpace (A->Mmc, PAGE_SIZE);
  }
  if (A->Cmu != NULL) {
    MmUnmapIoSpace (A->Cmu, PAGE_SIZE);
  }
  if (A->GpioRt != NULL) {
    MmUnmapIoSpace (A->GpioRt, PAGE_SIZE);
  }
  if (A->GpioT != NULL) {
    MmUnmapIoSpace (A->GpioT, PAGE_SIZE);
  }
  if (A->Bcm != NULL) {
    ExFreePoolWithTag (A->Bcm, WLAN_TAG);
  }
  NdisFreeSpinLock (&A->Lock);
  ExFreePoolWithTag (A, WLAN_TAG);
}

static PUCHAR
MapPage (ULONG Phys)
{
  PHYSICAL_ADDRESS  P;

  P.QuadPart = Phys;
  return MmMapIoSpace (P, PAGE_SIZE, MmNonCached);
}

NDIS_STATUS
mpInitialize (NDIS_HANDLE Handle, NDIS_HANDLE DriverContext, PNDIS_MINIPORT_INIT_PARAMETERS Params)
{
  NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES  Reg;
  NDIS_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES       Gen;
  NET_BUFFER_LIST_POOL_PARAMETERS                Pool;
  PCM_PARTIAL_RESOURCE_DESCRIPTOR                Desc;
  OBJECT_ATTRIBUTES                              Attr;
  HANDLE                                         ThreadHandle;
  PADAPTER                                       A;
  ULONG                                          Index;
  NDIS_STATUS                                    Status;
  NTSTATUS                                       Nt;

  UNREFERENCED_PARAMETER (DriverContext);

  A = ExAllocatePoolWithTag (NonPagedPool, sizeof (*A), WLAN_TAG);
  if (A == NULL) {
    return NDIS_STATUS_RESOURCES;
  }
  RtlZeroMemory (A, sizeof (*A));
  A->Handle  = Handle;
  A->Paused  = TRUE;
  A->Lookahead = RX_FRAME_MAX;
  NdisAllocateSpinLock (&A->Lock);
  KeInitializeEvent (&A->Wake, SynchronizationEvent, FALSE);

  //
  // dw_mmc1 from ACPI; clock and GPIO pages mapped directly.
  //
  if (Params->AllocatedResources != NULL) {
    for (Index = 0; Index < Params->AllocatedResources->Count; Index++) {
      Desc = &Params->AllocatedResources->PartialDescriptors[Index];
      if ((Desc->Type == CmResourceTypeMemory) && (A->Mmc == NULL)) {
        A->MmcPhys = Desc->u.Memory.Start.LowPart;
        A->Mmc     = MmMapIoSpace (Desc->u.Memory.Start, PAGE_SIZE, MmNonCached);
      }
    }
  }
  A->Cmu    = MapPage (CMU_PAGE);
  A->GpioRt = MapPage (GPIO_RT_PAGE);
  A->GpioT  = MapPage (GPIO_T_PAGE);
  A->Bcm    = ExAllocatePoolWithTag (NonPagedPool, sizeof (BCM_SDIO), WLAN_TAG);
  A->RxBuffer = ExAllocatePoolWithTag (NonPagedPool, RX_FRAME_MAX, WLAN_TAG);
  if ((A->Mmc == NULL) || (A->Cmu == NULL) || (A->GpioRt == NULL) || (A->GpioT == NULL) || (A->Bcm == NULL) ||
      (A->RxBuffer == NULL)) {
    OsLog ("exynoswlan: resources missing (dw_mmc1 %p)", A->Mmc);
    FreeAdapter (A);
    return NDIS_STATUS_RESOURCES;
  }
  RtlZeroMemory (A->Bcm, sizeof (BCM_SDIO));
  A->Bcm->RxHandler = RxHandler;
  A->Bcm->RxContext = A;

  RtlZeroMemory (&Pool, sizeof (Pool));
  Pool.Header.Type     = NDIS_OBJECT_TYPE_DEFAULT;
  Pool.Header.Revision = NET_BUFFER_LIST_POOL_PARAMETERS_REVISION_1;
  Pool.Header.Size     = NDIS_SIZEOF_NET_BUFFER_LIST_POOL_PARAMETERS_REVISION_1;
  Pool.fAllocateNetBuffer = TRUE;
  Pool.PoolTag         = WLAN_TAG;
  A->NblPool = NdisAllocateNetBufferListPool (Handle, &Pool);
  A->RxMdl   = (A->NblPool != NULL) ? NdisAllocateMdl (Handle, A->RxBuffer, RX_FRAME_MAX) : NULL;
  A->RxNbl   = (A->RxMdl != NULL) ? NdisAllocateNetBufferAndNetBufferList (A->NblPool, 0, 0, A->RxMdl, 0, 0) : NULL;
  if (A->RxNbl == NULL) {
    FreeAdapter (A);
    return NDIS_STATUS_RESOURCES;
  }

  ReadConfig (A);
  Status = StartChip (A);
  if (Status != NDIS_STATUS_SUCCESS) {
    FreeAdapter (A);
    return Status;
  }

  RtlZeroMemory (&Reg, sizeof (Reg));
  Reg.Header.Type        = NDIS_OBJECT_TYPE_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES;
  Reg.Header.Revision    = NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES_REVISION_1;
  Reg.Header.Size        = NDIS_SIZEOF_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES_REVISION_1;
  Reg.MiniportAdapterContext = A;
  Reg.AttributeFlags     = NDIS_MINIPORT_ATTRIBUTES_NO_HALT_ON_SUSPEND;
  Reg.InterfaceType      = NdisInterfaceInternal;
  Status = NdisMSetMiniportAttributes (Handle, (PNDIS_MINIPORT_ADAPTER_ATTRIBUTES)&Reg);
  if (Status != NDIS_STATUS_SUCCESS) {
    WlanPower (A, FALSE);
    FreeAdapter (A);
    return Status;
  }

  RtlZeroMemory (&Gen, sizeof (Gen));
  Gen.Header.Type        = NDIS_OBJECT_TYPE_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES;
  Gen.Header.Revision    = NDIS_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES_REVISION_1;
  Gen.Header.Size        = NDIS_SIZEOF_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES_REVISION_1;
  Gen.MediaType          = NdisMedium802_3;
  Gen.PhysicalMediumType = NdisPhysicalMediumUnspecified;
  Gen.MtuSize            = 1500;
  Gen.MaxXmitLinkSpeed   = 54000000ULL;
  Gen.MaxRcvLinkSpeed    = 54000000ULL;
  Gen.XmitLinkSpeed      = NDIS_LINK_SPEED_UNKNOWN;
  Gen.RcvLinkSpeed       = NDIS_LINK_SPEED_UNKNOWN;
  Gen.MediaConnectState  = MediaConnectStateDisconnected;
  Gen.MediaDuplexState   = MediaDuplexStateFull;
  Gen.LookaheadSize      = RX_FRAME_MAX;
  Gen.MacOptions         = NDIS_MAC_OPTION_COPY_LOOKAHEAD_DATA | NDIS_MAC_OPTION_TRANSFERS_NOT_PEND |
                           NDIS_MAC_OPTION_NO_LOOPBACK;
  Gen.SupportedPacketFilters = NDIS_PACKET_TYPE_DIRECTED | NDIS_PACKET_TYPE_MULTICAST |
                               NDIS_PACKET_TYPE_ALL_MULTICAST | NDIS_PACKET_TYPE_BROADCAST;
  Gen.MaxMulticastListSize = 32;
  Gen.MacAddressLength   = 6;
  RtlCopyMemory (Gen.PermanentMacAddress, A->Mac, 6);
  RtlCopyMemory (Gen.CurrentMacAddress, A->Mac, 6);
  Gen.RecvScaleCapabilities = NULL;
  Gen.AccessType         = NET_IF_ACCESS_BROADCAST;
  Gen.DirectionType      = NET_IF_DIRECTION_SENDRECEIVE;
  Gen.ConnectionType     = NET_IF_CONNECTION_DEDICATED;
  Gen.IfType             = IF_TYPE_ETHERNET_CSMACD;
  Gen.IfConnectorPresent = TRUE;
  Gen.SupportedStatistics = NDIS_STATISTICS_XMIT_OK_SUPPORTED | NDIS_STATISTICS_RCV_OK_SUPPORTED |
                            NDIS_STATISTICS_XMIT_ERROR_SUPPORTED;
  Gen.SupportedOidList   = (PNDIS_OID)mSupportedOids;
  Gen.SupportedOidListLength = sizeof (mSupportedOids);
  Status = NdisMSetMiniportAttributes (Handle, (PNDIS_MINIPORT_ADAPTER_ATTRIBUTES)&Gen);
  if (Status != NDIS_STATUS_SUCCESS) {
    WlanPower (A, FALSE);
    FreeAdapter (A);
    return Status;
  }

  Status = RegisterInterrupt (A);
  if (Status != NDIS_STATUS_SUCCESS) {
    WlanPower (A, FALSE);
    FreeAdapter (A);
    return Status;
  }

  //
  // Before the thread exists: from then on it owns the bus (join first).
  // An interrupt arriving in between only sets the event it waits on.
  //
  BcmSdioEnableInterrupts (A->Bcm);
  InitializeObjectAttributes (&Attr, NULL, OBJ_KERNEL_HANDLE, NULL, NULL);
  Nt = PsCreateSystemThread (&ThreadHandle, THREAD_ALL_ACCESS, &Attr, NULL, NULL, BusThread, A);
  if (!NT_SUCCESS (Nt)) {
    NdisMDeregisterInterruptEx (A->Interrupt);
    WlanPower (A, FALSE);
    FreeAdapter (A);
    return NDIS_STATUS_RESOURCES;
  }
  ObReferenceObjectByHandle (ThreadHandle, THREAD_ALL_ACCESS, NULL, KernelMode, (PVOID *)&A->Thread, NULL);
  ZwClose (ThreadHandle);
  OsLog ("exynoswlan: adapter ready, card interrupt on");
  return NDIS_STATUS_SUCCESS;
}

VOID
mpHalt (NDIS_HANDLE Context, NDIS_HALT_ACTION Action)
{
  PADAPTER  A = Context;

  UNREFERENCED_PARAMETER (Action);

  A->Stop = TRUE;
  KeSetEvent (&A->Wake, 0, FALSE);
  if (A->Thread != NULL) {
    KeWaitForSingleObject (A->Thread, Executive, KernelMode, FALSE, NULL);
    ObDereferenceObject (A->Thread);
  }
  if (A->Interrupt != NULL) {
    NdisMDeregisterInterruptEx (A->Interrupt);
  }
  if (A->Up) {
    BcmIoctl (A->Bcm, WLC_DOWN, TRUE, NULL, 0, 500);
  }
  WlanPower (A, FALSE);
  OsLog ("exynoswlan: halted after %lu interrupts, %lu frames, %lu superframes, %lu rx resyncs; chip powered off",
         A->Interrupts, A->Bcm->Frames, A->Bcm->Gloms, A->Bcm->RxFails);
  FreeAdapter (A);
}

VOID
mpUnload (PDRIVER_OBJECT DriverObject)
{
  UNREFERENCED_PARAMETER (DriverObject);
  NdisMDeregisterMiniportDriver (mDriverHandle);
}

NDIS_STATUS
mpPause (NDIS_HANDLE Context, PNDIS_MINIPORT_PAUSE_PARAMETERS Params)
{
  PADAPTER  A = Context;

  UNREFERENCED_PARAMETER (Params);
  A->Paused = TRUE;
  KeSetEvent (&A->Wake, 0, FALSE);      // queued sends complete as PAUSED
  return NDIS_STATUS_SUCCESS;
}

NDIS_STATUS
mpRestart (NDIS_HANDLE Context, PNDIS_MINIPORT_RESTART_PARAMETERS Params)
{
  PADAPTER  A = Context;

  UNREFERENCED_PARAMETER (Params);
  A->Paused      = FALSE;
  A->LinkChanged = TRUE;                // tell NDIS the current link state
  KeSetEvent (&A->Wake, 0, FALSE);
  return NDIS_STATUS_SUCCESS;
}

VOID
mpSend (NDIS_HANDLE Context, PNET_BUFFER_LIST List, NDIS_PORT_NUMBER Port, ULONG Flags)
{
  PADAPTER          A = Context;
  PNET_BUFFER_LIST  Last;
  PNET_BUFFER_LIST  Nbl;
  PNET_BUFFER_LIST  Next;
  ULONG             Count;
  BOOLEAN           Reject;

  UNREFERENCED_PARAMETER (Port);

  for (Count = 0, Last = List; NET_BUFFER_LIST_NEXT_NBL (Last) != NULL; Last = NET_BUFFER_LIST_NEXT_NBL (Last)) {
    Count++;
  }
  Count++;

  NdisAcquireSpinLock (&A->Lock);
  Reject = A->Paused || (A->TxQueued + Count > TX_QUEUE_MAX);
  if (!Reject) {
    if (A->TxTail != NULL) {
      NET_BUFFER_LIST_NEXT_NBL (A->TxTail) = List;
    } else {
      A->TxHead = List;
    }
    A->TxTail    = Last;
    A->TxQueued += Count;
  }
  NdisReleaseSpinLock (&A->Lock);

  if (Reject) {
    for (Nbl = List; Nbl != NULL; Nbl = Next) {
      Next = NET_BUFFER_LIST_NEXT_NBL (Nbl);
      NET_BUFFER_LIST_STATUS (Nbl) = A->Paused ? NDIS_STATUS_PAUSED : NDIS_STATUS_RESOURCES;
    }
    NdisMSendNetBufferListsComplete (A->Handle, List,
                                     NDIS_TEST_SEND_AT_DISPATCH_LEVEL (Flags) ? NDIS_SEND_COMPLETE_FLAGS_DISPATCH_LEVEL : 0);
    return;
  }
  KeSetEvent (&A->Wake, 0, FALSE);
}

VOID
mpReturn (NDIS_HANDLE Context, PNET_BUFFER_LIST List, ULONG Flags)
{
  //
  // Receives are indicated with NDIS_RECEIVE_FLAGS_RESOURCES: nothing comes
  // back here.
  //
  UNREFERENCED_PARAMETER (Context);
  UNREFERENCED_PARAMETER (List);
  UNREFERENCED_PARAMETER (Flags);
}

VOID
mpCancelSend (NDIS_HANDLE Context, PVOID CancelId)
{
  UNREFERENCED_PARAMETER (Context);
  UNREFERENCED_PARAMETER (CancelId);
}

VOID
mpPnpEvent (NDIS_HANDLE Context, PNET_DEVICE_PNP_EVENT Event)
{
  UNREFERENCED_PARAMETER (Context);
  UNREFERENCED_PARAMETER (Event);
}

VOID
mpShutdown (NDIS_HANDLE Context, NDIS_SHUTDOWN_ACTION Action)
{
  PADAPTER  A = Context;

  UNREFERENCED_PARAMETER (Action);
  WlanPower (A, FALSE);
}

VOID
mpCancelOid (NDIS_HANDLE Context, PVOID RequestId)
{
  UNREFERENCED_PARAMETER (Context);
  UNREFERENCED_PARAMETER (RequestId);
}

static NDIS_STATUS
Answer (PNDIS_OID_REQUEST Req, const VOID *Data, ULONG Size)
{
  if (Req->DATA.QUERY_INFORMATION.InformationBufferLength < Size) {
    Req->DATA.QUERY_INFORMATION.BytesNeeded = Size;
    return NDIS_STATUS_BUFFER_TOO_SHORT;
  }
  RtlCopyMemory (Req->DATA.QUERY_INFORMATION.InformationBuffer, Data, Size);
  Req->DATA.QUERY_INFORMATION.BytesWritten = Size;
  return NDIS_STATUS_SUCCESS;
}

NDIS_STATUS
mpOidRequest (NDIS_HANDLE Context, PNDIS_OID_REQUEST Req)
{
  PADAPTER                 A = Context;
  NDIS_STATISTICS_INFO     Stats;
  ULONG                    U;
  ULONG64                  U64;
  NDIS_MEDIA_STATE         Media;
  static const char        Vendor[] = "Broadcom BCM4354 (SM-P900)";

  if ((Req->RequestType == NdisRequestQueryInformation) || (Req->RequestType == NdisRequestQueryStatistics)) {
    switch (Req->DATA.QUERY_INFORMATION.Oid) {
      case OID_GEN_SUPPORTED_LIST:        return Answer (Req, mSupportedOids, sizeof (mSupportedOids));
      case OID_GEN_HARDWARE_STATUS:       U = NdisHardwareStatusReady; return Answer (Req, &U, 4);
      case OID_GEN_MEDIA_SUPPORTED:
      case OID_GEN_MEDIA_IN_USE:          U = NdisMedium802_3; return Answer (Req, &U, 4);
      case OID_GEN_MAXIMUM_FRAME_SIZE:    U = 1500; return Answer (Req, &U, 4);
      case OID_GEN_MAXIMUM_TOTAL_SIZE:
      case OID_GEN_TRANSMIT_BLOCK_SIZE:
      case OID_GEN_RECEIVE_BLOCK_SIZE:    U = 1514; return Answer (Req, &U, 4);
      case OID_GEN_LINK_SPEED:            U = 540000; return Answer (Req, &U, 4);   // 100 bps units
      case OID_GEN_VENDOR_ID:             U = 0x00904C; return Answer (Req, &U, 4);
      case OID_GEN_VENDOR_DESCRIPTION:    return Answer (Req, Vendor, sizeof (Vendor));
      case OID_GEN_VENDOR_DRIVER_VERSION: U = 0x00010000; return Answer (Req, &U, 4);
      case OID_GEN_DRIVER_VERSION:        U = 0x0630; return Answer (Req, &U, 2);
      case OID_GEN_CURRENT_PACKET_FILTER: return Answer (Req, &A->PacketFilter, 4);
      case OID_GEN_CURRENT_LOOKAHEAD:     return Answer (Req, &A->Lookahead, 4);
      case OID_GEN_MAXIMUM_SEND_PACKETS:  U = 16; return Answer (Req, &U, 4);
      case OID_GEN_MEDIA_CONNECT_STATUS:
        Media = A->LinkUp ? NdisMediaStateConnected : NdisMediaStateDisconnected;
        return Answer (Req, &Media, sizeof (Media));
      case OID_GEN_XMIT_OK:               U64 = A->TxFrames; return Answer (Req, &U64, 8);
      case OID_GEN_RCV_OK:                U64 = A->RxFrames; return Answer (Req, &U64, 8);
      case OID_GEN_XMIT_ERROR:            U64 = A->TxErrors; return Answer (Req, &U64, 8);
      case OID_GEN_RCV_ERROR:
      case OID_GEN_RCV_NO_BUFFER:
      case OID_802_3_RCV_ERROR_ALIGNMENT:
      case OID_802_3_XMIT_ONE_COLLISION:
      case OID_802_3_XMIT_MORE_COLLISIONS: U64 = 0; return Answer (Req, &U64, 8);
      case OID_802_3_PERMANENT_ADDRESS:
      case OID_802_3_CURRENT_ADDRESS:     return Answer (Req, A->Mac, 6);
      case OID_802_3_MAXIMUM_LIST_SIZE:   U = 32; return Answer (Req, &U, 4);
      case OID_GEN_STATISTICS:
        RtlZeroMemory (&Stats, sizeof (Stats));
        Stats.Header.Type     = NDIS_OBJECT_TYPE_DEFAULT;
        Stats.Header.Revision = NDIS_STATISTICS_INFO_REVISION_1;
        Stats.Header.Size     = NDIS_SIZEOF_STATISTICS_INFO_REVISION_1;
        Stats.SupportedStatistics = NDIS_STATISTICS_XMIT_OK_SUPPORTED | NDIS_STATISTICS_RCV_OK_SUPPORTED |
                                    NDIS_STATISTICS_XMIT_ERROR_SUPPORTED;
        Stats.ifHCInUcastPkts  = A->RxFrames;
        Stats.ifHCOutUcastPkts = A->TxFrames;
        Stats.ifOutErrors      = A->TxErrors;
        return Answer (Req, &Stats, sizeof (Stats));
      default:
        return NDIS_STATUS_NOT_SUPPORTED;
    }
  }

  if (Req->RequestType == NdisRequestSetInformation) {
    switch (Req->DATA.SET_INFORMATION.Oid) {
      case OID_GEN_CURRENT_PACKET_FILTER:
        if (Req->DATA.SET_INFORMATION.InformationBufferLength < 4) {
          return NDIS_STATUS_INVALID_LENGTH;
        }
        A->PacketFilter = *(PULONG)Req->DATA.SET_INFORMATION.InformationBuffer;
        Req->DATA.SET_INFORMATION.BytesRead = 4;
        return NDIS_STATUS_SUCCESS;
      case OID_GEN_CURRENT_LOOKAHEAD:
        if (Req->DATA.SET_INFORMATION.InformationBufferLength < 4) {
          return NDIS_STATUS_INVALID_LENGTH;
        }
        A->Lookahead = *(PULONG)Req->DATA.SET_INFORMATION.InformationBuffer;
        Req->DATA.SET_INFORMATION.BytesRead = 4;
        return NDIS_STATUS_SUCCESS;
      case OID_802_3_MULTICAST_LIST:
        //
        // The firmware passes multicast through as it is configured; the
        // list is accepted (filtering above us does the rest).
        //
        Req->DATA.SET_INFORMATION.BytesRead = Req->DATA.SET_INFORMATION.InformationBufferLength;
        return NDIS_STATUS_SUCCESS;
      default:
        return NDIS_STATUS_NOT_SUPPORTED;
    }
  }
  return NDIS_STATUS_NOT_SUPPORTED;
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  NDIS_MINIPORT_DRIVER_CHARACTERISTICS  Chars;
  NDIS_STATUS                           Status;

  OsLogInit ();
  OsLog ("exynoswlan: DriverEntry (built " __DATE__ " " __TIME__ ")");

  RtlZeroMemory (&Chars, sizeof (Chars));
  Chars.Header.Type     = NDIS_OBJECT_TYPE_MINIPORT_DRIVER_CHARACTERISTICS;
  Chars.Header.Revision = NDIS_MINIPORT_DRIVER_CHARACTERISTICS_REVISION_2;
  Chars.Header.Size     = NDIS_SIZEOF_MINIPORT_DRIVER_CHARACTERISTICS_REVISION_2;
  Chars.MajorNdisVersion = 6;
  Chars.MinorNdisVersion = 30;
  Chars.MajorDriverVersion = 1;
  Chars.MinorDriverVersion = 0;
  Chars.InitializeHandlerEx       = mpInitialize;
  Chars.HaltHandlerEx             = mpHalt;
  Chars.UnloadHandler             = mpUnload;
  Chars.PauseHandler              = mpPause;
  Chars.RestartHandler            = mpRestart;
  Chars.OidRequestHandler         = mpOidRequest;
  Chars.SendNetBufferListsHandler = mpSend;
  Chars.ReturnNetBufferListsHandler = mpReturn;
  Chars.CancelSendHandler         = mpCancelSend;
  Chars.DevicePnPEventNotifyHandler = mpPnpEvent;
  Chars.ShutdownHandlerEx         = mpShutdown;
  Chars.CancelOidRequestHandler   = mpCancelOid;

  Status = NdisMRegisterMiniportDriver (DriverObject, RegistryPath, NULL, &Chars, &mDriverHandle);
  if (Status != NDIS_STATUS_SUCCESS) {
    OsLog ("exynoswlan: NdisMRegisterMiniportDriver failed: 0x%08lx", Status);
  }
  return Status;
}
