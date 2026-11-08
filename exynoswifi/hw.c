/** @file
  exynoswifi chip side: board (power, pins, clock), firmware bring-up, the
  card interrupt, the bus thread, firmware events, the BSS cache, and the
  ExtSTA operations (scan, connect, disconnect, keys, radio) as firmware
  commands.  The data path converts 802.11 + LLC/SNAP (Native Wi-Fi) to
  802.3 (firmware) and back.

  Board, bring-up, interrupt and bus thread follow ../exynoswlan/exynoswlan.c
  (verified on the device); see that file for the pin, clock and ACPI notes.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include "wifi.h"

//
// Physical registers the driver maps itself (see exynoswlan.c).
//
#define CLK_CPLL_CON0       0x120
#define CLK_SRC_FSYS        0x244
#define CLK_SRC_MASK_FSYS   0x340
#define CLK_DIV_FSYS1       0x54C
#define CLK_GATE_IP_FSYS    0x944
#define GPY7_CON            0x000
#define GPY7_DAT            0x004
#define GPC1_CON            0x020
#define GPC1_PUD            0x028
#define GPC1_DRV            0x02C
#define WLAN_EN_PIN         7

//
// Events the firmware must report (event_msgs).
//
static const UCHAR  mEvents[] = {
  WLC_E_SET_SSID, WLC_E_JOIN, WLC_E_AUTH, WLC_E_DEAUTH, WLC_E_DEAUTH_IND, WLC_E_ASSOC, WLC_E_REASSOC,
  WLC_E_DISASSOC, WLC_E_DISASSOC_IND, WLC_E_LINK, WLC_E_ROAM, WLC_E_ESCAN_RESULT
};

static const UCHAR  mBroadcast[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

//
// ---------------------------------------------------------------------------
// Helpers.
// ---------------------------------------------------------------------------
//

ULONGLONG
NowMs (VOID)
{
  return KeQueryInterruptTime () / 10000;
}

static ULONG
Le16 (const UCHAR *P)
{
  return (ULONG)P[0] | ((ULONG)P[1] << 8);
}

static ULONG
Le32 (const UCHAR *P)
{
  return (ULONG)P[0] | ((ULONG)P[1] << 8) | ((ULONG)P[2] << 16) | ((ULONG)P[3] << 24);
}

static ULONG
Be32 (const UCHAR *P)
{
  return ((ULONG)P[0] << 24) | ((ULONG)P[1] << 16) | ((ULONG)P[2] << 8) | P[3];
}

static VOID
Put16 (UCHAR *P, ULONG V)
{
  P[0] = (UCHAR)V;
  P[1] = (UCHAR)(V >> 8);
}

static VOID
Put32 (UCHAR *P, ULONG V)
{
  P[0] = (UCHAR)V;
  P[1] = (UCHAR)(V >> 8);
  P[2] = (UCHAR)(V >> 16);
  P[3] = (UCHAR)(V >> 24);
}

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

static ULONG
LinkQuality (LONG Rssi)
{
  LONG  Q = 2 * (Rssi + 100);

  return (ULONG)((Q < 0) ? 0 : (Q > 100) ? 100 : Q);
}

/**
  Chanspec (d11ac format) -> channel, MHz, PHY id.
**/
static VOID
DecodeChannel (ULONG Chanspec, ULONG CtlCh, ULONG *Channel, ULONG *Freq, ULONG *PhyId)
{
  BOOLEAN  Is5G = (Chanspec & 0xC000) == 0xC000;
  ULONG    Ch   = (CtlCh != 0) ? CtlCh : (Chanspec & 0xFF);

  if (!Is5G && (Ch > 14)) {
    Is5G = TRUE;                        // older chanspec encodings
  }
  *Channel = Ch;
  *PhyId   = Is5G ? PHY_ID_OFDM : PHY_ID_ERP;
  *Freq    = Is5G ? (5000 + 5 * Ch) : ((Ch == 14) ? 2484 : (2407 + 5 * Ch));
}

/**
  TRUE when Data[0..Length) is exactly a chain of information elements.
**/
static BOOLEAN
IeChainOk (const UCHAR *Data, ULONG Length)
{
  ULONG  Pos = 0;

  while (Pos + 2 <= Length) {
    Pos += 2 + Data[Pos + 1];
  }
  return Pos == Length;
}

//
// ---------------------------------------------------------------------------
// Board.
// ---------------------------------------------------------------------------
//

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
      continue;                         // GPS_EN: never touched
    }
    RegWrite (A->GpioT, GPC1_CON, (Reg (A->GpioT, GPC1_CON) & ~(0xFu << (Pin * 4))) | (2u << (Pin * 4)));
    RegWrite (A->GpioT, GPC1_PUD, (Reg (A->GpioT, GPC1_PUD) & ~(3u << (Pin * 2))) | ((Pin == 0) ? 0u : (3u << (Pin * 2))));
    RegWrite (A->GpioT, GPC1_DRV, (Reg (A->GpioT, GPC1_DRV) & ~(3u << (Pin * 2))) | (1u << (Pin * 2)));
  }
}

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

VOID
HwPowerOff (PADAPTER A)
{
  if (A->GpioRt != NULL) {
    WlanPower (A, FALSE);
  }
}

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
    *Data = ExAllocatePoolWithTag (NonPagedPool, *Size + 8, WIFI_TAG);
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
    ExFreePoolWithTag (*Data, WIFI_TAG);
    *Data = NULL;
  }
  return Status;
}

//
// ---------------------------------------------------------------------------
// MIB defaults (ExtSTA reset values).
// ---------------------------------------------------------------------------
//

VOID
WifiResetMib (PADAPTER A)
{
  A->OperationMode      = DOT11_OPERATION_MODE_EXTENSIBLE_STATION;
  A->MibPhyId           = PHY_ID_ERP;
  A->DesiredSsidCount   = 0;
  A->DesiredBssType     = dot11_BSS_type_infrastructure;
  A->DesiredBssidCount  = 1;
  RtlCopyMemory (A->DesiredBssid[0], mBroadcast, 6);
  A->ExcludedCount      = 0;
  A->AuthAlgorithm      = DOT11_AUTH_ALGO_80211_OPEN;
  A->UnicastCipher      = DOT11_CIPHER_ALGO_NONE;
  A->MulticastCipher    = DOT11_CIPHER_ALGO_NONE;
  A->ExcludeUnencrypted = FALSE;
  A->PrivacyExemptCount = 0;
  A->AutoConfig         = DOT11_PHY_AUTO_CONFIG_ENABLED_FLAG | DOT11_MAC_AUTO_CONFIG_ENABLED_FLAG;
  A->MediaStreaming     = FALSE;
  A->UnreachableThreshold = 2000;
  A->UnicastUseGroup    = TRUE;
  A->SafeMode           = FALSE;
  A->HiddenNetwork      = FALSE;
  A->PowerSaving        = DOT11_POWER_SAVING_NO_POWER_SAVING;
  A->DefaultKeyId       = 0;
  A->RtsThreshold       = 2347;
  A->FragThreshold      = 2346;
  A->DesiredPhyCount    = 1;
  A->DesiredPhy[0]      = DOT11_PHY_ID_ANY;
  A->AssocIesLength     = 0;
  A->Country[0] = A->Country[1] = A->Country[2] = 0;
}

//
// ---------------------------------------------------------------------------
// Indications.
// ---------------------------------------------------------------------------
//

VOID
WifiIndicate (PADAPTER A, NDIS_STATUS Code, PVOID Buffer, ULONG Size)
{
  NDIS_STATUS_INDICATION  Ind;

  RtlZeroMemory (&Ind, sizeof (Ind));
  HDR (Ind.Header, NDIS_OBJECT_TYPE_STATUS_INDICATION, NDIS_STATUS_INDICATION_REVISION_1,
       NDIS_SIZEOF_STATUS_INDICATION_REVISION_1);
  Ind.SourceHandle     = A->Handle;
  Ind.PortNumber       = NDIS_DEFAULT_PORT_NUMBER;
  Ind.StatusCode       = Code;
  Ind.StatusBuffer     = Buffer;
  Ind.StatusBufferSize = Size;
  NdisMIndicateStatusEx (A->Handle, &Ind);
}

VOID
WifiIndicateLink (PADAPTER A, BOOLEAN Connected)
{
  NDIS_LINK_STATE  Link;

  RtlZeroMemory (&Link, sizeof (Link));
  HDR (Link.Header, NDIS_OBJECT_TYPE_DEFAULT, NDIS_LINK_STATE_REVISION_1, NDIS_SIZEOF_LINK_STATE_REVISION_1);
  Link.MediaConnectState = Connected ? MediaConnectStateConnected : MediaConnectStateDisconnected;
  Link.MediaDuplexState  = MediaDuplexStateHalf;
  Link.XmitLinkSpeed     = Connected ? A->LinkSpeed : NDIS_LINK_SPEED_UNKNOWN;
  Link.RcvLinkSpeed      = Link.XmitLinkSpeed;
  WifiIndicate (A, NDIS_STATUS_LINK_STATE, &Link, sizeof (Link));
}

static VOID
IndicateScanConfirm (PADAPTER A, NDIS_STATUS Status)
{
  A->Scanning = FALSE;
  OsLog ("exynoswifi: scan done (0x%08lx)", Status);
  WifiIndicate (A, NDIS_STATUS_DOT11_SCAN_CONFIRM, &Status, sizeof (Status));
}

static VOID
IndicateConnectionStart (PADAPTER A)
{
  DOT11_CONNECTION_START_PARAMETERS  P;

  RtlZeroMemory (&P, sizeof (P));
  HDR (P.Header, NDIS_OBJECT_TYPE_DEFAULT, DOT11_CONNECTION_START_PARAMETERS_REVISION_1, sizeof (P));
  P.BSSType = dot11_BSS_type_infrastructure;
  WifiIndicate (A, NDIS_STATUS_DOT11_CONNECTION_START, &P, sizeof (P));
}

static VOID
IndicateConnectionCompletion (PADAPTER A, ULONG Status)
{
  DOT11_CONNECTION_COMPLETION_PARAMETERS  P;

  RtlZeroMemory (&P, sizeof (P));
  HDR (P.Header, NDIS_OBJECT_TYPE_DEFAULT, DOT11_CONNECTION_COMPLETION_PARAMETERS_REVISION_1, sizeof (P));
  P.uStatus = Status;
  WifiIndicate (A, NDIS_STATUS_DOT11_CONNECTION_COMPLETION, &P, sizeof (P));
}

static VOID
IndicateAssociationStart (PADAPTER A)
{
  DOT11_ASSOCIATION_START_PARAMETERS  P;

  RtlZeroMemory (&P, sizeof (P));
  HDR (P.Header, NDIS_OBJECT_TYPE_DEFAULT, DOT11_ASSOCIATION_START_PARAMETERS_REVISION_1, sizeof (P));
  RtlCopyMemory (P.MacAddr, A->Bssid, 6);
  P.SSID = A->JoinSsid;
  A->AssocStartIndicated = TRUE;
  WifiIndicate (A, NDIS_STATUS_DOT11_ASSOCIATION_START, &P, sizeof (P));
}

static VOID
IndicateAssociationFailure (PADAPTER A, ULONG Status)
{
  DOT11_ASSOCIATION_COMPLETION_PARAMETERS  P;

  RtlZeroMemory (&P, sizeof (P));
  HDR (P.Header, NDIS_OBJECT_TYPE_DEFAULT, DOT11_ASSOCIATION_COMPLETION_PARAMETERS_REVISION_2, sizeof (P));
  RtlCopyMemory (P.MacAddr, A->Bssid, 6);
  P.uStatus = Status;
  WifiIndicate (A, NDIS_STATUS_DOT11_ASSOCIATION_COMPLETION, &P, sizeof (P));
}

static VOID
IndicateDisassociation (PADAPTER A, ULONG Reason)
{
  DOT11_DISASSOCIATION_PARAMETERS  P;

  RtlZeroMemory (&P, sizeof (P));
  HDR (P.Header, NDIS_OBJECT_TYPE_DEFAULT, DOT11_DISASSOCIATION_PARAMETERS_REVISION_1, sizeof (P));
  RtlCopyMemory (P.MacAddr, A->Bssid, 6);
  P.uReason = Reason;
  WifiIndicate (A, NDIS_STATUS_DOT11_DISASSOCIATION, &P, sizeof (P));
}

static VOID
IndicateLinkQuality (PADAPTER A)
{
  struct {
    DOT11_LINK_QUALITY_PARAMETERS  P;
    DOT11_LINK_QUALITY_ENTRY       E;
  } Q;

  RtlZeroMemory (&Q, sizeof (Q));
  HDR (Q.P.Header, NDIS_OBJECT_TYPE_DEFAULT, DOT11_LINK_QUALITY_PARAMETERS_REVISION_1, sizeof (Q.P));
  Q.P.uLinkQualityListSize   = 1;
  Q.P.uLinkQualityListOffset = FIELD_OFFSET (__typeof__ (Q), E);
  RtlCopyMemory (Q.E.PeerMacAddr, A->Bssid, 6);
  Q.E.ucLinkQuality = (UCHAR)LinkQuality (A->Rssi);
  WifiIndicate (A, NDIS_STATUS_DOT11_LINK_QUALITY, &Q, sizeof (Q));
}

//
// ---------------------------------------------------------------------------
// BSS cache.
// ---------------------------------------------------------------------------
//

/**
  One wl_bss_info_t (v109) into the cache.  Called from the RX handler
  (escan results) and from the thread (the joined BSS).
**/
static VOID
BssUpdate (PADAPTER A, const UCHAR *Bi, ULONG Length)
{
  BSS_ENTRY      *E;
  BSS_ENTRY      *Slot = NULL;
  BSS_ENTRY      *Oldest = NULL;
  ULONG          BiLen;
  ULONG          IeOff;
  ULONG          IeLen;
  ULONG          Index;
  LARGE_INTEGER  SysTime;
  KIRQL          Irql;

  if (Length < 128) {
    return;
  }
  BiLen = Le32 (Bi + 4);
  IeOff = Le16 (Bi + 116);
  IeLen = Le32 (Bi + 120);
  if ((BiLen > Length) || (BiLen < 128) || (IeOff < 128) || (IeOff > BiLen) || (IeLen > BiLen - IeOff)) {
    return;
  }
  if (IeLen > BSS_IE_MAX) {
    IeLen = BSS_IE_MAX;
  }
  KeQuerySystemTime (&SysTime);

  //
  // The entry of this BSSID, else a free one, else the oldest.
  //
  KeAcquireSpinLock (&A->BssLock, &Irql);
  for (Index = 0; (Index < BSS_CACHE_SIZE) && (Slot == NULL); Index++) {
    E = &A->Bss[Index];
    if (E->Valid && (RtlCompareMemory (E->Bssid, Bi + 8, 6) == 6)) {
      Slot = E;
    }
  }
  for (Index = 0; (Index < BSS_CACHE_SIZE) && (Slot == NULL); Index++) {
    if (!A->Bss[Index].Valid) {
      Slot = &A->Bss[Index];
    }
  }
  for (Index = 0; (Index < BSS_CACHE_SIZE) && (Slot == NULL); Index++) {
    E = &A->Bss[Index];
    if ((Oldest == NULL) || (E->SeenAt < Oldest->SeenAt)) {
      Oldest = E;
    }
  }
  if (Slot == NULL) {
    Slot = Oldest;
  }
  if (Slot != NULL) {
    E = Slot;
    E->Valid = TRUE;
    RtlCopyMemory (E->Bssid, Bi + 8, 6);
    E->BeaconPeriod = (USHORT)Le16 (Bi + 14);
    E->Capability   = (USHORT)Le16 (Bi + 16);
    E->SsidLength   = (Bi[18] > 32) ? 32 : Bi[18];
    RtlCopyMemory (E->Ssid, Bi + 19, E->SsidLength);
    E->Chanspec = (USHORT)Le16 (Bi + 72);
    E->Rssi     = (SHORT)Le16 (Bi + 78);
    DecodeChannel (E->Chanspec, Bi[88], &E->Channel, &E->Frequency, &E->PhyId);
    E->SeenAt        = KeQueryInterruptTime ();
    E->HostTimestamp = SysTime.QuadPart;
    E->IeLength      = IeLen;
    RtlCopyMemory (E->Ies, Bi + IeOff, IeLen);
  }
  KeReleaseSpinLock (&A->BssLock, Irql);
}

VOID
BssFlush (PADAPTER A)
{
  ULONG  Index;
  KIRQL  Irql;

  KeAcquireSpinLock (&A->BssLock, &Irql);
  for (Index = 0; Index < BSS_CACHE_SIZE; Index++) {
    if ((A->State == StateIdle) || (RtlCompareMemory (A->Bss[Index].Bssid, A->Bssid, 6) != 6)) {
      A->Bss[Index].Valid = FALSE;
    }
  }
  KeReleaseSpinLock (&A->BssLock, Irql);
}

/**
  DOT11_BSS_ENTRY records (no padding between them) into Out.  Returns the
  bytes written; *Total gets the bytes all entries need.
**/
ULONG
BssEnumerate (PADAPTER A, PUCHAR Out, ULONG OutSize, ULONG *Total)
{
  BSS_ENTRY        *E;
  DOT11_BSS_ENTRY  *D;
  ULONG            Index;
  ULONG            Need;
  ULONG            Written = 0;
  ULONGLONG        Now = KeQueryInterruptTime ();
  KIRQL            Irql;

  *Total = 0;
  KeAcquireSpinLock (&A->BssLock, &Irql);
  for (Index = 0; Index < BSS_CACHE_SIZE; Index++) {
    E = &A->Bss[Index];
    if (!E->Valid) {
      continue;
    }
    if ((Now - E->SeenAt > (ULONGLONG)BSS_AGE_MS * 10000) &&
        !((A->State != StateIdle) && (RtlCompareMemory (E->Bssid, A->Bssid, 6) == 6))) {
      continue;
    }
    Need    = FIELD_OFFSET (DOT11_BSS_ENTRY, ucBuffer) + E->IeLength;
    *Total += Need;
    if (Written + Need > OutSize) {
      continue;
    }
    D = (DOT11_BSS_ENTRY *)(Out + Written);
    RtlZeroMemory (D, FIELD_OFFSET (DOT11_BSS_ENTRY, ucBuffer));
    D->uPhyId = E->PhyId;
    D->PhySpecificInfo.uChCenterFrequency = E->Frequency;
    RtlCopyMemory (D->dot11BSSID, E->Bssid, 6);
    D->dot11BSSType   = (E->Capability & DOT11_CAPABILITY_INFO_IBSS) ? dot11_BSS_type_independent
                                                                     : dot11_BSS_type_infrastructure;
    D->lRSSI          = E->Rssi;
    D->uLinkQuality   = LinkQuality (E->Rssi);
    D->bInRegDomain   = TRUE;
    D->usBeaconPeriod = E->BeaconPeriod;
    D->ullTimestamp   = 0;
    D->ullHostTimestamp = E->HostTimestamp;
    D->usCapabilityInformation = E->Capability;
    D->uBufferLength  = E->IeLength;
    RtlCopyMemory (D->ucBuffer, E->Ies, E->IeLength);
    Written += Need;
  }
  KeReleaseSpinLock (&A->BssLock, Irql);
  return Written;
}

/**
  Best cached candidate for the desired SSID / BSSID lists; FALSE if none.
**/
static BOOLEAN
BssPick (PADAPTER A, BSS_ENTRY *Out)
{
  BSS_ENTRY  *E;
  BSS_ENTRY  *Best = NULL;
  ULONG      Index;
  ULONG      J;
  BOOLEAN    Ok;
  ULONGLONG  Now = KeQueryInterruptTime ();
  KIRQL      Irql;

  KeAcquireSpinLock (&A->BssLock, &Irql);
  for (Index = 0; Index < BSS_CACHE_SIZE; Index++) {
    E = &A->Bss[Index];
    if (!E->Valid || (Now - E->SeenAt > (ULONGLONG)BSS_AGE_MS * 10000) || (E->Capability & DOT11_CAPABILITY_INFO_IBSS)) {
      continue;
    }
    if ((A->DesiredSsidCount > 0) && (A->DesiredSsid[0].uSSIDLength != 0) &&
        ((E->SsidLength != A->DesiredSsid[0].uSSIDLength) ||
         (RtlCompareMemory (E->Ssid, A->DesiredSsid[0].ucSSID, E->SsidLength) != E->SsidLength))) {
      continue;
    }
    for (J = 0, Ok = TRUE; J < A->ExcludedCount; J++) {
      if (RtlCompareMemory (A->Excluded[J], E->Bssid, 6) == 6) {
        Ok = FALSE;
      }
    }
    if (Ok && (A->DesiredBssidCount > 0)) {
      for (J = 0, Ok = FALSE; J < A->DesiredBssidCount; J++) {
        if ((RtlCompareMemory (A->DesiredBssid[J], mBroadcast, 6) == 6) ||
            (RtlCompareMemory (A->DesiredBssid[J], E->Bssid, 6) == 6)) {
          Ok = TRUE;
        }
      }
    }
    if (Ok && (A->DesiredPhy[0] != DOT11_PHY_ID_ANY)) {
      for (J = 0, Ok = FALSE; J < A->DesiredPhyCount; J++) {
        if (A->DesiredPhy[J] == E->PhyId) {
          Ok = TRUE;
        }
      }
    }
    if (Ok && ((Best == NULL) || (E->Rssi > Best->Rssi))) {
      Best = E;
    }
  }
  if (Best != NULL) {
    RtlCopyMemory (Out, Best, sizeof (*Out));
  }
  KeReleaseSpinLock (&A->BssLock, Irql);
  return Best != NULL;
}

//
// ---------------------------------------------------------------------------
// Receive (called by BcmSdioPoll() in the bus thread).
// ---------------------------------------------------------------------------
//

/**
  An 802.3 frame from the firmware up as an 802.11 data frame (FromDS):
  addr1 = DA, addr2 = BSSID, addr3 = SA, then LLC/SNAP (802.1H for AARP
  and IPX, as nwifi expects) and the payload.
**/
static VOID
IndicateRx (PADAPTER A, const UCHAR *Eth, ULONG Length)
{
  PUCHAR  F = A->RxBuffer;
  ULONG   Type;
  ULONG   Out;

  if ((Length < 14) || A->Paused || (A->PacketFilter == 0)) {
    A->RxDropped++;
    return;
  }
  Type = ((ULONG)Eth[12] << 8) | Eth[13];
  F[0] = 0x08;                          // data
  F[1] = 0x02;                          // FromDS
  F[2] = F[3] = 0;
  RtlCopyMemory (F + 4, Eth, 6);        // DA
  RtlCopyMemory (F + 10, A->Bssid, 6);
  RtlCopyMemory (F + 16, Eth + 6, 6);   // SA
  F[22] = F[23] = 0;
  if (Type >= 0x600) {
    if (24 + 8 + (Length - 14) > RX_FRAME_MAX) {
      A->RxDropped++;
      return;
    }
    F[24] = 0xAA;
    F[25] = 0xAA;
    F[26] = 0x03;
    F[27] = 0x00;
    F[28] = 0x00;
    F[29] = ((Type == 0x80F3) || (Type == 0x8137)) ? 0xF8 : 0x00;
    F[30] = Eth[12];
    F[31] = Eth[13];
    RtlCopyMemory (F + 32, Eth + 14, Length - 14);
    Out = 32 + Length - 14;
  } else {
    //
    // 802.3 length field: the payload already starts with its LLC header.
    //
    if (24 + (Length - 14) > RX_FRAME_MAX) {
      A->RxDropped++;
      return;
    }
    RtlCopyMemory (F + 24, Eth + 14, Length - 14);
    Out = 24 + Length - 14;
  }

  RtlZeroMemory (&A->RxContext, sizeof (A->RxContext));
  HDR (A->RxContext.Header, NDIS_OBJECT_TYPE_DEFAULT, DOT11_EXTSTA_RECV_CONTEXT_REVISION_1,
       sizeof (DOT11_EXTSTA_RECV_CONTEXT));
  A->RxContext.uPhyId             = A->ActivePhyId;
  A->RxContext.uChCenterFrequency = A->ActiveFrequency;
  A->RxContext.usNumberOfMPDUsReceived = 1;
  A->RxContext.lRSSI              = A->Rssi;
  A->RxContext.ucDataRate         = (A->LinkSpeed >= 54000000) ? 108 : (UCHAR)(A->LinkSpeed / 500000);

  NET_BUFFER_DATA_LENGTH (NET_BUFFER_LIST_FIRST_NB (A->RxNbl)) = Out;
  NET_BUFFER_DATA_OFFSET (NET_BUFFER_LIST_FIRST_NB (A->RxNbl)) = 0;
  NET_BUFFER_CURRENT_MDL_OFFSET (NET_BUFFER_LIST_FIRST_NB (A->RxNbl)) = 0;
  NET_BUFFER_LIST_STATUS (A->RxNbl) = NDIS_STATUS_SUCCESS;
  NET_BUFFER_LIST_INFO (A->RxNbl, MediaSpecificInformation) = &A->RxContext;
  A->RxNbl->SourceHandle = A->Handle;
  A->RxFrames++;
  if (Eth[0] & 1) {
    A->RxMulticast++;
  }
  NdisMIndicateReceiveNetBufferLists (A->Handle, A->RxNbl, NDIS_DEFAULT_PORT_NUMBER, 1, NDIS_RECEIVE_FLAGS_RESOURCES);
}

static VOID
HandleEvent (PADAPTER A, const UCHAR *Payload, ULONG Length)
{
  const UCHAR  *E;
  const UCHAR  *Data;
  ULONG        Type;
  ULONG        Status;
  ULONG        Reason;
  ULONG        DataLen;
  ULONG        Flags;
  ULONG        Pos;
  ULONG        Count;
  ULONG        BiLen;

  if (Length < BCM_EVENT_MSG_OFFSET + 48) {
    return;
  }
  E       = Payload + BCM_EVENT_MSG_OFFSET;
  Flags   = ((ULONG)E[2] << 8) | E[3];
  Type    = Be32 (E + 4);
  Status  = Be32 (E + 8);
  Reason  = Be32 (E + 12);
  DataLen = Be32 (E + 20);
  Data    = E + 48;
  if (DataLen > Length - (BCM_EVENT_MSG_OFFSET + 48)) {
    DataLen = Length - (BCM_EVENT_MSG_OFFSET + 48);
  }

  switch (Type) {
    case WLC_E_ESCAN_RESULT:
      if (!A->Scanning) {
        break;
      }
      if (Status == WLC_E_STATUS_PARTIAL) {
        //
        // wl_escan_result_t: buflen, version, sync_id, bss_count, then the
        // wl_bss_info_t records.
        //
        if (DataLen < 12) {
          break;
        }
        Count = Le16 (Data + 10);
        for (Pos = 12; (Count > 0) && (Pos + 128 <= DataLen); Count--) {
          BiLen = Le32 (Data + Pos + 4);
          if ((BiLen < 128) || (Pos + BiLen > DataLen)) {
            break;
          }
          BssUpdate (A, Data + Pos, BiLen);
          Pos += BiLen;
        }
      } else {
        A->EvScanDone   = TRUE;
        A->EvScanStatus = Status;
      }
      break;

    case WLC_E_SET_SSID:
      OsLog ("exynoswifi: SET_SSID status %lu", Status);
      if (A->State == StateJoining) {
        A->EvJoinDone   = TRUE;
        A->EvJoinStatus = Status;
      }
      break;

    case WLC_E_LINK:
      OsLog ("exynoswifi: link %s (reason %lu)", (Flags & WLC_EVENT_MSG_LINK) ? "up" : "down", Reason);
      if (Flags & WLC_EVENT_MSG_LINK) {
        A->EvLinkUp = TRUE;
      } else if (A->State == StateConnected) {
        A->EvLinkDown = TRUE;
        if (A->EvLinkReason == 0) {
          A->EvLinkReason = DOT11_DISASSOC_REASON_PEER_UNREACHABLE;
        }
      }
      break;

    case WLC_E_DEAUTH:
    case WLC_E_DEAUTH_IND:
    case WLC_E_DISASSOC:
    case WLC_E_DISASSOC_IND:
      OsLog ("exynoswifi: event %lu status %lu reason %lu", Type, Status, Reason);
      if (A->State == StateConnected) {
        A->EvLinkDown   = TRUE;
        A->EvLinkReason = (((Type == WLC_E_DEAUTH) || (Type == WLC_E_DEAUTH_IND)) ? DOT11_DISASSOC_REASON_PEER_DEAUTHENTICATED
                                                                                    : DOT11_DISASSOC_REASON_PEER_DISASSOCIATED) |
                          (Reason & DOT11_ASSOC_STATUS_REASON_CODE_MASK);
      }
      break;

    default:
      break;
  }
}

static VOID
RxHandler (VOID *Context, UINT8 Channel, UINT8 *Payload, UINT32 Length)
{
  PADAPTER  A = Context;

  if (Channel == BCM_CHANNEL_EVENT) {
    HandleEvent (A, Payload, Length);
    return;
  }
  if ((Channel != BCM_CHANNEL_DATA) || (Length < 14) || (Length > ETH_FRAME_MAX)) {
    return;
  }
  if (A->State == StateConnected) {
    IndicateRx (A, Payload, Length);
  } else if ((A->State == StateJoining) && (Payload[12] == 0x88) && (Payload[13] == 0x8E) &&
             (A->HeldCount < MAX_HELD_EAPOL) && (Length <= BCM_EAPOL_MAX)) {
    //
    // The AP's 1/4 can beat our association completion: keep it for then.
    //
    RtlCopyMemory (A->Held[A->HeldCount], Payload, Length);
    A->HeldLength[A->HeldCount] = Length;
    A->HeldCount++;
  } else {
    A->RxDropped++;
  }
}

//
// ---------------------------------------------------------------------------
// Interrupt.
// ---------------------------------------------------------------------------
//

static MINIPORT_ISR                    mpIsr;
static MINIPORT_INTERRUPT_DPC          mpDpc;
static MINIPORT_ENABLE_INTERRUPT       mpEnableInterrupt;
static MINIPORT_DISABLE_INTERRUPT      mpDisableInterrupt;
static MINIPORT_SYNCHRONIZE_INTERRUPT  SyncInterruptDone;

static BOOLEAN
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

static VOID
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

static VOID
mpEnableInterrupt (NDIS_HANDLE Context)
{
  PADAPTER  A = Context;

  WRITE_REGISTER_ULONG ((PULONG)(A->Mmc + 0x24), READ_REGISTER_ULONG ((PULONG)(A->Mmc + 0x24)) | (1u << 16));
}

static VOID
mpDisableInterrupt (NDIS_HANDLE Context)
{
  PADAPTER  A = Context;

  WRITE_REGISTER_ULONG ((PULONG)(A->Mmc + 0x24), READ_REGISTER_ULONG ((PULONG)(A->Mmc + 0x24)) & ~(1u << 16));
}

static BOOLEAN
SyncInterruptDone (NDIS_HANDLE Context)
{
  PADAPTER  A = Context;

  BcmSdioInterruptDone (A->Bcm);
  return TRUE;
}

NDIS_STATUS
HwRegisterInterrupt (PADAPTER A)
{
  NDIS_MINIPORT_INTERRUPT_CHARACTERISTICS  Chars;
  NDIS_STATUS                              Status;

  RtlZeroMemory (&Chars, sizeof (Chars));
  HDR (Chars.Header, NDIS_OBJECT_TYPE_MINIPORT_INTERRUPT, NDIS_MINIPORT_INTERRUPT_REVISION_1,
       NDIS_SIZEOF_MINIPORT_INTERRUPT_CHARACTERISTICS_REVISION_1);
  Chars.InterruptHandler        = mpIsr;
  Chars.InterruptDpcHandler     = mpDpc;
  Chars.EnableInterruptHandler  = mpEnableInterrupt;
  Chars.DisableInterruptHandler = mpDisableInterrupt;
  Chars.MsiSupported            = FALSE;
  Status = NdisMRegisterInterruptEx (A->Handle, A, &Chars, &A->Interrupt);
  if (Status != NDIS_STATUS_SUCCESS) {
    A->Interrupt = NULL;
    OsLog ("exynoswifi: NdisMRegisterInterruptEx failed: 0x%08lx", Status);
  }
  return Status;
}

//
// ---------------------------------------------------------------------------
// Transmit (bus thread).
// ---------------------------------------------------------------------------
//

static PNET_BUFFER_LIST
TakeQueue (PADAPTER A)
{
  PNET_BUFFER_LIST  List;

  NdisAcquireSpinLock (&A->Lock);
  List        = A->TxHead;
  A->TxHead   = NULL;
  A->TxTail   = NULL;
  A->TxQueued = 0;
  NdisReleaseSpinLock (&A->Lock);
  return List;
}

/**
  An 802.11 data frame from nwifi (header, LLC/SNAP, payload) as 802.3 in
  A->TxFrame + 4.  Returns the 802.3 length, 0 to drop the frame.
**/
static ULONG
Dot11ToEth (PADAPTER A, const UCHAR *F, ULONG Length)
{
  ULONG        Fc;
  ULONG        Hdr = 24;
  const UCHAR  *Da;
  const UCHAR  *Sa;
  const UCHAR  *Body;
  ULONG        BodyLen;
  PUCHAR       Out = A->TxFrame + 4;

  if (Length < 24) {
    return 0;
  }
  Fc = Le16 (F);
  if ((((Fc >> 2) & 3) != 2) || ((Fc & 0x0300) == 0x0300) || (Fc & 0x4000)) {
    return 0;                           // not data, WDS, or already encrypted
  }
  if (Fc & 0x0080) {
    Hdr += 2;                           // QoS data
  }
  if (Length < Hdr) {
    return 0;
  }
  if (Fc & 0x0100) {                    // ToDS: BSSID, SA, DA
    Da = F + 16;
    Sa = F + 10;
  } else {
    Da = F + 4;
    Sa = F + 10;
  }
  Body    = F + Hdr;
  BodyLen = Length - Hdr;
  RtlCopyMemory (Out, Da, 6);
  RtlCopyMemory (Out + 6, Sa, 6);
  if ((BodyLen >= 8) && (Body[0] == 0xAA) && (Body[1] == 0xAA) && (Body[2] == 0x03) && (Body[3] == 0) &&
      (Body[4] == 0) && ((Body[5] == 0) || (Body[5] == 0xF8))) {
    if (14 + BodyLen - 8 > ETH_FRAME_MAX) {
      return 0;
    }
    Out[12] = Body[6];
    Out[13] = Body[7];
    RtlCopyMemory (Out + 14, Body + 8, BodyLen - 8);
    return 14 + BodyLen - 8;
  }
  if (14 + BodyLen > ETH_FRAME_MAX) {
    return 0;
  }
  Out[12] = (UCHAR)(BodyLen >> 8);
  Out[13] = (UCHAR)BodyLen;
  RtlCopyMemory (Out + 14, Body, BodyLen);
  return 14 + BodyLen;
}

static VOID
SendList (PADAPTER A, PNET_BUFFER_LIST List)
{
  PNET_BUFFER_LIST  Nbl;
  PNET_BUFFER_LIST  Next;
  PNET_BUFFER       Nb;
  PUCHAR            Data;
  ULONG             Length;
  ULONG             EthLen;
  NDIS_STATUS       Status;

  for (Nbl = List; Nbl != NULL; Nbl = Next) {
    Next = NET_BUFFER_LIST_NEXT_NBL (Nbl);
    NET_BUFFER_LIST_NEXT_NBL (Nbl) = NULL;
    Status = A->Paused ? NDIS_STATUS_PAUSED :
             (A->State == StateConnected) ? NDIS_STATUS_SUCCESS : NDIS_STATUS_MEDIA_DISCONNECTED;
    for (Nb = NET_BUFFER_LIST_FIRST_NB (Nbl); (Nb != NULL) && (Status == NDIS_STATUS_SUCCESS); Nb = NET_BUFFER_NEXT_NB (Nb)) {
      Length = NET_BUFFER_DATA_LENGTH (Nb);
      if ((Length < 24) || (Length > sizeof (A->TxRaw))) {
        Status = NDIS_STATUS_INVALID_LENGTH;
        break;
      }
      Data = NdisGetDataBuffer (Nb, Length, A->TxRaw, 1, 0);
      if (Data == NULL) {
        Status = NDIS_STATUS_RESOURCES;
        break;
      }
      EthLen = Dot11ToEth (A, Data, Length);
      if (EthLen == 0) {
        A->TxErrors++;
        continue;                       // dropped, not an error for the stack
      }
      A->TxFrame[0] = 2 << 4;           // BDC version 2
      A->TxFrame[1] = 0;
      A->TxFrame[2] = 0;
      A->TxFrame[3] = 0;
      if ((A->TxFrame[4 + 12] == 0x88) && (A->TxFrame[4 + 13] == 0x8E)) {
        A->LastEapolTx = KeQueryInterruptTime ();
      }
      if (EFI_ERROR (BcmSdioSend (A->Bcm, BCM_CHANNEL_DATA, A->TxFrame, EthLen + 4))) {
        Status = NDIS_STATUS_FAILURE;
        A->TxErrors++;
      } else {
        A->TxFrames++;
        if (A->TxFrame[4] & 1) {
          A->TxMulticast++;
        }
      }
    }
    NET_BUFFER_LIST_STATUS (Nbl) = Status;
    NdisMSendNetBufferListsComplete (A->Handle, Nbl, 0);
  }
}

//
// ---------------------------------------------------------------------------
// ExtSTA operations (bus thread, from OidOnBus()).
// ---------------------------------------------------------------------------
//

NDIS_STATUS
BusScan (PADAPTER A, PDOT11_SCAN_REQUEST_V2 Request, ULONG Length)
{
  UCHAR       P[72];
  EFI_STATUS  Status;

  UNREFERENCED_PARAMETER (Request);
  UNREFERENCED_PARAMETER (Length);

  if (!A->RadioOn) {
    return NDIS_STATUS_DOT11_POWER_STATE_INVALID;
  }
  if (A->Scanning || (A->State == StateJoining)) {
    return NDIS_STATUS_DOT11_MEDIA_IN_USE;
  }

  //
  // wl_escan_params_t: version 1, action start, sync id, then
  // wl_scan_params_t up to channel_list: broadcast SSID and BSSID, any BSS
  // type, active, firmware default times, every channel.
  //
  RtlZeroMemory (P, sizeof (P));
  Put32 (P + 0, 1);
  Put16 (P + 4, 1);
  Put16 (P + 6, ++A->ScanSyncId);
  RtlCopyMemory (P + 44, mBroadcast, 6);
  P[50] = 2;
  P[51] = 0;
  Put32 (P + 52, 0xFFFFFFFF);
  Put32 (P + 56, 0xFFFFFFFF);
  Put32 (P + 60, 0xFFFFFFFF);
  Put32 (P + 64, 0xFFFFFFFF);
  Put32 (P + 68, 0);

  BcmIoctlSetInt (A->Bcm, WLC_SET_PASSIVE_SCAN, 0);
  A->EvScanDone = FALSE;
  A->Scanning   = TRUE;
  A->ScanStart  = NowMs ();
  Status = BcmIovarSet (A->Bcm, "escan", P, sizeof (P));
  if (EFI_ERROR (Status)) {
    A->Scanning = FALSE;
    OsLog ("exynoswifi: escan failed: 0x%08Ix", Status);
    return NDIS_STATUS_FAILURE;
  }
  OsLog ("exynoswifi: scan started");
  return NDIS_STATUS_SUCCESS;
}

static ULONG
CipherBits (DOT11_CIPHER_ALGORITHM C)
{
  switch (C) {
    case DOT11_CIPHER_ALGO_CCMP:  return AES_ENABLED;
    case DOT11_CIPHER_ALGO_TKIP:  return TKIP_ENABLED;
    default:                      return 0;
  }
}

static ULONG
WpaAuthBits (DOT11_AUTH_ALGORITHM Auth)
{
  switch (Auth) {
    case DOT11_AUTH_ALGO_RSNA_PSK:  return WPA2_AUTH_PSK;
    case DOT11_AUTH_ALGO_RSNA:      return WPA2_AUTH_UNSPECIFIED;
    case DOT11_AUTH_ALGO_WPA_PSK:   return WPA_AUTH_PSK;
    case DOT11_AUTH_ALGO_WPA:       return WPA_AUTH_UNSPECIFIED;
    default:                        return WPA_AUTH_DISABLED;
  }
}

/**
  Stop a join that is going nowhere (or a link) on the firmware side.
**/
static VOID
FirmwareDisassoc (PADAPTER A)
{
  BcmIoctl (A->Bcm, WLC_DISASSOC, TRUE, NULL, 0, 1000);
}

/**
  Fail the connection attempt with Status: association completion (if its
  start was indicated) and connection completion.
**/
static VOID
JoinFailed (PADAPTER A, ULONG Status)
{
  OsLog ("exynoswifi: connect failed (0x%08lx)", Status);
  A->State = StateIdle;
  A->HeldCount = 0;
  if (A->AssocStartIndicated) {
    IndicateAssociationFailure (A, Status);
    A->AssocStartIndicated = FALSE;
  }
  IndicateConnectionCompletion (A, Status);
}

NDIS_STATUS
BusConnect (PADAPTER A)
{
  BSS_ENTRY   *Cand;
  UCHAR       J[52];
  BOOLEAN     Found;
  EFI_STATUS  Status;

  if (!A->RadioOn) {
    return NDIS_STATUS_DOT11_POWER_STATE_INVALID;
  }
  if (A->State != StateIdle) {
    return NDIS_STATUS_INVALID_STATE;
  }
  if ((A->DesiredSsidCount == 0) || (A->DesiredBssType == dot11_BSS_type_independent)) {
    return NDIS_STATUS_INVALID_DATA;
  }

  //
  // The best cached candidate, if any (else the firmware searches by SSID).
  //
  Cand  = (BSS_ENTRY *)ExAllocatePoolWithTag (NonPagedPool, sizeof (BSS_ENTRY), WIFI_TAG);
  Found = (Cand != NULL) && BssPick (A, Cand);

  A->JoinSsid = A->DesiredSsid[0];
  A->AssocStartIndicated = FALSE;
  A->HeldCount   = 0;
  A->EvJoinDone  = FALSE;
  A->EvLinkUp    = FALSE;
  A->EvLinkDown  = FALSE;
  A->EvLinkReason = 0;
  A->LinkSpeed   = 54000000;
  if (Found) {
    RtlCopyMemory (A->Bssid, Cand->Bssid, 6);
    A->ActivePhyId     = Cand->PhyId;
    A->ActiveFrequency = Cand->Frequency;
    A->Rssi            = Cand->Rssi;
  } else {
    RtlCopyMemory (A->Bssid, ((A->DesiredBssidCount > 0) ? A->DesiredBssid[0] : mBroadcast), 6);
    A->Rssi = -60;
  }
  OsLog ("exynoswifi: connect \"%.*s\" %02x:%02x:%02x:%02x:%02x:%02x auth %lu ucast %lu mcast %lu%s",
         (int)A->JoinSsid.uSSIDLength, A->JoinSsid.ucSSID, A->Bssid[0], A->Bssid[1], A->Bssid[2], A->Bssid[3],
         A->Bssid[4], A->Bssid[5], (ULONG)A->AuthAlgorithm, (ULONG)A->UnicastCipher, (ULONG)A->MulticastCipher,
         Found ? "" : " (not in the BSS list)");

  IndicateConnectionStart (A);
  A->State     = StateJoining;
  A->JoinStart = NowMs ();
  if (Found) {
    IndicateAssociationStart (A);
  }

  //
  // Security: the firmware builds the RSN / WPA IE from these; the OS's
  // supplicant does the handshake over the data path (Bcm->WpaActive stays
  // FALSE, so EAPOL frames reach us).
  //
  A->Bcm->WpaActive = FALSE;
  BcmIoctlSetInt (A->Bcm, WLC_SET_INFRA, 1);
  BcmIoctlSetInt (A->Bcm, WLC_SET_AUTH, 0);
  BcmIoctlSetInt (A->Bcm, WLC_SET_WSEC, CipherBits (A->UnicastCipher) | CipherBits (A->MulticastCipher));
  BcmIoctlSetInt (A->Bcm, WLC_SET_WPA_AUTH, WpaAuthBits (A->AuthAlgorithm));

  //
  // wl_join_params_t: SSID, then wl_assoc_params_t (BSSID, chanspec list).
  //
  RtlZeroMemory (J, sizeof (J));
  Put32 (J, A->JoinSsid.uSSIDLength);
  RtlCopyMemory (J + 4, A->JoinSsid.ucSSID, A->JoinSsid.uSSIDLength);
  RtlCopyMemory (J + 36, A->Bssid, 6);
  if (Found) {
    Put32 (J + 44, 1);
    Put16 (J + 48, Cand->Chanspec);
  }
  if (Cand != NULL) {
    ExFreePoolWithTag (Cand, WIFI_TAG);
  }
  Status = BcmIoctl (A->Bcm, WLC_SET_SSID, TRUE, J, sizeof (J), 1000);
  if (EFI_ERROR (Status)) {
    OsLog ("exynoswifi: WLC_SET_SSID failed: 0x%08Ix", Status);
    JoinFailed (A, DOT11_CONNECTION_STATUS_FAILURE);
  }

  //
  // The OID itself succeeded: the outcome comes as indications.
  //
  return NDIS_STATUS_SUCCESS;
}

/**
  Association completion, connection completion, link up, from what the
  firmware says about the association it made.
**/
static VOID
JoinSucceeded (PADAPTER A)
{
  DOT11_ASSOCIATION_COMPLETION_PARAMETERS  *P;
  PUCHAR      B = A->IndBuffer;
  PUCHAR      S = A->Scratch;
  UCHAR       Info[32];
  UCHAR       Bssid[6];
  ULONG       ReqLen;
  ULONG       RespLen;
  ULONG       Skip;
  ULONG       Pos;
  ULONG       BiLen;
  ULONG       IeOff;
  ULONG       IeLen;
  ULONG       Rate;
  BOOLEAN     Reassoc;
  ULONG       Index;

  if (!EFI_ERROR (BcmIoctl (A->Bcm, WLC_GET_BSSID, FALSE, Bssid, 6, 1000))) {
    RtlCopyMemory (A->Bssid, Bssid, 6);
  }
  if (!A->AssocStartIndicated) {
    IndicateAssociationStart (A);
  }

  RtlZeroMemory (B, IND_BUFFER_SIZE);
  P = (DOT11_ASSOCIATION_COMPLETION_PARAMETERS *)B;
  HDR (P->Header, NDIS_OBJECT_TYPE_DEFAULT, DOT11_ASSOCIATION_COMPLETION_PARAMETERS_REVISION_2, sizeof (*P));
  RtlCopyMemory (P->MacAddr, A->Bssid, 6);
  P->uStatus = DOT11_ASSOC_STATUS_SUCCESS;
  Pos = (sizeof (*P) + 7) & ~7u;

  //
  // assoc_info: req_len, resp_len, flags, then the fixed fields of the
  // request (capability, listen interval, current AP) and of the response
  // (capability, status, AID).
  //
  RtlZeroMemory (Info, sizeof (Info));
  BcmIovarGet (A->Bcm, "assoc_info", Info, sizeof (Info));
  ReqLen  = Le32 (Info + 0);
  RespLen = Le32 (Info + 4);
  Reassoc = (Le32 (Info + 8) & 1) != 0;
  A->Aid  = (USHORT)(Le16 (Info + 26) & 0x3FFF);

  //
  // Association request frame body: fixed fields + the firmware's IEs (the
  // RSN IE among them is what the OS's 2/4 must repeat).
  //
  P->bReAssocReq    = Reassoc;
  P->uAssocReqOffset = Pos;
  B[Pos + 0] = Info[12];
  B[Pos + 1] = Info[13];
  B[Pos + 2] = Info[14];
  B[Pos + 3] = Info[15];
  Pos += 4;
  if (Reassoc) {
    RtlCopyMemory (B + Pos, Info + 16, 6);
    Pos += 6;
  }
  if ((ReqLen > 0) && (ReqLen <= SCRATCH_SIZE)) {
    RtlZeroMemory (S, SCRATCH_SIZE);
    if (!EFI_ERROR (BcmIovarGet (A->Bcm, "assoc_req_ies", S, ReqLen))) {
      Skip = 0;
      if (!IeChainOk (S, ReqLen)) {
        Skip = (ReqLen > 10 && IeChainOk (S + 10, ReqLen - 10) && Reassoc) ? 10 :
               (ReqLen > 4 && IeChainOk (S + 4, ReqLen - 4)) ? 4 : 0;
      }
      RtlCopyMemory (B + Pos, S + Skip, ReqLen - Skip);
      Pos += ReqLen - Skip;
    }
  }
  P->uAssocReqSize = Pos - P->uAssocReqOffset;
  Pos = (Pos + 7) & ~7u;

  //
  // Association response frame body.
  //
  P->bReAssocResp     = Reassoc;
  P->uAssocRespOffset = Pos;
  RtlCopyMemory (B + Pos, Info + 22, 6);
  Pos += 6;
  if ((RespLen > 0) && (RespLen <= SCRATCH_SIZE)) {
    RtlZeroMemory (S, SCRATCH_SIZE);
    if (!EFI_ERROR (BcmIovarGet (A->Bcm, "assoc_resp_ies", S, RespLen))) {
      Skip = (!IeChainOk (S, RespLen) && (RespLen > 6) && IeChainOk (S + 6, RespLen - 6)) ? 6 : 0;
      RtlCopyMemory (B + Pos, S + Skip, RespLen - Skip);
      Pos += RespLen - Skip;
    }
  }
  P->uAssocRespSize = Pos - P->uAssocRespOffset;
  Pos = (Pos + 7) & ~7u;

  //
  // Beacon frame body (timestamp, interval, capability, IEs) from the
  // current BSS info, which also refreshes the cache entry.
  //
  P->uBeaconOffset = Pos;
  RtlZeroMemory (S, SCRATCH_SIZE);
  Put32 (S, SCRATCH_SIZE);
  if (!EFI_ERROR (BcmIoctl (A->Bcm, WLC_GET_BSS_INFO, FALSE, S, SCRATCH_SIZE, 1000))) {
    BiLen = Le32 (S + 4 + 4);
    if ((BiLen >= 128) && (BiLen <= SCRATCH_SIZE - 4)) {
      BssUpdate (A, S + 4, BiLen);
      IeOff = Le16 (S + 4 + 116);
      IeLen = Le32 (S + 4 + 120);
      if ((IeOff >= 128) && (IeOff <= BiLen) && (IeLen <= BiLen - IeOff) && (Pos + 12 + IeLen < IND_BUFFER_SIZE - 64)) {
        RtlZeroMemory (B + Pos, 8);                     // timestamp
        RtlCopyMemory (B + Pos + 8, S + 4 + 14, 2);     // beacon interval
        RtlCopyMemory (B + Pos + 10, S + 4 + 16, 2);    // capability
        RtlCopyMemory (B + Pos + 12, S + 4 + IeOff, IeLen);
        Pos += 12 + IeLen;
        A->Rssi = (SHORT)Le16 (S + 4 + 78);
        DecodeChannel (Le16 (S + 4 + 72), S[4 + 88], &Index, &A->ActiveFrequency, &A->ActivePhyId);
      }
    }
  }
  P->uBeaconSize = Pos - P->uBeaconOffset;
  Pos = (Pos + 7) & ~7u;

  //
  // nwifi may not have told us the group cipher: take it from the RSN IE
  // the AP advertises (00-0F-AC:2 TKIP, :4 CCMP), else follow the pairwise one.
  //
  if (A->MulticastCipher == DOT11_CIPHER_ALGO_NONE && A->UnicastCipher != DOT11_CIPHER_ALGO_NONE) {
    ULONG  BeaconIe = P->uBeaconOffset + 12;
    ULONG  End = P->uBeaconOffset + P->uBeaconSize;
    A->MulticastCipher = A->UnicastCipher;
    while (BeaconIe + 2 <= End && BeaconIe + 2 + B[BeaconIe + 1] <= End) {
      if (B[BeaconIe] == 0x30 && B[BeaconIe + 1] >= 8 &&
          B[BeaconIe + 4] == 0x00 && B[BeaconIe + 5] == 0x0F && B[BeaconIe + 6] == 0xAC) {
        if (B[BeaconIe + 7] == 2) {
          A->MulticastCipher = DOT11_CIPHER_ALGO_TKIP;
        } else if (B[BeaconIe + 7] == 4) {
          A->MulticastCipher = DOT11_CIPHER_ALGO_CCMP;
        }
        break;
      }
      BeaconIe += 2 + B[BeaconIe + 1];
    }
    OsLog ("exynoswifi: group cipher %lu (from the AP)", (ULONG)A->MulticastCipher);
  }
  P->AuthAlgo        = A->AuthAlgorithm;
  P->UnicastCipher   = A->UnicastCipher;
  P->MulticastCipher = A->MulticastCipher;
  P->uActivePhyListOffset = Pos;
  *(ULONG *)(B + Pos) = A->ActivePhyId;
  Pos += sizeof (ULONG);
  P->uActivePhyListSize = sizeof (ULONG);
  P->bFourAddressSupported = FALSE;
  P->bPortAuthorized = FALSE;            // the OS authorizes the port
  P->ucActiveQoSProtocol = 0;
  P->DSInfo = DOT11_DS_CHANGED;
  P->MulticastMgmtCipher = DOT11_CIPHER_ALGO_NONE;

  Rate = 0;
  if (!EFI_ERROR (BcmIoctl (A->Bcm, WLC_GET_RATE, FALSE, &Rate, sizeof (Rate), 1000)) && (Rate != 0)) {
    A->LinkSpeed = (ULONG64)Rate * 500000;
  }

  OsLog ("exynoswifi: associated with %02x:%02x:%02x:%02x:%02x:%02x (%lu MHz, RSSI %ld, %lu Mbps, req %lu resp %lu beacon %lu)",
         A->Bssid[0], A->Bssid[1], A->Bssid[2], A->Bssid[3], A->Bssid[4], A->Bssid[5], A->ActiveFrequency, A->Rssi,
         (ULONG)(A->LinkSpeed / 1000000), P->uAssocReqSize, P->uAssocRespSize, P->uBeaconSize);

  //
  // Connected before the indication: the OS may start the 4-way handshake
  // (send 2/4) from inside it.
  //
  A->State        = StateConnected;
  A->ConnectedAt  = NowMs ();
  A->LastRssiPoll = A->ConnectedAt;
  A->EvLinkDown   = FALSE;
  A->EvLinkReason = 0;
  WifiIndicate (A, NDIS_STATUS_DOT11_ASSOCIATION_COMPLETION, B, Pos);
  A->AssocStartIndicated = FALSE;
  IndicateConnectionCompletion (A, DOT11_CONNECTION_STATUS_SUCCESS);
  WifiIndicateLink (A, TRUE);

  for (Index = 0; Index < A->HeldCount; Index++) {
    IndicateRx (A, A->Held[Index], A->HeldLength[Index]);
  }
  A->HeldCount = 0;
}

/**
  Leave the current association (Connected) or attempt (Joining).
**/
NDIS_STATUS
BusDisconnect (PADAPTER A, ULONG Reason)
{
  if (A->State == StateJoining) {
    FirmwareDisassoc (A);
    JoinFailed (A, DOT11_CONNECTION_STATUS_CANCELLED);
  } else if (A->State == StateConnected) {
    FirmwareDisassoc (A);
    A->State = StateIdle;
    OsLog ("exynoswifi: disconnected (0x%08lx)", Reason);
    IndicateDisassociation (A, Reason);
    WifiIndicateLink (A, FALSE);
  }
  A->EvLinkDown = FALSE;
  A->EvLinkUp   = FALSE;
  A->EvJoinDone = FALSE;
  return NDIS_STATUS_SUCCESS;
}

NDIS_STATUS
BusReset (PADAPTER A, BOOLEAN SetDefaultMib)
{
  ULONG  Index;

  if (A->Scanning) {
    IndicateScanConfirm (A, NDIS_STATUS_REQUEST_ABORTED);
  }
  BusDisconnect (A, DOT11_ASSOC_STATUS_DISASSOCIATED_BY_RESET);
  if (A->Up) {
    for (Index = 0; Index < MAX_DEFAULT_KEYS; Index++) {
      BcmSetKey (A->Bcm, Index, NULL, 0, BCM_CRYPTO_ALGO_OFF, NULL, NULL);
    }
  }
  if (SetDefaultMib) {
    WifiResetMib (A);
  }
  return NDIS_STATUS_SUCCESS;
}

NDIS_STATUS
BusRadio (PADAPTER A, BOOLEAN On)
{
  DOT11_PHY_STATE_PARAMETERS  P;
  ULONG                       Phy;
  EFI_STATUS                  Status;

  if (On == A->RadioOn) {
    return NDIS_STATUS_SUCCESS;
  }
  if (!On) {
    if (A->Scanning) {
      IndicateScanConfirm (A, NDIS_STATUS_REQUEST_ABORTED);
    }
    BusDisconnect (A, DOT11_DISASSOC_REASON_RADIO_OFF);
    Status = BcmIoctl (A->Bcm, WLC_DOWN, TRUE, NULL, 0, 2000);
  } else {
    Status = BcmIoctl (A->Bcm, WLC_UP, TRUE, NULL, 0, 2000);
  }
  OsLog ("exynoswifi: radio %s - 0x%08Ix", On ? "on" : "off", Status);
  if (EFI_ERROR (Status)) {
    return NDIS_STATUS_FAILURE;
  }
  A->RadioOn = On;
  for (Phy = 0; Phy < PHY_COUNT; Phy++) {
    RtlZeroMemory (&P, sizeof (P));
    HDR (P.Header, NDIS_OBJECT_TYPE_DEFAULT, DOT11_PHY_STATE_PARAMETERS_REVISION_1, sizeof (P));
    P.uPhyId            = Phy;
    P.bHardwarePhyState = TRUE;
    P.bSoftwarePhyState = On;
    WifiIndicate (A, NDIS_STATUS_DOT11_PHY_STATE_CHANGED, &P, sizeof (P));
  }
  return NDIS_STATUS_SUCCESS;
}

NDIS_STATUS
BusSetKey (PADAPTER A, KEY_WORK *K)
{
  ULONG       Algo;
  ULONGLONG   Since;
  EFI_STATUS  Status;

  switch (K->Algorithm) {
    case DOT11_CIPHER_ALGO_CCMP:   Algo = BCM_CRYPTO_ALGO_AES_CCM; break;
    case DOT11_CIPHER_ALGO_TKIP:   Algo = BCM_CRYPTO_ALGO_TKIP; break;
    case DOT11_CIPHER_ALGO_WEP40:  Algo = BCM_CRYPTO_ALGO_WEP1; break;
    case DOT11_CIPHER_ALGO_WEP104: Algo = BCM_CRYPTO_ALGO_WEP128; break;
    case DOT11_CIPHER_ALGO_WEP:    Algo = (K->Length == 5) ? BCM_CRYPTO_ALGO_WEP1 : BCM_CRYPTO_ALGO_WEP128; break;
    default:
      if (!K->Delete) {
        return NDIS_STATUS_INVALID_DATA;
      }
      Algo = BCM_CRYPTO_ALGO_OFF;
      break;
  }

  //
  // The pairwise key arrives right after the OS queued 4/4: give that frame
  // time to leave the firmware unencrypted (BcmWpa.c waits the same way).
  //
  if (K->Pairwise && !K->Delete && (A->LastEapolTx != 0)) {
    Since = (KeQueryInterruptTime () - A->LastEapolTx) / 10000;
    if (Since < EAPOL_SETTLE_MS) {
      gBS->Stall ((UINTN)(EAPOL_SETTLE_MS - Since) * 1000);
    }
  }

  Status = BcmSetKey (A->Bcm, K->Pairwise ? 0 : K->Index, K->Delete ? NULL : K->Key, K->Delete ? 0 : K->Length, Algo,
                      (K->HaveRsc && !K->Delete) ? K->Rsc : NULL, K->Pairwise ? K->Mac : NULL);
  OsLog ("exynoswifi: %s key %lu %s (algo %lu, %lu bytes) - 0x%08Ix", K->Pairwise ? "pairwise" : "group", K->Index,
         K->Delete ? "deleted" : "set", (ULONG)K->Algorithm, K->Length, Status);
  return EFI_ERROR (Status) ? NDIS_STATUS_FAILURE : NDIS_STATUS_SUCCESS;
}

//
// ---------------------------------------------------------------------------
// Bus thread.
// ---------------------------------------------------------------------------
//

/**
  What the RX handler noted, plus the timeouts.
**/
static VOID
ProcessEvents (PADAPTER A)
{
  ULONG  Reason;
  ULONG  Status;
  LONG   Rssi;
  UCHAR  Scb[12];

  if (A->Scanning && (A->EvScanDone || (NowMs () - A->ScanStart > SCAN_TIMEOUT_MS))) {
    //
    // An aborted or timed-out scan still confirms with success: what was
    // found is in the BSS list.
    //
    Status = NDIS_STATUS_SUCCESS;
    if (!A->EvScanDone) {
      OsLog ("exynoswifi: scan timed out");
    } else if (A->EvScanStatus != WLC_E_STATUS_SUCCESS) {
      OsLog ("exynoswifi: escan ended with status %lu", A->EvScanStatus);
    }
    A->EvScanDone = FALSE;
    IndicateScanConfirm (A, Status);
  }

  if (A->State == StateJoining) {
    if (A->EvJoinDone && (A->EvJoinStatus != WLC_E_STATUS_SUCCESS)) {
      A->EvJoinDone = FALSE;
      FirmwareDisassoc (A);
      JoinFailed (A, (A->EvJoinStatus == 3) ? DOT11_CONNECTION_STATUS_CANDIDATE_LIST_EXHAUSTED
                                            : DOT11_CONNECTION_STATUS_FAILURE);
    } else if (A->EvJoinDone || A->EvLinkUp) {
      A->EvJoinDone = FALSE;
      A->EvLinkUp   = FALSE;
      JoinSucceeded (A);
    } else if (NowMs () - A->JoinStart > CONNECT_TIMEOUT_MS) {
      FirmwareDisassoc (A);
      JoinFailed (A, DOT11_ASSOC_STATUS_UNREACHABLE);
    }
  }

  if ((A->State == StateConnected) && A->EvLinkDown) {
    Reason = (A->EvLinkReason != 0) ? A->EvLinkReason : DOT11_DISASSOC_REASON_PEER_UNREACHABLE;
    A->EvLinkDown   = FALSE;
    A->EvLinkReason = 0;
    A->State        = StateIdle;
    OsLog ("exynoswifi: link lost (0x%08lx)", Reason);
    IndicateDisassociation (A, Reason);
    WifiIndicateLink (A, FALSE);
  }
  A->EvLinkUp = FALSE;

  if ((A->State == StateConnected) && (NowMs () - A->LastRssiPoll > RSSI_POLL_MS)) {
    A->LastRssiPoll = NowMs ();
    RtlZeroMemory (Scb, sizeof (Scb));                  // scb_val_t: val, ea
    RtlCopyMemory (Scb + 4, A->Bssid, 6);
    Rssi = EFI_ERROR (BcmIoctl (A->Bcm, WLC_GET_RSSI, FALSE, Scb, sizeof (Scb), 1000)) ? 0 : (LONG)Le32 (Scb);
    if ((Rssi < 0) && (Rssi > -128)) {
      A->Rssi = Rssi;
      IndicateLinkQuality (A);
    }
  }
}

static VOID
ServeOid (PADAPTER A)
{
  PNDIS_OID_REQUEST  Req;
  NDIS_STATUS        Status;

  NdisAcquireSpinLock (&A->Lock);
  Req           = A->PendingOid;
  A->PendingOid = NULL;
  NdisReleaseSpinLock (&A->Lock);
  if (Req != NULL) {
    Status = A->Stop ? NDIS_STATUS_REQUEST_ABORTED : OidOnBus (A, Req);
    NdisMOidRequestComplete (A->Handle, Req, Status);
  }
}

static VOID
BusThread (PVOID Context)
{
  PADAPTER          A = Context;
  LARGE_INTEGER     Timeout;
  PNET_BUFFER_LIST  List;
  ULONG             Round;

  while (!A->Stop) {
    Timeout.QuadPart = -(LONGLONG)((A->Interrupts != 0) ? SAFETY_POLL_MS : FALLBACK_POLL_MS) * 10000;
    if (A->Scanning || (A->State == StateJoining)) {
      Timeout.QuadPart = -(LONGLONG)200 * 10000;        // timeouts are checked here
    }
    KeWaitForSingleObject (&A->Wake, Executive, KernelMode, FALSE, &Timeout);
    if (A->Stop) {
      break;
    }

    for (Round = 0; (Round < DRAIN_ROUNDS) && (BcmSdioPoll (A->Bcm) > 0); Round++) {
    }
    if (InterlockedExchange (&A->IrqPending, 0) != 0) {
      NdisMSynchronizeWithInterruptEx (A->Interrupt, 0, SyncInterruptDone, A);
    }
    ProcessEvents (A);

    //
    // Frames before OIDs: a queued 4/4 leaves before the key it precedes.
    //
    List = TakeQueue (A);
    if (List != NULL) {
      SendList (A, List);
    }
    ServeOid (A);
    ProcessEvents (A);
  }

  ServeOid (A);
  List = TakeQueue (A);
  if (List != NULL) {
    A->Paused = TRUE;
    SendList (A, List);
  }
  PsTerminateSystemThread (STATUS_SUCCESS);
}

NDIS_STATUS
HwStartThread (PADAPTER A)
{
  OBJECT_ATTRIBUTES  Attr;
  HANDLE             ThreadHandle;
  NTSTATUS           Nt;

  //
  // The card interrupt before the thread exists: from then on only the
  // thread touches the chip.
  //
  BcmSdioEnableInterrupts (A->Bcm);
  InitializeObjectAttributes (&Attr, NULL, OBJ_KERNEL_HANDLE, NULL, NULL);
  Nt = PsCreateSystemThread (&ThreadHandle, THREAD_ALL_ACCESS, &Attr, NULL, NULL, BusThread, A);
  if (!NT_SUCCESS (Nt)) {
    return NDIS_STATUS_RESOURCES;
  }
  ObReferenceObjectByHandle (ThreadHandle, THREAD_ALL_ACCESS, NULL, KernelMode, (PVOID *)&A->Thread, NULL);
  ZwClose (ThreadHandle);
  return NDIS_STATUS_SUCCESS;
}

VOID
HwStopThread (PADAPTER A)
{
  A->Stop = TRUE;
  KeSetEvent (&A->Wake, 0, FALSE);
  if (A->Thread != NULL) {
    KeWaitForSingleObject (A->Thread, Executive, KernelMode, FALSE, NULL);
    ObDereferenceObject (A->Thread);
    A->Thread = NULL;
  }
}

//
// ---------------------------------------------------------------------------
// Bring-up.
// ---------------------------------------------------------------------------
//

NDIS_STATUS
HwStart (PADAPTER A)
{
  PUCHAR      Firmware;
  PUCHAR      Nvram;
  ULONG       FirmwareSize;
  ULONG       NvramSize;
  ULONG       CiuHz;
  CHAR8       Ver[128];
  UCHAR       Events[16];               // WL_EVENTING_MASK_LEN
  ULONG       Index;
  EFI_STATUS  Status;
  NTSTATUS    Nt;

  A->Bcm->RxHandler = RxHandler;
  A->Bcm->RxContext = A;

  Nt = LoadFile (L"\\SystemRoot\\System32\\drivers\\bcmdhd_sta.bin", &Firmware, &FirmwareSize);
  if (!NT_SUCCESS (Nt)) {
    OsLog ("exynoswifi: cannot read bcmdhd_sta.bin: 0x%08lx", Nt);
    return NDIS_STATUS_FILE_NOT_FOUND;
  }
  Nt = LoadFile (L"\\SystemRoot\\System32\\drivers\\nvram_net.txt", &Nvram, &NvramSize);
  if (!NT_SUCCESS (Nt)) {
    OsLog ("exynoswifi: cannot read nvram_net.txt: 0x%08lx", Nt);
    ExFreePoolWithTag (Firmware, WIFI_TAG);
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
  ExFreePoolWithTag (Firmware, WIFI_TAG);
  ExFreePoolWithTag (Nvram, WIFI_TAG);
  if (!EFI_ERROR (Status)) {
    Status = BcmSdioBusInit (A->Bcm);
  }
  if (!EFI_ERROR (Status)) {
    RtlZeroMemory (Ver, sizeof (Ver));
    BcmIovarGet (A->Bcm, "ver", Ver, sizeof (Ver) - 1);
    OsLog ("exynoswifi: firmware %s", Ver);
    Status = BcmIovarGet (A->Bcm, "cur_etheraddr", A->Mac, sizeof (A->Mac));
  }
  if (!EFI_ERROR (Status)) {
    OsLog ("exynoswifi: MAC %02x:%02x:%02x:%02x:%02x:%02x", A->Mac[0], A->Mac[1], A->Mac[2], A->Mac[3], A->Mac[4], A->Mac[5]);

    //
    // Events (escan results on top of the join / link set), one frame per
    // read, radio always up for scans, no silent firmware roaming (roams
    // are not reported to the OS), no power save.
    //
    RtlZeroMemory (Events, sizeof (Events));
    BcmIovarGet (A->Bcm, "event_msgs", Events, sizeof (Events));
    for (Index = 0; Index < ARRAY_SIZE (mEvents); Index++) {
      Events[mEvents[Index] / 8] |= (UCHAR)(1u << (mEvents[Index] % 8));
    }
    OsLog ("exynoswifi: event_msgs - 0x%08Ix", BcmIovarSet (A->Bcm, "event_msgs", Events, sizeof (Events)));
    BcmIovarSetInt (A->Bcm, "bus:rxglom", 0);
    BcmIovarSetInt (A->Bcm, "mpc", 0);
    BcmIovarSetInt (A->Bcm, "roam_off", 1);
    BcmIoctlSetInt (A->Bcm, WLC_SET_INFRA, 1);
    BcmIoctlSetInt (A->Bcm, WLC_SET_PM, 0);
    Status = BcmIoctl (A->Bcm, WLC_UP, TRUE, NULL, 0, 2000);
  }
  if (EFI_ERROR (Status)) {
    OsLog ("exynoswifi: chip bring-up failed: 0x%08Ix", Status);
    BcmLogConsole (A->Bcm);
    WlanPower (A, FALSE);
    return NDIS_STATUS_ADAPTER_NOT_FOUND;
  }
  A->Up      = TRUE;
  A->RadioOn = TRUE;
  return NDIS_STATUS_SUCCESS;
}

VOID
HwStop (PADAPTER A)
{
  if (A->Up) {
    BcmIoctl (A->Bcm, WLC_DOWN, TRUE, NULL, 0, 500);
    A->Up = FALSE;
  }
  WlanPower (A, FALSE);
  OsLog ("exynoswifi: stopped after %lu interrupts, %lu frames, %lu superframes, %lu rx resyncs; chip powered off",
         A->Interrupts, A->Bcm->Frames, A->Bcm->Gloms, A->Bcm->RxFails);
}
