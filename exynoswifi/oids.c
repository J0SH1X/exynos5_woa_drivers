/** @file
  exynoswifi OIDs: the general NDIS set, the 802.11 MIB and the ExtSTA
  set (windot11.h).  Most are MIB values kept in the adapter; the ones that
  need the chip (scan, connect, disconnect, reset, keys, radio) run on the
  bus thread (OidNeedsBus / OidOnBus, completed asynchronously).  Every OID
  that is not handled is logged: that list is what Native Wi-Fi still
  misses.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include "wifi.h"

const NDIS_OID  gSupportedOids[] = {
  OID_GEN_SUPPORTED_LIST, OID_GEN_HARDWARE_STATUS, OID_GEN_MEDIA_SUPPORTED, OID_GEN_MEDIA_IN_USE,
  OID_GEN_MAXIMUM_FRAME_SIZE, OID_GEN_MAXIMUM_TOTAL_SIZE, OID_GEN_LINK_SPEED, OID_GEN_TRANSMIT_BLOCK_SIZE,
  OID_GEN_RECEIVE_BLOCK_SIZE, OID_GEN_VENDOR_ID, OID_GEN_VENDOR_DESCRIPTION, OID_GEN_VENDOR_DRIVER_VERSION,
  OID_GEN_CURRENT_PACKET_FILTER, OID_GEN_CURRENT_LOOKAHEAD, OID_GEN_DRIVER_VERSION, OID_GEN_MAXIMUM_SEND_PACKETS,
  OID_GEN_XMIT_OK, OID_GEN_RCV_OK, OID_GEN_XMIT_ERROR, OID_GEN_RCV_ERROR, OID_GEN_RCV_NO_BUFFER,
  OID_GEN_STATISTICS, OID_GEN_INTERRUPT_MODERATION, OID_GEN_LINK_PARAMETERS,

  OID_DOT11_OPERATION_MODE_CAPABILITY, OID_DOT11_CURRENT_OPERATION_MODE, OID_DOT11_ATIM_WINDOW,
  OID_DOT11_SCAN_REQUEST, OID_DOT11_RESET_REQUEST, OID_DOT11_NIC_POWER_STATE, OID_DOT11_MPDU_MAX_LENGTH,
  OID_DOT11_STATION_ID, OID_DOT11_OPERATIONAL_RATE_SET, OID_DOT11_BEACON_PERIOD, OID_DOT11_DTIM_PERIOD,
  OID_DOT11_MAC_ADDRESS, OID_DOT11_CURRENT_ADDRESS, OID_DOT11_PERMANENT_ADDRESS, OID_DOT11_MULTICAST_LIST,
  OID_DOT11_MAXIMUM_LIST_SIZE, OID_DOT11_RTS_THRESHOLD, OID_DOT11_SHORT_RETRY_LIMIT, OID_DOT11_LONG_RETRY_LIMIT,
  OID_DOT11_FRAGMENTATION_THRESHOLD, OID_DOT11_MAX_TRANSMIT_MSDU_LIFETIME, OID_DOT11_MAX_RECEIVE_LIFETIME,
  OID_DOT11_SUPPORTED_PHY_TYPES, OID_DOT11_CURRENT_REG_DOMAIN, OID_DOT11_TEMP_TYPE, OID_DOT11_DIVERSITY_SUPPORT,
  OID_DOT11_SUPPORTED_POWER_LEVELS, OID_DOT11_CURRENT_TX_POWER_LEVEL, OID_DOT11_CURRENT_CHANNEL,
  OID_DOT11_CURRENT_FREQUENCY, OID_DOT11_FREQUENCY_BANDS_SUPPORTED, OID_DOT11_SUPPORTED_DATA_RATES_VALUE,
  OID_DOT11_REG_DOMAINS_SUPPORT_VALUE, OID_DOT11_MULTI_DOMAIN_CAPABILITY_IMPLEMENTED,
  OID_DOT11_SHORT_PREAMBLE_OPTION_IMPLEMENTED, OID_DOT11_PBCC_OPTION_IMPLEMENTED,
  OID_DOT11_CHANNEL_AGILITY_PRESENT, OID_DOT11_CHANNEL_AGILITY_ENABLED, OID_DOT11_ERP_PBCC_OPTION_IMPLEMENTED,
  OID_DOT11_ERP_PBCC_OPTION_ENABLED, OID_DOT11_DSSS_OFDM_OPTION_IMPLEMENTED, OID_DOT11_DSSS_OFDM_OPTION_ENABLED,
  OID_DOT11_SHORT_SLOT_TIME_OPTION_IMPLEMENTED, OID_DOT11_SHORT_SLOT_TIME_OPTION_ENABLED,
  OID_DOT11_SUPPORTED_OFDM_FREQUENCY_LIST, OID_DOT11_SUPPORTED_DSSS_CHANNEL_LIST,

  OID_DOT11_AUTO_CONFIG_ENABLED, OID_DOT11_ENUM_BSS_LIST, OID_DOT11_FLUSH_BSS_LIST, OID_DOT11_POWER_MGMT_REQUEST,
  OID_DOT11_DESIRED_SSID_LIST, OID_DOT11_EXCLUDED_MAC_ADDRESS_LIST, OID_DOT11_DESIRED_BSSID_LIST,
  OID_DOT11_DESIRED_BSS_TYPE, OID_DOT11_CONNECT_REQUEST, OID_DOT11_EXCLUDE_UNENCRYPTED, OID_DOT11_STATISTICS,
  OID_DOT11_PRIVACY_EXEMPTION_LIST, OID_DOT11_ENABLED_AUTHENTICATION_ALGORITHM,
  OID_DOT11_SUPPORTED_UNICAST_ALGORITHM_PAIR, OID_DOT11_ENABLED_UNICAST_CIPHER_ALGORITHM,
  OID_DOT11_SUPPORTED_MULTICAST_ALGORITHM_PAIR, OID_DOT11_ENABLED_MULTICAST_CIPHER_ALGORITHM,
  OID_DOT11_CIPHER_DEFAULT_KEY_ID, OID_DOT11_CIPHER_DEFAULT_KEY, OID_DOT11_CIPHER_KEY_MAPPING_KEY,
  OID_DOT11_ENUM_ASSOCIATION_INFO, OID_DOT11_DISCONNECT_REQUEST, OID_DOT11_UNICAST_USE_GROUP_ENABLED,
  OID_DOT11_HARDWARE_PHY_STATE, OID_DOT11_DESIRED_PHY_LIST, OID_DOT11_CURRENT_PHY_ID,
  OID_DOT11_MEDIA_STREAMING_ENABLED, OID_DOT11_UNREACHABLE_DETECTION_THRESHOLD, OID_DOT11_ACTIVE_PHY_LIST,
  OID_DOT11_EXTSTA_CAPABILITY, OID_DOT11_DATA_RATE_MAPPING_TABLE, OID_DOT11_SUPPORTED_COUNTRY_OR_REGION_STRING,
  OID_DOT11_DESIRED_COUNTRY_OR_REGION_STRING, OID_DOT11_IBSS_PARAMS, OID_DOT11_SAFE_MODE_ENABLED,
  OID_DOT11_HIDDEN_NETWORK_ENABLED, OID_DOT11_ASSOCIATION_PARAMS,
};
const ULONG  gSupportedOidsSize = sizeof (gSupportedOids);

#define LIST_ARRAY_OFFSET  12           // Header, uNumOfEntries, uTotalNumOfEntries

static ULONG
Rd32 (const VOID *P)
{
  ULONG  V;

  RtlCopyMemory (&V, P, 4);
  return V;
}

static USHORT
Rd16 (const VOID *P)
{
  USHORT  V;

  RtlCopyMemory (&V, P, 2);
  return V;
}

//
// ---------------------------------------------------------------------------
// Answer helpers.
// ---------------------------------------------------------------------------
//

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

static NDIS_STATUS
AnswerUlong (PNDIS_OID_REQUEST Req, ULONG Value)
{
  return Answer (Req, &Value, sizeof (Value));
}

static NDIS_STATUS
AnswerBool (PNDIS_OID_REQUEST Req, BOOLEAN Value)
{
  return Answer (Req, &Value, sizeof (Value));
}

/**
  A windot11 list (NDIS header, uNumOfEntries, uTotalNumOfEntries, array at
  ArrayOffset): as many entries as fit, BUFFER_OVERFLOW when not all do.
**/
static NDIS_STATUS
AnswerList (PNDIS_OID_REQUEST Req, UCHAR Revision, ULONG HeaderSize, ULONG ArrayOffset, const VOID *Elems,
            ULONG Count, ULONG ElemSize)
{
  ULONG               Length = Req->DATA.QUERY_INFORMATION.InformationBufferLength;
  PUCHAR              Out    = Req->DATA.QUERY_INFORMATION.InformationBuffer;
  ULONG               Need   = ArrayOffset + Count * ElemSize;
  ULONG               Fit;
  NDIS_OBJECT_HEADER  H;

  if (Length < ArrayOffset) {
    Req->DATA.QUERY_INFORMATION.BytesNeeded = Need;
    return NDIS_STATUS_BUFFER_TOO_SHORT;
  }
  Fit = (ElemSize == 0) ? Count : MIN (Count, (Length - ArrayOffset) / ElemSize);
  RtlZeroMemory (Out, ArrayOffset);
  HDR (H, NDIS_OBJECT_TYPE_DEFAULT, Revision, HeaderSize);
  RtlCopyMemory (Out, &H, sizeof (H));
  RtlCopyMemory (Out + 4, &Fit, 4);
  RtlCopyMemory (Out + 8, &Count, 4);
  if (Fit > 0) {
    RtlCopyMemory (Out + ArrayOffset, Elems, Fit * ElemSize);
  }
  Req->DATA.QUERY_INFORMATION.BytesWritten = ArrayOffset + Fit * ElemSize;
  if (Fit < Count) {
    Req->DATA.QUERY_INFORMATION.BytesNeeded = Need;
    return NDIS_STATUS_BUFFER_OVERFLOW;
  }
  return NDIS_STATUS_SUCCESS;
}

/**
  A windot11 list from a set request: header type, entry count against Max,
  length.  *Elems / *Count on success.
**/
static NDIS_STATUS
TakeList (PNDIS_OID_REQUEST Req, ULONG ArrayOffset, ULONG ElemSize, ULONG Max, const UCHAR **Elems, ULONG *Count)
{
  ULONG        Length = Req->DATA.SET_INFORMATION.InformationBufferLength;
  const UCHAR  *In    = Req->DATA.SET_INFORMATION.InformationBuffer;
  ULONG        N;

  if (Length < ArrayOffset) {
    Req->DATA.SET_INFORMATION.BytesNeeded = ArrayOffset;
    return NDIS_STATUS_INVALID_LENGTH;
  }
  if (In[0] != NDIS_OBJECT_TYPE_DEFAULT) {
    return NDIS_STATUS_INVALID_DATA;
  }
  N = Rd32 (In + 4);
  if (N > Max) {
    return NDIS_STATUS_INVALID_LENGTH;
  }
  if (Length < ArrayOffset + N * ElemSize) {
    Req->DATA.SET_INFORMATION.BytesNeeded = ArrayOffset + N * ElemSize;
    return NDIS_STATUS_INVALID_LENGTH;
  }
  Req->DATA.SET_INFORMATION.BytesRead = ArrayOffset + N * ElemSize;
  *Elems = In + ArrayOffset;
  *Count = N;
  return NDIS_STATUS_SUCCESS;
}

static NDIS_STATUS
TakeValue (PNDIS_OID_REQUEST Req, VOID *Out, ULONG Size)
{
  if (Req->DATA.SET_INFORMATION.InformationBufferLength < Size) {
    Req->DATA.SET_INFORMATION.BytesNeeded = Size;
    return NDIS_STATUS_INVALID_LENGTH;
  }
  RtlCopyMemory (Out, Req->DATA.SET_INFORMATION.InformationBuffer, Size);
  Req->DATA.SET_INFORMATION.BytesRead = Size;
  return NDIS_STATUS_SUCCESS;
}

static BOOLEAN
AuthSupported (DOT11_AUTH_ALGORITHM Auth)
{
  ULONG  Index;

  for (Index = 0; Index < gUnicastPairCount; Index++) {
    if (gUnicastPairs[Index].AuthAlgoId == Auth) {
      return TRUE;
    }
  }
  return FALSE;
}

/**
  Cipher C appears in Pairs (with any authentication algorithm: the OS may
  set the ciphers before the algorithm).
**/
static BOOLEAN
CipherSupported (const DOT11_AUTH_CIPHER_PAIR *Pairs, ULONG Count, DOT11_CIPHER_ALGORITHM C)
{
  ULONG  Index;

  for (Index = 0; Index < Count; Index++) {
    if (Pairs[Index].CipherAlgoId == C) {
      return TRUE;
    }
  }
  return FALSE;
}

//
// ---------------------------------------------------------------------------
// Queries.
// ---------------------------------------------------------------------------
//

static NDIS_STATUS
QueryStatistics (PADAPTER A, PNDIS_OID_REQUEST Req)
{
  struct {
    DOT11_STATISTICS            S;
    DOT11_PHY_FRAME_STATISTICS  More[PHY_COUNT - 1];
  } St;
  ULONG  Phy;

  RtlZeroMemory (&St, sizeof (St));
  HDR (St.S.Header, NDIS_OBJECT_TYPE_DEFAULT, DOT11_STATISTICS_REVISION_1, sizeof (DOT11_STATISTICS));
  St.S.MacUcastCounters.ullTransmittedFrameCount        = A->TxFrames - A->TxMulticast;
  St.S.MacUcastCounters.ullReceivedFrameCount           = A->RxFrames - A->RxMulticast;
  St.S.MacUcastCounters.ullTransmittedFailureFrameCount = A->TxErrors;
  St.S.MacMcastCounters.ullTransmittedFrameCount        = A->TxMulticast;
  St.S.MacMcastCounters.ullReceivedFrameCount           = A->RxMulticast;
  for (Phy = 0; Phy < PHY_COUNT; Phy++) {
    if (Phy == A->ActivePhyId) {
      St.S.PhyCounters[Phy].ullTransmittedFrameCount = A->TxFrames;
      St.S.PhyCounters[Phy].ullMulticastTransmittedFrameCount = A->TxMulticast;
      St.S.PhyCounters[Phy].ullFailedCount           = A->TxErrors;
      St.S.PhyCounters[Phy].ullReceivedFrameCount    = A->RxFrames;
      St.S.PhyCounters[Phy].ullMulticastReceivedFrameCount = A->RxMulticast;
    }
  }
  return Answer (Req, &St, FIELD_OFFSET (DOT11_STATISTICS, PhyCounters) + PHY_COUNT * sizeof (DOT11_PHY_FRAME_STATISTICS));
}

static NDIS_STATUS
QueryAssociationInfo (PADAPTER A, PNDIS_OID_REQUEST Req)
{
  DOT11_ASSOCIATION_INFO_EX  E;
  ULONG                      Index;

  RtlZeroMemory (&E, sizeof (E));
  if (A->State == StateConnected) {
    RtlCopyMemory (E.PeerMacAddress, A->Bssid, 6);
    RtlCopyMemory (E.BSSID, A->Bssid, 6);
    E.usCapabilityInformation = DOT11_CAPABILITY_INFO_ESS;
    E.usListenInterval        = 10;
    for (Index = 0; Index < gRateCount; Index++) {
      E.ucPeerSupportedRates[Index] = gRateTable[Index].ucDataRateIndex;
    }
    E.usAssociationID       = A->Aid;
    E.dot11AssociationState = dot11_assoc_state_auth_assoc;
    E.dot11PowerMode        = dot11_power_mode_active;
    E.liAssociationUpTime.QuadPart = (LONGLONG)(NowMs () - A->ConnectedAt) * 10000;
    E.ullNumOfTxPacketSuccesses = A->TxFrames;
    E.ullNumOfTxPacketFailures  = A->TxErrors;
    E.ullNumOfRxPacketSuccesses = A->RxFrames;
  }
  return AnswerList (Req, DOT11_ASSOCIATION_INFO_LIST_REVISION_1, sizeof (DOT11_ASSOCIATION_INFO_LIST),
                     FIELD_OFFSET (DOT11_ASSOCIATION_INFO_LIST, dot11AssocInfo), &E,
                     (A->State == StateConnected) ? 1 : 0, sizeof (E));
}

static NDIS_STATUS
QueryBssList (PADAPTER A, PVOID Buffer, ULONG Length, UINT *Written, UINT *Needed)
{
  DOT11_BYTE_ARRAY  *Out = Buffer;
  ULONG             Hdr  = FIELD_OFFSET (DOT11_BYTE_ARRAY, ucBuffer);
  ULONG             Total;
  ULONG             Bytes;

  if (Length < Hdr) {
    BssEnumerate (A, NULL, 0, &Total);
    *Needed = Hdr + Total;
    return NDIS_STATUS_BUFFER_TOO_SHORT;
  }
  Bytes = BssEnumerate (A, Out->ucBuffer, Length - Hdr, &Total);
  HDR (Out->Header, NDIS_OBJECT_TYPE_DEFAULT, DOT11_BSS_ENTRY_BYTE_ARRAY_REVISION_1, sizeof (DOT11_BYTE_ARRAY));
  Out->uNumOfBytes      = Bytes;
  Out->uTotalNumOfBytes = Total;
  *Written = Hdr + Bytes;
  if (Bytes < Total) {
    *Needed = Hdr + Total;
    return NDIS_STATUS_BUFFER_OVERFLOW;
  }
  return NDIS_STATUS_SUCCESS;
}

static NDIS_STATUS
Query (PADAPTER A, PNDIS_OID_REQUEST Req)
{
  static const char  Vendor[] = "Broadcom BCM4354 Native Wi-Fi (SM-P900)";
  static const ULONG Dsss[]   = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 };
  static const ULONG Ofdm[]   = { 5180, 5200, 5220, 5240, 5260, 5280, 5300, 5320, 5500, 5520, 5540, 5560, 5580,
                                  5600, 5620, 5640, 5660, 5680, 5700, 5745, 5765, 5785, 5805, 5825 };
  NDIS_STATISTICS_INFO                 Stats;
  NDIS_INTERRUPT_MODERATION_PARAMETERS Mod;
  DOT11_OPERATION_MODE_CAPABILITY      Cap;
  DOT11_CURRENT_OPERATION_MODE         Mode;
  DOT11_EXTSTA_CAPABILITY              Ext;
  DOT11_DATA_RATE_MAPPING_TABLE        Rates;
  DOT11_RATE_SET                       RateSet;
  DOT11_SUPPORTED_POWER_LEVELS         Power;
  DOT11_REG_DOMAINS_SUPPORT_VALUE      Reg;
  struct {
    ULONG           uNumOfEntries;
    ULONG           uTotalNumOfEntries;
    DOT11_PHY_TYPE  Types[PHY_COUNT];
  } PhyTypes;
  struct {
    ULONG  uNumOfEntries;
    ULONG  uTotalNumOfEntries;
    ULONG  Values[24];
  } Chan;
  DOT11_AUTH_ALGORITHM    Auth;
  DOT11_CIPHER_ALGORITHM  Cipher;
  ULONG                   Phy;
  ULONG                   U;
  ULONG64                 U64;
  ULONG                   Index;
  NDIS_OID                Oid = Req->DATA.QUERY_INFORMATION.Oid;
  NDIS_STATUS             Status;

  Phy = (A->MibPhyId < PHY_COUNT) ? A->MibPhyId : 0;
  switch (Oid) {
    //
    // General.
    //
    case OID_GEN_SUPPORTED_LIST:        return Answer (Req, gSupportedOids, gSupportedOidsSize);
    case OID_GEN_HARDWARE_STATUS:       return AnswerUlong (Req, NdisHardwareStatusReady);
    case OID_GEN_MEDIA_SUPPORTED:
    case OID_GEN_MEDIA_IN_USE:          return AnswerUlong (Req, NdisMediumNative802_11);
    case OID_GEN_MAXIMUM_FRAME_SIZE:    return AnswerUlong (Req, 1500);
    case OID_GEN_MAXIMUM_TOTAL_SIZE:
    case OID_GEN_TRANSMIT_BLOCK_SIZE:
    case OID_GEN_RECEIVE_BLOCK_SIZE:    return AnswerUlong (Req, 1500 + 32);
    case OID_GEN_LINK_SPEED:            return AnswerUlong (Req, (A->State == StateConnected) ? (ULONG)(A->LinkSpeed / 100) : 0);
    case OID_GEN_VENDOR_ID:             return AnswerUlong (Req, 0x00904C);
    case OID_GEN_VENDOR_DESCRIPTION:    return Answer (Req, Vendor, sizeof (Vendor));
    case OID_GEN_VENDOR_DRIVER_VERSION: return AnswerUlong (Req, 0x00010000);
    case OID_GEN_DRIVER_VERSION:        U = 0x0630; return Answer (Req, &U, 2);
    case OID_GEN_CURRENT_PACKET_FILTER: return AnswerUlong (Req, A->PacketFilter);
    case OID_GEN_CURRENT_LOOKAHEAD:     return AnswerUlong (Req, A->Lookahead);
    case OID_GEN_MAXIMUM_SEND_PACKETS:  return AnswerUlong (Req, 16);
    case OID_GEN_XMIT_OK:               U64 = A->TxFrames; return Answer (Req, &U64, 8);
    case OID_GEN_RCV_OK:                U64 = A->RxFrames; return Answer (Req, &U64, 8);
    case OID_GEN_XMIT_ERROR:            U64 = A->TxErrors; return Answer (Req, &U64, 8);
    case OID_GEN_RCV_ERROR:             U64 = 0; return Answer (Req, &U64, 8);
    case OID_GEN_RCV_NO_BUFFER:         U64 = A->RxDropped; return Answer (Req, &U64, 8);
    case OID_GEN_STATISTICS:
      RtlZeroMemory (&Stats, sizeof (Stats));
      HDR (Stats.Header, NDIS_OBJECT_TYPE_DEFAULT, NDIS_STATISTICS_INFO_REVISION_1, NDIS_SIZEOF_STATISTICS_INFO_REVISION_1);
      Stats.SupportedStatistics = NDIS_STATISTICS_XMIT_OK_SUPPORTED | NDIS_STATISTICS_RCV_OK_SUPPORTED |
                                  NDIS_STATISTICS_XMIT_ERROR_SUPPORTED | NDIS_STATISTICS_RCV_DISCARDS_SUPPORTED;
      Stats.ifHCInUcastPkts  = A->RxFrames - A->RxMulticast;
      Stats.ifHCInMulticastPkts = A->RxMulticast;
      Stats.ifHCOutUcastPkts = A->TxFrames - A->TxMulticast;
      Stats.ifHCOutMulticastPkts = A->TxMulticast;
      Stats.ifInDiscards     = A->RxDropped;
      Stats.ifOutErrors      = A->TxErrors;
      return Answer (Req, &Stats, sizeof (Stats));
    case OID_GEN_INTERRUPT_MODERATION:
      RtlZeroMemory (&Mod, sizeof (Mod));
      HDR (Mod.Header, NDIS_OBJECT_TYPE_DEFAULT, NDIS_INTERRUPT_MODERATION_PARAMETERS_REVISION_1,
           NDIS_SIZEOF_INTERRUPT_MODERATION_PARAMETERS_REVISION_1);
      Mod.InterruptModeration = NdisInterruptModerationNotSupported;
      return Answer (Req, &Mod, sizeof (Mod));

    //
    // 802.11 MIB.
    //
    case OID_DOT11_OPERATION_MODE_CAPABILITY:
      RtlZeroMemory (&Cap, sizeof (Cap));
      Cap.uMajorVersion     = 2;
      Cap.uMinorVersion     = 0;
      Cap.uNumOfTXBuffers   = TX_QUEUE_MAX;
      Cap.uNumOfRXBuffers   = 1;
      Cap.uOpModeCapability = DOT11_OPERATION_MODE_EXTENSIBLE_STATION;
      return Answer (Req, &Cap, sizeof (Cap));
    case OID_DOT11_CURRENT_OPERATION_MODE:
      RtlZeroMemory (&Mode, sizeof (Mode));
      Mode.uCurrentOpMode = A->OperationMode;
      return Answer (Req, &Mode, sizeof (Mode));
    case OID_DOT11_ATIM_WINDOW:           return AnswerUlong (Req, 0);
    case OID_DOT11_NIC_POWER_STATE:       return AnswerBool (Req, A->RadioOn);
    case OID_DOT11_HARDWARE_PHY_STATE:    return AnswerBool (Req, TRUE);
    case OID_DOT11_MPDU_MAX_LENGTH:       return AnswerUlong (Req, DOT11_MAX_PDU_SIZE);
    case OID_DOT11_STATION_ID:
    case OID_DOT11_MAC_ADDRESS:
    case OID_DOT11_CURRENT_ADDRESS:
    case OID_DOT11_PERMANENT_ADDRESS:     return Answer (Req, A->Mac, 6);
    case OID_DOT11_MULTICAST_LIST:        Req->DATA.QUERY_INFORMATION.BytesWritten = 0; return NDIS_STATUS_SUCCESS;
    case OID_DOT11_MAXIMUM_LIST_SIZE:     return AnswerUlong (Req, 32);
    case OID_DOT11_BEACON_PERIOD:         return AnswerUlong (Req, 100);
    case OID_DOT11_DTIM_PERIOD:           return AnswerUlong (Req, 1);
    case OID_DOT11_RTS_THRESHOLD:         return AnswerUlong (Req, A->RtsThreshold);
    case OID_DOT11_FRAGMENTATION_THRESHOLD: return AnswerUlong (Req, A->FragThreshold);
    case OID_DOT11_SHORT_RETRY_LIMIT:     return AnswerUlong (Req, 7);
    case OID_DOT11_LONG_RETRY_LIMIT:      return AnswerUlong (Req, 4);
    case OID_DOT11_MAX_TRANSMIT_MSDU_LIFETIME:
    case OID_DOT11_MAX_RECEIVE_LIFETIME:  return AnswerUlong (Req, 512);
    case OID_DOT11_OPERATIONAL_RATE_SET:
      RtlZeroMemory (&RateSet, sizeof (RateSet));
      for (Index = 0; (Index < MAX_NUM_SUPPORTED_RATES_V2) &&
                      (gPhyAttributes[Phy].SupportedDataRatesValue.ucSupportedTxDataRatesValue[Index] != 0); Index++) {
        RateSet.ucRateSet[Index] = gPhyAttributes[Phy].SupportedDataRatesValue.ucSupportedTxDataRatesValue[Index];
      }
      RateSet.uRateSetLength = Index;
      return Answer (Req, &RateSet, sizeof (RateSet));
    case OID_DOT11_SUPPORTED_PHY_TYPES:
      PhyTypes.uNumOfEntries      = PHY_COUNT;
      PhyTypes.uTotalNumOfEntries = PHY_COUNT;
      for (Index = 0; Index < PHY_COUNT; Index++) {
        PhyTypes.Types[Index] = gPhyAttributes[Index].PhyType;
      }
      return Answer (Req, &PhyTypes, sizeof (PhyTypes));
    case OID_DOT11_CURRENT_REG_DOMAIN:    return AnswerUlong (Req, DOT11_REG_DOMAIN_ETSI);
    case OID_DOT11_REG_DOMAINS_SUPPORT_VALUE:
      RtlZeroMemory (&Reg, sizeof (Reg));
      Reg.uNumOfEntries      = 1;
      Reg.uTotalNumOfEntries = 1;
      Reg.dot11RegDomainValue[0].uRegDomainsSupportIndex = 0;
      Reg.dot11RegDomainValue[0].uRegDomainsSupportValue = DOT11_REG_DOMAIN_ETSI;
      return Answer (Req, &Reg, sizeof (Reg));
    case OID_DOT11_TEMP_TYPE:             return AnswerUlong (Req, dot11_temp_type_1);
    case OID_DOT11_DIVERSITY_SUPPORT:     return AnswerUlong (Req, dot11_diversity_support_dynamic);
    case OID_DOT11_SUPPORTED_POWER_LEVELS:
      RtlZeroMemory (&Power, sizeof (Power));
      Power.uNumOfSupportedPowerLevels = 1;
      Power.uTxPowerLevelValues[0]     = 100;
      return Answer (Req, &Power, sizeof (Power));
    case OID_DOT11_CURRENT_TX_POWER_LEVEL: return AnswerUlong (Req, 1);
    case OID_DOT11_CURRENT_CHANNEL:
      return AnswerUlong (Req, ((A->State == StateConnected) && (A->ActivePhyId == PHY_ID_ERP))
                               ? ((A->ActiveFrequency == 2484) ? 14 : (A->ActiveFrequency - 2407) / 5) : 1);
    case OID_DOT11_CURRENT_FREQUENCY:
      return AnswerUlong (Req, ((A->State == StateConnected) && (A->ActivePhyId == PHY_ID_OFDM))
                               ? (A->ActiveFrequency - 5000) / 5 : 36);
    case OID_DOT11_FREQUENCY_BANDS_SUPPORTED:
      return AnswerUlong (Req, DOT11_FREQUENCY_BANDS_LOWER | DOT11_FREQUENCY_BANDS_MIDDLE | DOT11_FREQUENCY_BANDS_UPPER);
    case OID_DOT11_SUPPORTED_DATA_RATES_VALUE:
      return Answer (Req, &gPhyAttributes[Phy].SupportedDataRatesValue, sizeof (DOT11_SUPPORTED_DATA_RATES_VALUE_V2));
    case OID_DOT11_MULTI_DOMAIN_CAPABILITY_IMPLEMENTED:
    case OID_DOT11_PBCC_OPTION_IMPLEMENTED:
    case OID_DOT11_CHANNEL_AGILITY_PRESENT:
    case OID_DOT11_CHANNEL_AGILITY_ENABLED:
    case OID_DOT11_ERP_PBCC_OPTION_IMPLEMENTED:
    case OID_DOT11_ERP_PBCC_OPTION_ENABLED:
    case OID_DOT11_DSSS_OFDM_OPTION_IMPLEMENTED:
    case OID_DOT11_DSSS_OFDM_OPTION_ENABLED:   return AnswerBool (Req, FALSE);
    case OID_DOT11_SHORT_PREAMBLE_OPTION_IMPLEMENTED:
    case OID_DOT11_SHORT_SLOT_TIME_OPTION_IMPLEMENTED:
    case OID_DOT11_SHORT_SLOT_TIME_OPTION_ENABLED: return AnswerBool (Req, TRUE);
    case OID_DOT11_SUPPORTED_DSSS_CHANNEL_LIST:
      Chan.uNumOfEntries = Chan.uTotalNumOfEntries = ARRAY_SIZE (Dsss);
      RtlCopyMemory (Chan.Values, Dsss, sizeof (Dsss));
      return Answer (Req, &Chan, 8 + sizeof (Dsss));
    case OID_DOT11_SUPPORTED_OFDM_FREQUENCY_LIST:
      Chan.uNumOfEntries = Chan.uTotalNumOfEntries = ARRAY_SIZE (Ofdm);
      RtlCopyMemory (Chan.Values, Ofdm, sizeof (Ofdm));
      return Answer (Req, &Chan, 8 + sizeof (Ofdm));

    //
    // ExtSTA.
    //
    case OID_DOT11_EXTSTA_CAPABILITY:
      RtlZeroMemory (&Ext, sizeof (Ext));
      HDR (Ext.Header, NDIS_OBJECT_TYPE_DEFAULT, DOT11_EXTSTA_CAPABILITY_REVISION_1, sizeof (Ext));
      Ext.uScanSSIDListSize           = MAX_SCAN_SSIDS;
      Ext.uDesiredBSSIDListSize       = MAX_DESIRED_BSSIDS;
      Ext.uDesiredSSIDListSize        = MAX_DESIRED_SSIDS;
      Ext.uExcludedMacAddressListSize = MAX_EXCLUDED_MACS;
      Ext.uPrivacyExemptionListSize   = MAX_PRIVACY_EXEMPT;
      Ext.uKeyMappingTableSize        = MAX_KEY_MAPPING;
      Ext.uDefaultKeyTableSize        = MAX_DEFAULT_KEYS;
      Ext.uWEPKeyValueMaxLength       = 13;
      return Answer (Req, &Ext, sizeof (Ext));
    case OID_DOT11_DATA_RATE_MAPPING_TABLE:
      RtlZeroMemory (&Rates, sizeof (Rates));
      HDR (Rates.Header, NDIS_OBJECT_TYPE_DEFAULT, DOT11_DATA_RATE_MAPPING_TABLE_REVISION_1, sizeof (Rates));
      Rates.uDataRateMappingLength = gRateCount;
      RtlCopyMemory (Rates.DataRateMappingEntries, gRateTable, gRateCount * sizeof (gRateTable[0]));
      return Answer (Req, &Rates, sizeof (Rates));
    case OID_DOT11_AUTO_CONFIG_ENABLED:   return AnswerUlong (Req, A->AutoConfig);
    case OID_DOT11_POWER_MGMT_REQUEST:    return AnswerUlong (Req, A->PowerSaving);
    case OID_DOT11_DESIRED_BSS_TYPE:      return AnswerUlong (Req, A->DesiredBssType);
    case OID_DOT11_EXCLUDE_UNENCRYPTED:   return AnswerBool (Req, A->ExcludeUnencrypted);
    case OID_DOT11_CIPHER_DEFAULT_KEY_ID: return AnswerUlong (Req, A->DefaultKeyId);
    case OID_DOT11_UNICAST_USE_GROUP_ENABLED: return AnswerBool (Req, A->UnicastUseGroup);
    case OID_DOT11_CURRENT_PHY_ID:        return AnswerUlong (Req, A->MibPhyId);
    case OID_DOT11_MEDIA_STREAMING_ENABLED: return AnswerBool (Req, A->MediaStreaming);
    case OID_DOT11_UNREACHABLE_DETECTION_THRESHOLD: return AnswerUlong (Req, A->UnreachableThreshold);
    case OID_DOT11_SAFE_MODE_ENABLED:     return AnswerBool (Req, A->SafeMode);
    case OID_DOT11_HIDDEN_NETWORK_ENABLED: return AnswerBool (Req, A->HiddenNetwork);
    case OID_DOT11_DESIRED_COUNTRY_OR_REGION_STRING: return Answer (Req, A->Country, 3);
    case OID_DOT11_STATISTICS:            return QueryStatistics (A, Req);
    case OID_DOT11_ENUM_ASSOCIATION_INFO: return QueryAssociationInfo (A, Req);
    case OID_DOT11_DESIRED_SSID_LIST:
      return AnswerList (Req, DOT11_SSID_LIST_REVISION_1, sizeof (DOT11_SSID_LIST), FIELD_OFFSET (DOT11_SSID_LIST, SSIDs),
                         A->DesiredSsid, A->DesiredSsidCount, sizeof (DOT11_SSID));
    case OID_DOT11_DESIRED_BSSID_LIST:
      return AnswerList (Req, DOT11_BSSID_LIST_REVISION_1, sizeof (DOT11_BSSID_LIST), FIELD_OFFSET (DOT11_BSSID_LIST, BSSIDs),
                         A->DesiredBssid, A->DesiredBssidCount, 6);
    case OID_DOT11_EXCLUDED_MAC_ADDRESS_LIST:
      return AnswerList (Req, DOT11_MAC_ADDRESS_LIST_REVISION_1, sizeof (DOT11_MAC_ADDRESS_LIST),
                         FIELD_OFFSET (DOT11_MAC_ADDRESS_LIST, MacAddrs), A->Excluded, A->ExcludedCount, 6);
    case OID_DOT11_PRIVACY_EXEMPTION_LIST:
      return AnswerList (Req, DOT11_PRIVACY_EXEMPTION_LIST_REVISION_1, sizeof (DOT11_PRIVACY_EXEMPTION_LIST),
                         FIELD_OFFSET (DOT11_PRIVACY_EXEMPTION_LIST, PrivacyExemptionEntries), A->PrivacyExempt,
                         A->PrivacyExemptCount, sizeof (DOT11_PRIVACY_EXEMPTION));
    case OID_DOT11_ENABLED_AUTHENTICATION_ALGORITHM:
      Auth = A->AuthAlgorithm;
      return AnswerList (Req, DOT11_AUTH_ALGORITHM_LIST_REVISION_1, sizeof (DOT11_AUTH_ALGORITHM_LIST),
                         FIELD_OFFSET (DOT11_AUTH_ALGORITHM_LIST, AlgorithmIds), &Auth, 1, sizeof (Auth));
    case OID_DOT11_ENABLED_UNICAST_CIPHER_ALGORITHM:
    case OID_DOT11_ENABLED_MULTICAST_CIPHER_ALGORITHM:
      Cipher = (Oid == OID_DOT11_ENABLED_UNICAST_CIPHER_ALGORITHM) ? A->UnicastCipher : A->MulticastCipher;
      return AnswerList (Req, DOT11_CIPHER_ALGORITHM_LIST_REVISION_1, sizeof (DOT11_CIPHER_ALGORITHM_LIST),
                         FIELD_OFFSET (DOT11_CIPHER_ALGORITHM_LIST, AlgorithmIds), &Cipher, 1, sizeof (Cipher));
    case OID_DOT11_SUPPORTED_UNICAST_ALGORITHM_PAIR:
      return AnswerList (Req, DOT11_AUTH_CIPHER_PAIR_LIST_REVISION_1, sizeof (DOT11_AUTH_CIPHER_PAIR_LIST),
                         FIELD_OFFSET (DOT11_AUTH_CIPHER_PAIR_LIST, AuthCipherPairs), gUnicastPairs, gUnicastPairCount,
                         sizeof (DOT11_AUTH_CIPHER_PAIR));
    case OID_DOT11_SUPPORTED_MULTICAST_ALGORITHM_PAIR:
      return AnswerList (Req, DOT11_AUTH_CIPHER_PAIR_LIST_REVISION_1, sizeof (DOT11_AUTH_CIPHER_PAIR_LIST),
                         FIELD_OFFSET (DOT11_AUTH_CIPHER_PAIR_LIST, AuthCipherPairs), gMulticastPairs, gMulticastPairCount,
                         sizeof (DOT11_AUTH_CIPHER_PAIR));
    case OID_DOT11_DESIRED_PHY_LIST:
      return AnswerList (Req, DOT11_PHY_ID_LIST_REVISION_1, sizeof (DOT11_PHY_ID_LIST),
                         FIELD_OFFSET (DOT11_PHY_ID_LIST, dot11PhyId), A->DesiredPhy, A->DesiredPhyCount, sizeof (ULONG));
    case OID_DOT11_ACTIVE_PHY_LIST:
      U = A->ActivePhyId;
      return AnswerList (Req, DOT11_PHY_ID_LIST_REVISION_1, sizeof (DOT11_PHY_ID_LIST),
                         FIELD_OFFSET (DOT11_PHY_ID_LIST, dot11PhyId), &U, (A->State == StateConnected) ? 1 : 0,
                         sizeof (ULONG));
    case OID_DOT11_SUPPORTED_COUNTRY_OR_REGION_STRING:
      return AnswerList (Req, DOT11_COUNTRY_OR_REGION_STRING_LIST_REVISION_1, sizeof (DOT11_COUNTRY_OR_REGION_STRING_LIST),
                         FIELD_OFFSET (DOT11_COUNTRY_OR_REGION_STRING_LIST, CountryOrRegionStrings), NULL, 0,
                         sizeof (DOT11_COUNTRY_OR_REGION_STRING));
    case OID_DOT11_ENUM_BSS_LIST:
      Status = QueryBssList (A, Req->DATA.QUERY_INFORMATION.InformationBuffer,
                             Req->DATA.QUERY_INFORMATION.InformationBufferLength,
                             &Req->DATA.QUERY_INFORMATION.BytesWritten, &Req->DATA.QUERY_INFORMATION.BytesNeeded);
      return Status;
    default:
      OsLog ("exynoswifi: query OID 0x%08lx not supported", Oid);
      return NDIS_STATUS_NOT_SUPPORTED;
  }
}

//
// ---------------------------------------------------------------------------
// Sets (MIB only; chip work is in OidOnBus).
// ---------------------------------------------------------------------------
//

static NDIS_STATUS
Set (PADAPTER A, PNDIS_OID_REQUEST Req)
{
  NDIS_OID                Oid = Req->DATA.SET_INFORMATION.Oid;
  ULONG                   Length = Req->DATA.SET_INFORMATION.InformationBufferLength;
  const UCHAR             *Elems;
  ULONG                   Count;
  ULONG                   U;
  ULONG                   Index;
  BOOLEAN                 B;
  DOT11_AUTH_ALGORITHM    Auth;
  DOT11_CIPHER_ALGORITHM  Cipher;
  DOT11_CURRENT_OPERATION_MODE  Mode;
  DOT11_ASSOCIATION_PARAMS      Ap;
  NDIS_STATUS             Status;

  switch (Oid) {
    case OID_802_3_MULTICAST_LIST:
      Req->DATA.SET_INFORMATION.BytesRead = Length;
      return NDIS_STATUS_SUCCESS;
    case OID_GEN_CURRENT_PACKET_FILTER:   return TakeValue (Req, &A->PacketFilter, 4);
    case OID_GEN_CURRENT_LOOKAHEAD:       return TakeValue (Req, &A->Lookahead, 4);
    case OID_GEN_LINK_PARAMETERS:
    case OID_DOT11_MULTICAST_LIST:
      //
      // The firmware passes multicast as configured; the filter above us
      // does the rest.
      //
      Req->DATA.SET_INFORMATION.BytesRead = Length;
      return NDIS_STATUS_SUCCESS;
    case OID_GEN_INTERRUPT_MODERATION:    return NDIS_STATUS_INVALID_DATA;

    case OID_DOT11_CURRENT_OPERATION_MODE:
      Status = TakeValue (Req, &Mode, sizeof (Mode));
      if (Status == NDIS_STATUS_SUCCESS) {
        if (Mode.uCurrentOpMode != DOT11_OPERATION_MODE_EXTENSIBLE_STATION) {
          return NDIS_STATUS_INVALID_DATA;
        }
        A->OperationMode = Mode.uCurrentOpMode;
      }
      return Status;
    case OID_DOT11_ATIM_WINDOW:
    case OID_DOT11_CURRENT_CHANNEL:
    case OID_DOT11_CURRENT_FREQUENCY:
    case OID_DOT11_CURRENT_REG_DOMAIN:
    case OID_DOT11_CURRENT_TX_POWER_LEVEL:
    case OID_DOT11_SHORT_RETRY_LIMIT:
    case OID_DOT11_LONG_RETRY_LIMIT:
    case OID_DOT11_BEACON_PERIOD:
      return TakeValue (Req, &U, 4);    // accepted, the firmware decides
    case OID_DOT11_RTS_THRESHOLD:         return TakeValue (Req, &A->RtsThreshold, 4);
    case OID_DOT11_FRAGMENTATION_THRESHOLD: return TakeValue (Req, &A->FragThreshold, 4);
    case OID_DOT11_AUTO_CONFIG_ENABLED:   return TakeValue (Req, &A->AutoConfig, 4);
    case OID_DOT11_POWER_MGMT_REQUEST:    return TakeValue (Req, &A->PowerSaving, 4);   // PM stays off
    case OID_DOT11_UNREACHABLE_DETECTION_THRESHOLD: return TakeValue (Req, &A->UnreachableThreshold, 4);
    case OID_DOT11_EXCLUDE_UNENCRYPTED:   return TakeValue (Req, &A->ExcludeUnencrypted, 1);
    case OID_DOT11_UNICAST_USE_GROUP_ENABLED: return TakeValue (Req, &A->UnicastUseGroup, 1);
    case OID_DOT11_MEDIA_STREAMING_ENABLED: return TakeValue (Req, &A->MediaStreaming, 1);
    case OID_DOT11_SAFE_MODE_ENABLED:
      Status = TakeValue (Req, &B, 1);
      if ((Status == NDIS_STATUS_SUCCESS) && B) {
        return NDIS_STATUS_INVALID_DATA;                // not implemented
      }
      return Status;
    case OID_DOT11_HIDDEN_NETWORK_ENABLED: return TakeValue (Req, &A->HiddenNetwork, 1);
    case OID_DOT11_DESIRED_COUNTRY_OR_REGION_STRING: return TakeValue (Req, A->Country, 3);
    case OID_DOT11_CIPHER_DEFAULT_KEY_ID:
      Status = TakeValue (Req, &U, 4);
      if ((Status == NDIS_STATUS_SUCCESS) && (U >= MAX_DEFAULT_KEYS)) {
        return NDIS_STATUS_INVALID_DATA;
      }
      A->DefaultKeyId = U;
      return Status;
    case OID_DOT11_CURRENT_PHY_ID:
      Status = TakeValue (Req, &U, 4);
      if ((Status == NDIS_STATUS_SUCCESS) && (U >= PHY_COUNT)) {
        return NDIS_STATUS_INVALID_DATA;
      }
      A->MibPhyId = U;
      return Status;
    case OID_DOT11_DESIRED_BSS_TYPE:
      Status = TakeValue (Req, &U, 4);
      if (Status == NDIS_STATUS_SUCCESS) {
        if ((U != dot11_BSS_type_infrastructure) && (U != dot11_BSS_type_any) && (U != dot11_BSS_type_independent)) {
          return NDIS_STATUS_INVALID_DATA;
        }
        A->DesiredBssType = (DOT11_BSS_TYPE)U;
      }
      return Status;
    case OID_DOT11_FLUSH_BSS_LIST:
      BssFlush (A);
      return NDIS_STATUS_SUCCESS;
    case OID_DOT11_IBSS_PARAMS:
      Req->DATA.SET_INFORMATION.BytesRead = Length;
      return NDIS_STATUS_SUCCESS;
    case OID_DOT11_ASSOCIATION_PARAMS:
      Status = TakeValue (Req, &Ap, sizeof (Ap));
      if (Status == NDIS_STATUS_SUCCESS) {
        A->AssocIesLength = 0;
        if ((Ap.uAssocRequestIEsLength > 0) && (Ap.uAssocRequestIEsLength <= ASSOC_IES_MAX) &&
            (Ap.uAssocRequestIEsOffset >= sizeof (Ap)) &&
            (Ap.uAssocRequestIEsOffset + Ap.uAssocRequestIEsLength <= Length)) {
          RtlCopyMemory (A->AssocIes, (PUCHAR)Req->DATA.SET_INFORMATION.InformationBuffer + Ap.uAssocRequestIEsOffset,
                         Ap.uAssocRequestIEsLength);
          A->AssocIesLength = Ap.uAssocRequestIEsLength;
        }
        Req->DATA.SET_INFORMATION.BytesRead = Length;
      }
      return Status;

    case OID_DOT11_DESIRED_SSID_LIST:
      Status = TakeList (Req, FIELD_OFFSET (DOT11_SSID_LIST, SSIDs), sizeof (DOT11_SSID), MAX_DESIRED_SSIDS, &Elems, &Count);
      if (Status == NDIS_STATUS_SUCCESS) {
        for (Index = 0; Index < Count; Index++) {
          RtlCopyMemory (&A->DesiredSsid[Index], Elems + Index * sizeof (DOT11_SSID), sizeof (DOT11_SSID));
          if (A->DesiredSsid[Index].uSSIDLength > DOT11_SSID_MAX_LENGTH) {
            A->DesiredSsid[Index].uSSIDLength = DOT11_SSID_MAX_LENGTH;
          }
        }
        A->DesiredSsidCount = Count;
      }
      return Status;
    case OID_DOT11_DESIRED_BSSID_LIST:
      Status = TakeList (Req, FIELD_OFFSET (DOT11_BSSID_LIST, BSSIDs), 6, MAX_DESIRED_BSSIDS, &Elems, &Count);
      if (Status == NDIS_STATUS_SUCCESS) {
        RtlCopyMemory (A->DesiredBssid, Elems, Count * 6);
        A->DesiredBssidCount = Count;
      }
      return Status;
    case OID_DOT11_EXCLUDED_MAC_ADDRESS_LIST:
      Status = TakeList (Req, FIELD_OFFSET (DOT11_MAC_ADDRESS_LIST, MacAddrs), 6, MAX_EXCLUDED_MACS, &Elems, &Count);
      if (Status == NDIS_STATUS_SUCCESS) {
        RtlCopyMemory (A->Excluded, Elems, Count * 6);
        A->ExcludedCount = Count;
      }
      return Status;
    case OID_DOT11_PRIVACY_EXEMPTION_LIST:
      Status = TakeList (Req, FIELD_OFFSET (DOT11_PRIVACY_EXEMPTION_LIST, PrivacyExemptionEntries),
                         sizeof (DOT11_PRIVACY_EXEMPTION), MAX_PRIVACY_EXEMPT, &Elems, &Count);
      if (Status == NDIS_STATUS_SUCCESS) {
        RtlCopyMemory (A->PrivacyExempt, Elems, Count * sizeof (DOT11_PRIVACY_EXEMPTION));
        A->PrivacyExemptCount = Count;
      }
      return Status;
    case OID_DOT11_DESIRED_PHY_LIST:
      Status = TakeList (Req, FIELD_OFFSET (DOT11_PHY_ID_LIST, dot11PhyId), sizeof (ULONG), PHY_COUNT + 1, &Elems, &Count);
      if (Status == NDIS_STATUS_SUCCESS) {
        if (Count == 0) {
          return NDIS_STATUS_INVALID_DATA;
        }
        for (Index = 0; Index < Count; Index++) {
          U = Rd32 (Elems + Index * 4);
          if ((U != DOT11_PHY_ID_ANY) && (U >= PHY_COUNT)) {
            return NDIS_STATUS_INVALID_DATA;
          }
          A->DesiredPhy[Index] = U;
          if (U == DOT11_PHY_ID_ANY) {
            A->DesiredPhy[0] = DOT11_PHY_ID_ANY;
            Count = 1;
            break;
          }
        }
        A->DesiredPhyCount = Count;
      }
      return Status;
    case OID_DOT11_ENABLED_AUTHENTICATION_ALGORITHM:
      Status = TakeList (Req, FIELD_OFFSET (DOT11_AUTH_ALGORITHM_LIST, AlgorithmIds), sizeof (DOT11_AUTH_ALGORITHM), 1,
                         &Elems, &Count);
      if (Status == NDIS_STATUS_SUCCESS) {
        Auth = (DOT11_AUTH_ALGORITHM)Rd32 (Elems);
        if ((Count != 1) || !AuthSupported (Auth)) {
          return NDIS_STATUS_INVALID_DATA;
        }
        A->AuthAlgorithm = Auth;
      }
      return Status;
    case OID_DOT11_ENABLED_UNICAST_CIPHER_ALGORITHM:
    case OID_DOT11_ENABLED_MULTICAST_CIPHER_ALGORITHM:
      Status = TakeList (Req, FIELD_OFFSET (DOT11_CIPHER_ALGORITHM_LIST, AlgorithmIds), sizeof (DOT11_CIPHER_ALGORITHM),
                         4, &Elems, &Count);
      if (Status == NDIS_STATUS_SUCCESS) {
        for (Index = 0; Index < Count; Index++) {
          Cipher = (DOT11_CIPHER_ALGORITHM)Rd32 (Elems + Index * 4);
          if ((Oid == OID_DOT11_ENABLED_UNICAST_CIPHER_ALGORITHM)
              ? CipherSupported (gUnicastPairs, gUnicastPairCount, Cipher)
              : CipherSupported (gMulticastPairs, gMulticastPairCount, Cipher)) {
            break;
          }
        }
        if (Index == Count) {
          return NDIS_STATUS_INVALID_DATA;
        }
        if (Oid == OID_DOT11_ENABLED_UNICAST_CIPHER_ALGORITHM) {
          A->UnicastCipher = Cipher;
        } else {
          A->MulticastCipher = Cipher;
        }
      }
      return Status;

    default:
      OsLog ("exynoswifi: set OID 0x%08lx not supported", Oid);
      return NDIS_STATUS_NOT_SUPPORTED;
  }
}

//
// ---------------------------------------------------------------------------
// Bus OIDs.
// ---------------------------------------------------------------------------
//

BOOLEAN
OidNeedsBus (PNDIS_OID_REQUEST Req)
{
  if (Req->RequestType == NdisRequestSetInformation) {
    switch (Req->DATA.SET_INFORMATION.Oid) {
      case OID_DOT11_SCAN_REQUEST:
      case OID_DOT11_CONNECT_REQUEST:
      case OID_DOT11_DISCONNECT_REQUEST:
      case OID_DOT11_CIPHER_DEFAULT_KEY:
      case OID_DOT11_CIPHER_KEY_MAPPING_KEY:
      case OID_DOT11_NIC_POWER_STATE:
        return TRUE;
      default:
        return FALSE;
    }
  }
  return (Req->RequestType == NdisRequestMethod) && (Req->DATA.METHOD_INFORMATION.Oid == OID_DOT11_RESET_REQUEST);
}

/**
  Key material of a TKIP / CCMP / WEP key value (ucKey, Length bytes) into K.
**/
static NDIS_STATUS
ParseKey (DOT11_CIPHER_ALGORITHM Algo, const UCHAR *Key, ULONG Length, KEY_WORK *K)
{
  ULONG  Tk;
  ULONG  Mic;

  K->Algorithm = Algo;
  switch (Algo) {
    case DOT11_CIPHER_ALGO_CCMP:
      if (Length < FIELD_OFFSET (DOT11_KEY_ALGO_CCMP, ucCCMPKey)) {
        return NDIS_STATUS_INVALID_DATA;
      }
      Tk = Rd32 (Key + FIELD_OFFSET (DOT11_KEY_ALGO_CCMP, ulCCMPKeyLength));
      if ((Tk != 16) || (FIELD_OFFSET (DOT11_KEY_ALGO_CCMP, ucCCMPKey) + Tk > Length)) {
        return NDIS_STATUS_INVALID_DATA;
      }
      RtlCopyMemory (K->Rsc, Key, 6);
      RtlCopyMemory (K->Key, Key + FIELD_OFFSET (DOT11_KEY_ALGO_CCMP, ucCCMPKey), Tk);
      K->Length  = Tk;
      K->HaveRsc = TRUE;
      return NDIS_STATUS_SUCCESS;
    case DOT11_CIPHER_ALGO_TKIP:
      //
      // TK then the MIC keys in PTK / GTK order (authenticator Tx, Rx);
      // BcmSetKey() swaps them for the station side.  NOT VERIFIED.
      //
      if (Length < FIELD_OFFSET (DOT11_KEY_ALGO_TKIP_MIC, ucTKIPMICKeys)) {
        return NDIS_STATUS_INVALID_DATA;
      }
      Tk  = Rd32 (Key + FIELD_OFFSET (DOT11_KEY_ALGO_TKIP_MIC, ulTKIPKeyLength));
      Mic = Rd32 (Key + FIELD_OFFSET (DOT11_KEY_ALGO_TKIP_MIC, ulMICKeyLength));
      if ((Tk != 16) || (Mic != 16) || (FIELD_OFFSET (DOT11_KEY_ALGO_TKIP_MIC, ucTKIPMICKeys) + Tk + Mic > Length)) {
        return NDIS_STATUS_INVALID_DATA;
      }
      RtlCopyMemory (K->Rsc, Key, 6);
      RtlCopyMemory (K->Key, Key + FIELD_OFFSET (DOT11_KEY_ALGO_TKIP_MIC, ucTKIPMICKeys), 32);
      K->Length  = 32;
      K->HaveRsc = TRUE;
      return NDIS_STATUS_SUCCESS;
    case DOT11_CIPHER_ALGO_WEP:
    case DOT11_CIPHER_ALGO_WEP40:
    case DOT11_CIPHER_ALGO_WEP104:
      if ((Length != 5) && (Length != 13)) {
        return NDIS_STATUS_INVALID_DATA;
      }
      RtlCopyMemory (K->Key, Key, Length);
      K->Length = Length;
      return NDIS_STATUS_SUCCESS;
    default:
      return NDIS_STATUS_INVALID_DATA;
  }
}

static NDIS_STATUS
SetDefaultKey (PADAPTER A, PNDIS_OID_REQUEST Req)
{
  const UCHAR  *In    = Req->DATA.SET_INFORMATION.InformationBuffer;
  ULONG        Length = Req->DATA.SET_INFORMATION.InformationBufferLength;
  ULONG        Fixed  = FIELD_OFFSET (DOT11_CIPHER_DEFAULT_KEY_VALUE, ucKey);
  ULONG        KeyLen;
  KEY_WORK     K;
  NDIS_STATUS  Status;

  if (Length < Fixed) {
    Req->DATA.SET_INFORMATION.BytesNeeded = Fixed;
    return NDIS_STATUS_INVALID_LENGTH;
  }
  KeyLen = Rd16 (In + FIELD_OFFSET (DOT11_CIPHER_DEFAULT_KEY_VALUE, usKeyLength));
  if (Fixed + KeyLen > Length) {
    Req->DATA.SET_INFORMATION.BytesNeeded = Fixed + KeyLen;
    return NDIS_STATUS_INVALID_LENGTH;
  }
  RtlZeroMemory (&K, sizeof (K));
  K.Index  = Rd32 (In + FIELD_OFFSET (DOT11_CIPHER_DEFAULT_KEY_VALUE, uKeyIndex));
  K.Delete = In[FIELD_OFFSET (DOT11_CIPHER_DEFAULT_KEY_VALUE, bDelete)];
  K.Algorithm = (DOT11_CIPHER_ALGORITHM)Rd32 (In + FIELD_OFFSET (DOT11_CIPHER_DEFAULT_KEY_VALUE, AlgorithmId));
  if (K.Index >= MAX_DEFAULT_KEYS) {
    return NDIS_STATUS_INVALID_DATA;
  }
  if (!K.Delete) {
    Status = ParseKey (K.Algorithm, In + Fixed, KeyLen, &K);
    if (Status != NDIS_STATUS_SUCCESS) {
      OsLog ("exynoswifi: default key %lu: algo %lu, %lu bytes not usable", K.Index, (ULONG)K.Algorithm, KeyLen);
      return Status;
    }
  }
  Req->DATA.SET_INFORMATION.BytesRead = Fixed + KeyLen;
  return BusSetKey (A, &K);
}

static NDIS_STATUS
SetKeyMappingKeys (PADAPTER A, PNDIS_OID_REQUEST Req)
{
  const UCHAR  *In    = Req->DATA.SET_INFORMATION.InformationBuffer;
  ULONG        Length = Req->DATA.SET_INFORMATION.InformationBufferLength;
  ULONG        Hdr    = FIELD_OFFSET (DOT11_BYTE_ARRAY, ucBuffer);
  ULONG        Fixed  = FIELD_OFFSET (DOT11_CIPHER_KEY_MAPPING_KEY_VALUE, ucKey);
  ULONG        Bytes;
  ULONG        Pos;
  ULONG        KeyLen;
  const UCHAR  *V;
  KEY_WORK     K;
  NDIS_STATUS  Status = NDIS_STATUS_SUCCESS;

  if (Length < Hdr) {
    Req->DATA.SET_INFORMATION.BytesNeeded = Hdr;
    return NDIS_STATUS_INVALID_LENGTH;
  }
  Bytes = Rd32 (In + FIELD_OFFSET (DOT11_BYTE_ARRAY, uNumOfBytes));
  if (Hdr + Bytes > Length) {
    Req->DATA.SET_INFORMATION.BytesNeeded = Hdr + Bytes;
    return NDIS_STATUS_INVALID_LENGTH;
  }
  for (Pos = 0; (Pos + Fixed <= Bytes) && (Status == NDIS_STATUS_SUCCESS); Pos += Fixed + KeyLen) {
    V      = In + Hdr + Pos;
    KeyLen = Rd16 (V + FIELD_OFFSET (DOT11_CIPHER_KEY_MAPPING_KEY_VALUE, usKeyLength));
    if (Pos + Fixed + KeyLen > Bytes) {
      return NDIS_STATUS_INVALID_DATA;
    }
    RtlZeroMemory (&K, sizeof (K));
    K.Pairwise  = TRUE;
    K.Delete    = V[FIELD_OFFSET (DOT11_CIPHER_KEY_MAPPING_KEY_VALUE, bDelete)];
    K.Algorithm = (DOT11_CIPHER_ALGORITHM)Rd32 (V + FIELD_OFFSET (DOT11_CIPHER_KEY_MAPPING_KEY_VALUE, AlgorithmId));
    RtlCopyMemory (K.Mac, V + FIELD_OFFSET (DOT11_CIPHER_KEY_MAPPING_KEY_VALUE, PeerMacAddr), 6);
    if (!K.Delete) {
      Status = ParseKey (K.Algorithm, V + Fixed, KeyLen, &K);
      if (Status != NDIS_STATUS_SUCCESS) {
        OsLog ("exynoswifi: pairwise key: algo %lu, %lu bytes not usable", (ULONG)K.Algorithm, KeyLen);
        break;
      }
      K.HaveRsc = FALSE;                // the receive counter starts at 0
    }
    Status = BusSetKey (A, &K);
  }
  Req->DATA.SET_INFORMATION.BytesRead = Hdr + Bytes;
  return Status;
}

NDIS_STATUS
OidOnBus (PADAPTER A, PNDIS_OID_REQUEST Req)
{
  DOT11_RESET_REQUEST      Reset;
  DOT11_STATUS_INDICATION  Confirm;
  BOOLEAN                  On;
  ULONG                    Length;
  NDIS_STATUS              Status;

  if (Req->RequestType == NdisRequestMethod) {
    //
    // OID_DOT11_RESET_REQUEST: DOT11_RESET_REQUEST in, DOT11_STATUS_INDICATION out.
    //
    if ((Req->DATA.METHOD_INFORMATION.InputBufferLength < sizeof (Reset)) ||
        (Req->DATA.METHOD_INFORMATION.OutputBufferLength < sizeof (Confirm))) {
      Req->DATA.METHOD_INFORMATION.BytesNeeded = MAX (sizeof (Reset), sizeof (Confirm));
      return NDIS_STATUS_INVALID_LENGTH;
    }
    RtlCopyMemory (&Reset, Req->DATA.METHOD_INFORMATION.InformationBuffer, sizeof (Reset));
    OsLog ("exynoswifi: reset (type %lu, default MIB %u)", (ULONG)Reset.dot11ResetType, Reset.bSetDefaultMIB);
    Status = BusReset (A, Reset.bSetDefaultMIB);
    Confirm.uStatusType = DOT11_STATUS_RESET_CONFIRM;
    Confirm.ndisStatus  = Status;
    RtlCopyMemory (Req->DATA.METHOD_INFORMATION.InformationBuffer, &Confirm, sizeof (Confirm));
    Req->DATA.METHOD_INFORMATION.BytesRead    = sizeof (Reset);
    Req->DATA.METHOD_INFORMATION.BytesWritten = sizeof (Confirm);
    return Status;
  }

  Length = Req->DATA.SET_INFORMATION.InformationBufferLength;
  switch (Req->DATA.SET_INFORMATION.Oid) {
    case OID_DOT11_SCAN_REQUEST:
      if (Length < FIELD_OFFSET (DOT11_SCAN_REQUEST_V2, ucBuffer)) {
        Req->DATA.SET_INFORMATION.BytesNeeded = FIELD_OFFSET (DOT11_SCAN_REQUEST_V2, ucBuffer);
        return NDIS_STATUS_INVALID_LENGTH;
      }
      Req->DATA.SET_INFORMATION.BytesRead = Length;
      return BusScan (A, Req->DATA.SET_INFORMATION.InformationBuffer, Length);
    case OID_DOT11_CONNECT_REQUEST:
      return BusConnect (A);
    case OID_DOT11_DISCONNECT_REQUEST:
      return BusDisconnect (A, DOT11_DISASSOC_REASON_OS);
    case OID_DOT11_CIPHER_DEFAULT_KEY:
      return SetDefaultKey (A, Req);
    case OID_DOT11_CIPHER_KEY_MAPPING_KEY:
      return SetKeyMappingKeys (A, Req);
    case OID_DOT11_NIC_POWER_STATE:
      Status = TakeValue (Req, &On, 1);
      return (Status == NDIS_STATUS_SUCCESS) ? BusRadio (A, On) : Status;
    default:
      return NDIS_STATUS_NOT_SUPPORTED;
  }
}

NDIS_STATUS
OidRequest (PADAPTER A, PNDIS_OID_REQUEST Req)
{
  switch (Req->RequestType) {
    case NdisRequestQueryInformation:
    case NdisRequestQueryStatistics:
      return Query (A, Req);
    case NdisRequestSetInformation: {
      NDIS_STATUS  St = Set (A, Req);
      if (St != NDIS_STATUS_SUCCESS && St != NDIS_STATUS_PENDING) {
        OsLog ("exynoswifi: set OID 0x%08lx -> 0x%08lx", Req->DATA.SET_INFORMATION.Oid, (ULONG)St);
      }
      return St;
    }
    case NdisRequestMethod:
      if (Req->DATA.METHOD_INFORMATION.Oid == OID_DOT11_ENUM_BSS_LIST) {
        return QueryBssList (A, Req->DATA.METHOD_INFORMATION.InformationBuffer,
                             Req->DATA.METHOD_INFORMATION.OutputBufferLength,
                             &Req->DATA.METHOD_INFORMATION.BytesWritten, &Req->DATA.METHOD_INFORMATION.BytesNeeded);
      }
      OsLog ("exynoswifi: method OID 0x%08lx not supported", Req->DATA.METHOD_INFORMATION.Oid);
      return NDIS_STATUS_NOT_SUPPORTED;
    default:
      return NDIS_STATUS_NOT_SUPPORTED;
  }
}
