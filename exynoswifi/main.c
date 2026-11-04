/** @file
  exynoswifi: NDIS 6.30 Native 802.11 (ExtSTA) miniport for the SM-P900's
  Broadcom BCM4354.  Driver entry, adapter set-up (general and Native
  802.11 attributes), the PHY and algorithm tables, and the NDIS entry
  points that are not chip work (send queue, pause / restart, halt).  See
  wifi.h for the design, hw.c for the chip side, oids.c for the OIDs.

  ACPI: _HID EXYN5009 (as exynoswlan), Memory32Fixed 0x12210000 + 0x1000
  (dw_mmc1), Interrupt 108.
  Firmware: %SystemRoot%\System32\drivers\bcmdhd_sta.bin, nvram_net.txt.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#define OSLOG_GLOBAL
#include "wifi.h"

#define CMU_PAGE            0x10020000
#define GPIO_RT_PAGE        0x13400000
#define GPIO_T_PAGE         0x13410000

static NDIS_HANDLE  mDriverHandle;

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
DRIVER_INITIALIZE                  DriverEntry;

//
// ---------------------------------------------------------------------------
// Capabilities.
// ---------------------------------------------------------------------------
//

//
// Open, WPA2-PSK and WPA2-Enterprise with CCMP (pairwise); the group
// cipher may also be TKIP (mixed-mode APs).  WPA(1) / TKIP pairwise and
// WEP are left out: untested key layouts.
//
const DOT11_AUTH_CIPHER_PAIR  gUnicastPairs[] = {
  { DOT11_AUTH_ALGO_80211_OPEN, DOT11_CIPHER_ALGO_NONE },
  { DOT11_AUTH_ALGO_RSNA_PSK,   DOT11_CIPHER_ALGO_CCMP },
  { DOT11_AUTH_ALGO_RSNA,       DOT11_CIPHER_ALGO_CCMP },
};
const ULONG  gUnicastPairCount = ARRAY_SIZE (gUnicastPairs);

const DOT11_AUTH_CIPHER_PAIR  gMulticastPairs[] = {
  { DOT11_AUTH_ALGO_80211_OPEN, DOT11_CIPHER_ALGO_NONE },
  { DOT11_AUTH_ALGO_RSNA_PSK,   DOT11_CIPHER_ALGO_CCMP },
  { DOT11_AUTH_ALGO_RSNA_PSK,   DOT11_CIPHER_ALGO_TKIP },
  { DOT11_AUTH_ALGO_RSNA,       DOT11_CIPHER_ALGO_CCMP },
  { DOT11_AUTH_ALGO_RSNA,       DOT11_CIPHER_ALGO_TKIP },
};
const ULONG  gMulticastPairCount = ARRAY_SIZE (gMulticastPairs);

//
// Legacy rates in 500 kb/s units (index = value).  HT / VHT rates are the
// firmware's business; the link speed comes from WLC_GET_RATE.
//
const DOT11_DATA_RATE_MAPPING_ENTRY  gRateTable[] = {
  { 2, 0, 2 }, { 4, 0, 4 }, { 11, 0, 11 }, { 22, 0, 22 }, { 12, 0, 12 }, { 18, 0, 18 },
  { 24, 0, 24 }, { 36, 0, 36 }, { 48, 0, 48 }, { 72, 0, 72 }, { 96, 0, 96 }, { 108, 0, 108 },
};
const ULONG  gRateCount = ARRAY_SIZE (gRateTable);

DOT11_PHY_ATTRIBUTES  gPhyAttributes[PHY_COUNT];

static DOT11_EXTSTA_ATTRIBUTES  mExtSta;

VOID
FillPhyAttributes (VOID)
{
  static const UCHAR  ErpRates[]  = { 2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108 };
  static const UCHAR  OfdmRates[] = { 12, 18, 24, 36, 48, 72, 96, 108 };
  DOT11_PHY_ATTRIBUTES  *P;
  ULONG                 Phy;
  ULONG                 Index;

  RtlZeroMemory (gPhyAttributes, sizeof (gPhyAttributes));
  for (Phy = 0; Phy < PHY_COUNT; Phy++) {
    P = &gPhyAttributes[Phy];
    HDR (P->Header, NDIS_OBJECT_TYPE_DEFAULT, DOT11_PHY_ATTRIBUTES_REVISION_1, sizeof (*P));
    P->PhyType           = (Phy == PHY_ID_ERP) ? dot11_phy_type_erp : dot11_phy_type_ofdm;
    P->bHardwarePhyState = TRUE;
    P->bSoftwarePhyState = TRUE;
    P->bCFPollable       = FALSE;
    P->uMPDUMaxLength    = DOT11_MAX_PDU_SIZE;
    P->TempType          = dot11_temp_type_1;
    P->DiversitySupport  = dot11_diversity_support_dynamic;
    if (Phy == PHY_ID_ERP) {
      P->ERPAttributes.bShortPreambleOptionImplemented = TRUE;
      P->ERPAttributes.bPBCCOptionImplemented          = FALSE;
      P->ERPAttributes.bChannelAgilityPresent          = FALSE;
      P->ERPAttributes.uHRCCAModeSupported             = DOT11_HR_CCA_MODE_ED_ONLY;
      P->ERPAttributes.bERPPBCCOptionImplemented       = FALSE;
      P->ERPAttributes.bDSSSOFDMOptionImplemented      = FALSE;
      P->ERPAttributes.bShortSlotTimeOptionImplemented = TRUE;
      for (Index = 0; Index < sizeof (ErpRates); Index++) {
        P->SupportedDataRatesValue.ucSupportedTxDataRatesValue[Index] = ErpRates[Index];
        P->SupportedDataRatesValue.ucSupportedRxDataRatesValue[Index] = ErpRates[Index];
      }
    } else {
      P->OFDMAttributes.uFrequencyBandsSupported = DOT11_FREQUENCY_BANDS_LOWER | DOT11_FREQUENCY_BANDS_MIDDLE |
                                                   DOT11_FREQUENCY_BANDS_UPPER;
      for (Index = 0; Index < sizeof (OfdmRates); Index++) {
        P->SupportedDataRatesValue.ucSupportedTxDataRatesValue[Index] = OfdmRates[Index];
        P->SupportedDataRatesValue.ucSupportedRxDataRatesValue[Index] = OfdmRates[Index];
      }
    }
    P->uNumberSupportedPowerLevels = 1;
    P->TxPowerLevels[0]            = 100;          // mW
    P->uNumDataRateMappingEntries  = gRateCount;
    RtlCopyMemory (P->DataRateMappingEntries, gRateTable, sizeof (gRateTable));
  }

  RtlZeroMemory (&mExtSta, sizeof (mExtSta));
  HDR (mExtSta.Header, NDIS_OBJECT_TYPE_DEFAULT, DOT11_EXTSTA_ATTRIBUTES_REVISION_3,
       RTL_SIZEOF_THROUGH_FIELD (DOT11_EXTSTA_ATTRIBUTES, uMaxNetworkOffloadListSize));
  mExtSta.uScanSSIDListSize           = MAX_SCAN_SSIDS;
  mExtSta.uDesiredBSSIDListSize       = MAX_DESIRED_BSSIDS;
  mExtSta.uDesiredSSIDListSize        = MAX_DESIRED_SSIDS;
  mExtSta.uExcludedMacAddressListSize = MAX_EXCLUDED_MACS;
  mExtSta.uPrivacyExemptionListSize   = MAX_PRIVACY_EXEMPT;
  mExtSta.uKeyMappingTableSize        = MAX_KEY_MAPPING;
  mExtSta.uDefaultKeyTableSize        = MAX_DEFAULT_KEYS;
  mExtSta.uWEPKeyValueMaxLength       = 13;
  mExtSta.uPMKIDCacheSize             = 0;
  mExtSta.uMaxNumPerSTADefaultKeyTables = 0;
  mExtSta.bStrictlyOrderedServiceClassImplemented = FALSE;
  mExtSta.ucSupportedQoSProtocolFlags = 0;
  mExtSta.bSafeModeImplemented        = FALSE;
  mExtSta.uNumSupportedCountryOrRegionStrings = 0;
  mExtSta.pSupportedCountryOrRegionStrings    = NULL;
  mExtSta.uInfraNumSupportedUcastAlgoPairs = gUnicastPairCount;
  mExtSta.pInfraSupportedUcastAlgoPairs    = (PDOT11_AUTH_CIPHER_PAIR)gUnicastPairs;
  mExtSta.uInfraNumSupportedMcastAlgoPairs = gMulticastPairCount;
  mExtSta.pInfraSupportedMcastAlgoPairs    = (PDOT11_AUTH_CIPHER_PAIR)gMulticastPairs;
  mExtSta.uAdhocNumSupportedUcastAlgoPairs = 0;
  mExtSta.pAdhocSupportedUcastAlgoPairs    = NULL;
  mExtSta.uAdhocNumSupportedMcastAlgoPairs = 0;
  mExtSta.pAdhocSupportedMcastAlgoPairs    = NULL;
  mExtSta.bAutoPowerSaveMode         = FALSE;
  mExtSta.uMaxNetworkOffloadListSize = 0;
}

//
// ---------------------------------------------------------------------------
// Adapter.
// ---------------------------------------------------------------------------
//

static PUCHAR
MapPage (ULONG Phys)
{
  PHYSICAL_ADDRESS  P;

  P.QuadPart = Phys;
  return MmMapIoSpace (P, PAGE_SIZE, MmNonCached);
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
    ExFreePoolWithTag (A->RxBuffer, WIFI_TAG);
  }
  if (A->NblPool != NULL) {
    NdisFreeNetBufferListPool (A->NblPool);
  }
  if (A->IndBuffer != NULL) {
    ExFreePoolWithTag (A->IndBuffer, WIFI_TAG);
  }
  if (A->Bss != NULL) {
    ExFreePoolWithTag (A->Bss, WIFI_TAG);
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
    ExFreePoolWithTag (A->Bcm, WIFI_TAG);
  }
  NdisFreeSpinLock (&A->Lock);
  ExFreePoolWithTag (A, WIFI_TAG);
}

static NDIS_STATUS
SetAttributes (PADAPTER A)
{
  NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES   Reg;
  NDIS_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES        Gen;
  NDIS_MINIPORT_ADAPTER_NATIVE_802_11_ATTRIBUTES  Nat;
  NDIS_STATUS                                     Status;

  RtlZeroMemory (&Reg, sizeof (Reg));
  HDR (Reg.Header, NDIS_OBJECT_TYPE_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES,
       NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES_REVISION_1, NDIS_SIZEOF_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES_REVISION_1);
  Reg.MiniportAdapterContext = A;
  Reg.AttributeFlags         = NDIS_MINIPORT_ATTRIBUTES_NO_HALT_ON_SUSPEND;
  Reg.InterfaceType          = NdisInterfaceInternal;
  Status = NdisMSetMiniportAttributes (A->Handle, (PNDIS_MINIPORT_ADAPTER_ATTRIBUTES)&Reg);
  if (Status != NDIS_STATUS_SUCCESS) {
    OsLog ("exynoswifi: registration attributes: 0x%08lx", Status);
    return Status;
  }

  RtlZeroMemory (&Gen, sizeof (Gen));
  HDR (Gen.Header, NDIS_OBJECT_TYPE_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES,
       NDIS_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES_REVISION_1, NDIS_SIZEOF_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES_REVISION_1);
  Gen.MediaType          = NdisMediumNative802_11;
  Gen.PhysicalMediumType = NdisPhysicalMediumNative802_11;
  Gen.MtuSize            = 1500;
  Gen.MaxXmitLinkSpeed   = 150000000ULL;
  Gen.MaxRcvLinkSpeed    = 150000000ULL;
  Gen.XmitLinkSpeed      = NDIS_LINK_SPEED_UNKNOWN;
  Gen.RcvLinkSpeed       = NDIS_LINK_SPEED_UNKNOWN;
  Gen.MediaConnectState  = MediaConnectStateDisconnected;
  Gen.MediaDuplexState   = MediaDuplexStateHalf;
  Gen.LookaheadSize      = 1500;
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
  Gen.IfType             = IF_TYPE_IEEE80211;
  Gen.IfConnectorPresent = TRUE;
  Gen.SupportedStatistics = NDIS_STATISTICS_XMIT_OK_SUPPORTED | NDIS_STATISTICS_RCV_OK_SUPPORTED |
                            NDIS_STATISTICS_XMIT_ERROR_SUPPORTED | NDIS_STATISTICS_RCV_DISCARDS_SUPPORTED;
  Gen.SupportedOidList       = (PNDIS_OID)gSupportedOids;
  Gen.SupportedOidListLength = gSupportedOidsSize;
  Status = NdisMSetMiniportAttributes (A->Handle, (PNDIS_MINIPORT_ADAPTER_ATTRIBUTES)&Gen);
  if (Status != NDIS_STATUS_SUCCESS) {
    OsLog ("exynoswifi: general attributes: 0x%08lx", Status);
    return Status;
  }

  RtlZeroMemory (&Nat, sizeof (Nat));
  HDR (Nat.Header, NDIS_OBJECT_TYPE_MINIPORT_ADAPTER_NATIVE_802_11_ATTRIBUTES,
       NDIS_MINIPORT_ADAPTER_802_11_ATTRIBUTES_REVISION_3, NDIS_SIZEOF_MINIPORT_ADAPTER_NATIVE_802_11_ATTRIBUTES_REVISION_3);
  Nat.OpModeCapability = DOT11_OPERATION_MODE_EXTENSIBLE_STATION;
  Nat.NumOfTXBuffers   = TX_QUEUE_MAX;
  Nat.NumOfRXBuffers   = 1;
  Nat.MultiDomainCapabilityImplemented = FALSE;
  Nat.NumSupportedPhys       = PHY_COUNT;
  Nat.SupportedPhyAttributes = gPhyAttributes;
  Nat.ExtSTAAttributes       = &mExtSta;
  Status = NdisMSetMiniportAttributes (A->Handle, (PNDIS_MINIPORT_ADAPTER_ATTRIBUTES)&Nat);
  if (Status != NDIS_STATUS_SUCCESS) {
    OsLog ("exynoswifi: native 802.11 attributes: 0x%08lx", Status);
  }
  return Status;
}

NDIS_STATUS
mpInitialize (NDIS_HANDLE Handle, NDIS_HANDLE DriverContext, PNDIS_MINIPORT_INIT_PARAMETERS Params)
{
  NET_BUFFER_LIST_POOL_PARAMETERS  Pool;
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  PADAPTER                         A;
  ULONG                            Index;
  NDIS_STATUS                      Status;

  UNREFERENCED_PARAMETER (DriverContext);

  A = ExAllocatePoolWithTag (NonPagedPool, sizeof (*A), WIFI_TAG);
  if (A == NULL) {
    return NDIS_STATUS_RESOURCES;
  }
  RtlZeroMemory (A, sizeof (*A));
  A->Handle    = Handle;
  A->Paused    = TRUE;
  A->Lookahead = 1500;
  NdisAllocateSpinLock (&A->Lock);
  KeInitializeSpinLock (&A->BssLock);
  KeInitializeEvent (&A->Wake, SynchronizationEvent, FALSE);
  WifiResetMib (A);

  if (Params->AllocatedResources != NULL) {
    for (Index = 0; Index < Params->AllocatedResources->Count; Index++) {
      Desc = &Params->AllocatedResources->PartialDescriptors[Index];
      if ((Desc->Type == CmResourceTypeMemory) && (A->Mmc == NULL)) {
        A->Mmc = MmMapIoSpace (Desc->u.Memory.Start, PAGE_SIZE, MmNonCached);
      }
    }
  }
  A->Cmu       = MapPage (CMU_PAGE);
  A->GpioRt    = MapPage (GPIO_RT_PAGE);
  A->GpioT     = MapPage (GPIO_T_PAGE);
  A->Bcm       = ExAllocatePoolWithTag (NonPagedPool, sizeof (BCM_SDIO), WIFI_TAG);
  A->RxBuffer  = ExAllocatePoolWithTag (NonPagedPool, RX_FRAME_MAX, WIFI_TAG);
  A->IndBuffer = ExAllocatePoolWithTag (NonPagedPool, IND_BUFFER_SIZE, WIFI_TAG);
  A->Bss       = ExAllocatePoolWithTag (NonPagedPool, sizeof (BSS_ENTRY) * BSS_CACHE_SIZE, WIFI_TAG);
  if ((A->Mmc == NULL) || (A->Cmu == NULL) || (A->GpioRt == NULL) || (A->GpioT == NULL) || (A->Bcm == NULL) ||
      (A->RxBuffer == NULL) || (A->IndBuffer == NULL) || (A->Bss == NULL)) {
    OsLog ("exynoswifi: resources missing (dw_mmc1 %p)", A->Mmc);
    FreeAdapter (A);
    return NDIS_STATUS_RESOURCES;
  }
  RtlZeroMemory (A->Bcm, sizeof (BCM_SDIO));
  RtlZeroMemory (A->Bss, sizeof (BSS_ENTRY) * BSS_CACHE_SIZE);

  RtlZeroMemory (&Pool, sizeof (Pool));
  HDR (Pool.Header, NDIS_OBJECT_TYPE_DEFAULT, NET_BUFFER_LIST_POOL_PARAMETERS_REVISION_1,
       NDIS_SIZEOF_NET_BUFFER_LIST_POOL_PARAMETERS_REVISION_1);
  Pool.fAllocateNetBuffer = TRUE;
  Pool.PoolTag            = WIFI_TAG;
  A->NblPool = NdisAllocateNetBufferListPool (Handle, &Pool);
  A->RxMdl   = (A->NblPool != NULL) ? NdisAllocateMdl (Handle, A->RxBuffer, RX_FRAME_MAX) : NULL;
  A->RxNbl   = (A->RxMdl != NULL) ? NdisAllocateNetBufferAndNetBufferList (A->NblPool, 0, 0, A->RxMdl, 0, 0) : NULL;
  if (A->RxNbl == NULL) {
    FreeAdapter (A);
    return NDIS_STATUS_RESOURCES;
  }

  Status = HwStart (A);
  if (Status != NDIS_STATUS_SUCCESS) {
    FreeAdapter (A);
    return Status;
  }
  Status = SetAttributes (A);
  if (Status == NDIS_STATUS_SUCCESS) {
    Status = HwRegisterInterrupt (A);
  }
  if (Status == NDIS_STATUS_SUCCESS) {
    Status = HwStartThread (A);
    if (Status != NDIS_STATUS_SUCCESS) {
      NdisMDeregisterInterruptEx (A->Interrupt);
    }
  }
  if (Status != NDIS_STATUS_SUCCESS) {
    HwStop (A);
    FreeAdapter (A);
    return Status;
  }
  OsLog ("exynoswifi: Native 802.11 adapter ready (ExtSTA, %lu PHYs)", (ULONG)PHY_COUNT);
  return NDIS_STATUS_SUCCESS;
}

VOID
mpHalt (NDIS_HANDLE Context, NDIS_HALT_ACTION Action)
{
  PADAPTER  A = Context;

  UNREFERENCED_PARAMETER (Action);

  HwStopThread (A);
  if (A->Interrupt != NULL) {
    NdisMDeregisterInterruptEx (A->Interrupt);
  }
  HwStop (A);
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
  A->Paused = FALSE;
  return NDIS_STATUS_SUCCESS;
}

VOID
mpSend (NDIS_HANDLE Context, PNET_BUFFER_LIST List, NDIS_PORT_NUMBER Port, ULONG Flags)
{
  PADAPTER          A = Context;
  PNET_BUFFER_LIST  Last;
  PNET_BUFFER_LIST  Nbl;
  ULONG             Count;
  BOOLEAN           Reject;
  NDIS_STATUS       Status;

  UNREFERENCED_PARAMETER (Port);

  for (Count = 1, Last = List; NET_BUFFER_LIST_NEXT_NBL (Last) != NULL; Last = NET_BUFFER_LIST_NEXT_NBL (Last)) {
    Count++;
  }

  NdisAcquireSpinLock (&A->Lock);
  Status = A->Paused ? NDIS_STATUS_PAUSED :
           (A->State != StateConnected) ? NDIS_STATUS_MEDIA_DISCONNECTED :
           (A->TxQueued + Count > TX_QUEUE_MAX) ? NDIS_STATUS_RESOURCES : NDIS_STATUS_SUCCESS;
  Reject = Status != NDIS_STATUS_SUCCESS;
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
    for (Nbl = List; Nbl != NULL; Nbl = NET_BUFFER_LIST_NEXT_NBL (Nbl)) {
      NET_BUFFER_LIST_STATUS (Nbl) = Status;
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
  // Receives are indicated with NDIS_RECEIVE_FLAGS_RESOURCES.
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
  UNREFERENCED_PARAMETER (Action);
  HwPowerOff ((PADAPTER)Context);
}

NDIS_STATUS
mpOidRequest (NDIS_HANDLE Context, PNDIS_OID_REQUEST Request)
{
  PADAPTER     A = Context;
  NDIS_STATUS  Status;

  if (!OidNeedsBus (Request)) {
    return OidRequest (A, Request);
  }

  //
  // Chip work: the bus thread does it and completes the request.  NDIS
  // has at most one such (non-statistics) request outstanding.
  //
  NdisAcquireSpinLock (&A->Lock);
  if (A->PendingOid == NULL) {
    A->PendingOid = Request;
    Status = NDIS_STATUS_PENDING;
  } else {
    Status = NDIS_STATUS_NOT_ACCEPTED;
  }
  NdisReleaseSpinLock (&A->Lock);
  if (Status == NDIS_STATUS_PENDING) {
    KeSetEvent (&A->Wake, 0, FALSE);
  } else {
    OsLog ("exynoswifi: OID 0x%08lx refused, another one is pending", Request->DATA.SET_INFORMATION.Oid);
  }
  return Status;
}

VOID
mpCancelOid (NDIS_HANDLE Context, PVOID RequestId)
{
  UNREFERENCED_PARAMETER (Context);
  UNREFERENCED_PARAMETER (RequestId);
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  NDIS_MINIPORT_DRIVER_CHARACTERISTICS  Chars;
  NDIS_STATUS                           Status;

  OsLogInit ();
  OsLog ("exynoswifi: DriverEntry (built " __DATE__ " " __TIME__ ")");
  FillPhyAttributes ();

  RtlZeroMemory (&Chars, sizeof (Chars));
  HDR (Chars.Header, NDIS_OBJECT_TYPE_MINIPORT_DRIVER_CHARACTERISTICS, NDIS_MINIPORT_DRIVER_CHARACTERISTICS_REVISION_2,
       NDIS_SIZEOF_MINIPORT_DRIVER_CHARACTERISTICS_REVISION_2);
  Chars.MajorNdisVersion   = 6;
  Chars.MinorNdisVersion   = 30;
  Chars.MajorDriverVersion = 1;
  Chars.MinorDriverVersion = 0;
  Chars.InitializeHandlerEx         = mpInitialize;
  Chars.HaltHandlerEx               = mpHalt;
  Chars.UnloadHandler               = mpUnload;
  Chars.PauseHandler                = mpPause;
  Chars.RestartHandler              = mpRestart;
  Chars.OidRequestHandler           = mpOidRequest;
  Chars.SendNetBufferListsHandler   = mpSend;
  Chars.ReturnNetBufferListsHandler = mpReturn;
  Chars.CancelSendHandler           = mpCancelSend;
  Chars.DevicePnPEventNotifyHandler = mpPnpEvent;
  Chars.ShutdownHandlerEx           = mpShutdown;
  Chars.CancelOidRequestHandler     = mpCancelOid;

  Status = NdisMRegisterMiniportDriver (DriverObject, RegistryPath, NULL, &Chars, &mDriverHandle);
  if (Status != NDIS_STATUS_SUCCESS) {
    OsLog ("exynoswifi: NdisMRegisterMiniportDriver failed: 0x%08lx", Status);
  }
  return Status;
}
