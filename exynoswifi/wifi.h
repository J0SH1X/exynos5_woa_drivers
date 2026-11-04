/** @file
  exynoswifi: v1awifi (SM-P900) Wi-Fi as a Native 802.11 (ExtSTA) NDIS 6.30
  miniport for Windows RT 8.1, so the OS's own WLAN stack drives it:
  network list, profiles, WPA2 (the OS supplicant does the 4-way handshake
  over our data path), the taskbar UI.

  The Broadcom BCM4354 is FullMAC: it scans, authenticates and associates
  itself and moves 802.3 frames.  This driver maps the ExtSTA model onto it:

    OID_DOT11_SCAN_REQUEST       firmware escan; results (WLC_E_ESCAN_RESULT)
                                 feed a BSS cache -> OID_DOT11_ENUM_BSS_LIST
    OID_DOT11_CONNECT_REQUEST    wpa_auth / wsec from the enabled algorithms,
                                 WLC_SET_SSID with the chosen BSSID; the
                                 firmware's association request / response
                                 IEs go into the association completion
    OID_DOT11_CIPHER_*_KEY       WLC_SET_KEY (BcmSetKey)
    data                         802.11 + LLC/SNAP <-> 802.3 (both ways),
                                 every receive with a DOT11_EXTSTA_RECV_CONTEXT

  Board, chip bring-up, interrupt and bus thread as in exynoswlan (which
  stays the simple Ethernet-style driver); the Broadcom core is the UEFI
  tree's Drivers/BcmWlan, built unchanged.  Only the bus thread touches the
  chip: OIDs that need it are queued to it (NDIS_STATUS_PENDING) and
  completed from there, so an OID issued from inside one of our own status
  indications can never deadlock.

  NOT TESTED ON DEVICE YET.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef EXYNOSWIFI_H_
#define EXYNOSWIFI_H_

#define NDIS_MINIPORT_DRIVER  1
#define NDIS630_MINIPORT      1
#include "MmcEnv.h"
#include <ndis.h>

#include "../../uefi/Exynos5420Pkg/Drivers/BcmWlan/BcmSdio.h"

#define WIFI_TAG            'fiwE'
#define SAFETY_POLL_MS      1000
#define FALLBACK_POLL_MS    20
#define DRAIN_ROUNDS        16
#define ETH_FRAME_MAX       1600
#define RX_FRAME_MAX        2048        // 802.11 header + SNAP + payload
#define TX_QUEUE_MAX        64
#define SCRATCH_SIZE        BCM_IOCTL_MAX
#define IND_BUFFER_SIZE     8192        // association completion indication

#define BSS_CACHE_SIZE      48
#define BSS_IE_MAX          1024
#define BSS_AGE_MS          (90 * 1000)
#define SCAN_TIMEOUT_MS     12000
#define CONNECT_TIMEOUT_MS  12000
#define RSSI_POLL_MS        5000
#define EAPOL_SETTLE_MS     50          // 4/4 out unencrypted before the PTK goes in

#define PHY_ID_ERP          0           // 2.4 GHz (802.11g/b, HT on top)
#define PHY_ID_OFDM         1           // 5 GHz (802.11a, HT / VHT on top)
#define PHY_COUNT           2

#define MAX_SCAN_SSIDS      1
#define MAX_DESIRED_SSIDS   1
#define MAX_DESIRED_BSSIDS  4
#define MAX_EXCLUDED_MACS   4
#define MAX_PRIVACY_EXEMPT  8
#define MAX_KEY_MAPPING     4
#define MAX_DEFAULT_KEYS    4
#define MAX_HELD_EAPOL      4
#define ASSOC_IES_MAX       256

//
// Broadcom events and ioctls this driver adds to BcmSdio.h's.
//
#define WLC_E_JOIN          1
#define WLC_E_AUTH          3
#define WLC_E_DEAUTH        5
#define WLC_E_ASSOC         7
#define WLC_E_REASSOC       9
#define WLC_E_DISASSOC      11
#define WLC_E_ROAM          19
#define WLC_E_ESCAN_RESULT  69
#define WLC_E_STATUS_SUCCESS  0
#define WLC_E_STATUS_PARTIAL  8
#define WLC_GET_RATE        12
#define WLC_SET_PASSIVE_SCAN  49
#define WLC_GET_BSS_INFO    136
#define WPA_AUTH_DISABLED   0x0000
#define WPA_AUTH_UNSPECIFIED  0x0002
#define WPA_AUTH_PSK        0x0004
#define WPA2_AUTH_UNSPECIFIED 0x0040

typedef struct {
  BOOLEAN    Valid;
  UCHAR      Bssid[6];
  UCHAR      Ssid[32];
  UCHAR      SsidLength;
  LONG       Rssi;
  ULONG      Channel;
  USHORT     Chanspec;
  ULONG      Frequency;                 // MHz
  ULONG      PhyId;
  USHORT     BeaconPeriod;
  USHORT     Capability;
  ULONGLONG  SeenAt;                    // KeQueryInterruptTime() of the last result
  ULONGLONG  HostTimestamp;             // system time of the last result
  ULONG      IeLength;
  UCHAR      Ies[BSS_IE_MAX];
} BSS_ENTRY;

typedef enum {
  StateIdle,
  StateJoining,
  StateConnected
} CONNECT_STATE;

typedef struct _ADAPTER {
  NDIS_HANDLE        Handle;
  PUCHAR             Mmc;
  PUCHAR             Cmu;
  PUCHAR             GpioRt;
  PUCHAR             GpioT;

  BCM_SDIO           *Bcm;
  UCHAR              Mac[6];
  BOOLEAN            Up;
  BOOLEAN            Paused;
  BOOLEAN            RadioOn;

  //
  // Bus thread, and the one OID handed to it.
  //
  PETHREAD           Thread;
  KEVENT             Wake;
  BOOLEAN            Stop;
  NDIS_HANDLE        Interrupt;
  volatile LONG      IrqPending;
  ULONG              Interrupts;
  PNDIS_OID_REQUEST  PendingOid;        // under Lock

  //
  // TX queue.
  //
  NDIS_SPIN_LOCK     Lock;
  PNET_BUFFER_LIST   TxHead;
  PNET_BUFFER_LIST   TxTail;
  ULONG              TxQueued;
  ULONGLONG          LastEapolTx;       // KeQueryInterruptTime()

  //
  // RX: one frame at a time, NDIS_RECEIVE_FLAGS_RESOURCES.
  //
  NDIS_HANDLE                NblPool;
  PNET_BUFFER_LIST           RxNbl;
  PMDL                       RxMdl;
  PUCHAR                     RxBuffer;
  DOT11_EXTSTA_RECV_CONTEXT  RxContext;

  //
  // EAPOL frames that arrive before the association completion is
  // indicated (802.3, as received), handed up right after it.
  //
  ULONG              HeldCount;
  ULONG              HeldLength[MAX_HELD_EAPOL];
  UCHAR              Held[MAX_HELD_EAPOL][BCM_EAPOL_MAX];

  //
  // Events from the RX handler (poll context) for the thread.
  //
  BOOLEAN            EvScanDone;
  ULONG              EvScanStatus;
  BOOLEAN            EvJoinDone;
  ULONG              EvJoinStatus;
  BOOLEAN            EvLinkUp;
  BOOLEAN            EvLinkDown;
  ULONG              EvLinkReason;      // DOT11_ASSOC_STATUS_xxx

  //
  // Scan / BSS cache.
  //
  BOOLEAN            Scanning;
  ULONGLONG          ScanStart;
  USHORT             ScanSyncId;
  KSPIN_LOCK         BssLock;
  BSS_ENTRY          *Bss;              // BSS_CACHE_SIZE entries

  //
  // Connection.
  //
  CONNECT_STATE      State;
  ULONGLONG          JoinStart;
  ULONGLONG          ConnectedAt;
  ULONGLONG          LastRssiPoll;
  BOOLEAN            AssocStartIndicated;
  UCHAR              Bssid[6];          // the AP we join / joined
  DOT11_SSID         JoinSsid;
  ULONG              ActivePhyId;
  ULONG              ActiveFrequency;
  LONG               Rssi;
  ULONG64            LinkSpeed;         // bps
  USHORT             Aid;

  //
  // ExtSTA MIB (set by the OS).
  //
  ULONG              OperationMode;
  ULONG              MibPhyId;          // OID_DOT11_CURRENT_PHY_ID
  DOT11_SSID         DesiredSsid[MAX_DESIRED_SSIDS];
  ULONG              DesiredSsidCount;
  DOT11_BSS_TYPE     DesiredBssType;
  ULONG              DesiredBssidCount;
  UCHAR              DesiredBssid[MAX_DESIRED_BSSIDS][6];
  ULONG              ExcludedCount;
  UCHAR              Excluded[MAX_EXCLUDED_MACS][6];
  DOT11_AUTH_ALGORITHM    AuthAlgorithm;
  DOT11_CIPHER_ALGORITHM  UnicastCipher;
  DOT11_CIPHER_ALGORITHM  MulticastCipher;
  BOOLEAN            ExcludeUnencrypted;
  ULONG              PrivacyExemptCount;
  DOT11_PRIVACY_EXEMPTION PrivacyExempt[MAX_PRIVACY_EXEMPT];
  ULONG              AutoConfig;
  BOOLEAN            MediaStreaming;
  ULONG              UnreachableThreshold;
  BOOLEAN            UnicastUseGroup;
  BOOLEAN            SafeMode;
  BOOLEAN            HiddenNetwork;
  ULONG              PowerSaving;
  ULONG              DefaultKeyId;
  ULONG              RtsThreshold;
  ULONG              FragThreshold;
  ULONG              DesiredPhyCount;
  ULONG              DesiredPhy[PHY_COUNT + 1];   // DOT11_PHY_ID_ANY or ids
  ULONG              PacketFilter;
  ULONG              Lookahead;
  UCHAR              Country[3];
  ULONG              AssocIesLength;    // OID_DOT11_ASSOCIATION_PARAMS extra IEs
  UCHAR              AssocIes[ASSOC_IES_MAX];

  //
  // Statistics.
  //
  ULONG64            TxFrames;
  ULONG64            RxFrames;
  ULONG64            TxMulticast;
  ULONG64            RxMulticast;
  ULONG64            TxErrors;
  ULONG64            RxDropped;

  UCHAR              Scratch[SCRATCH_SIZE];      // bus thread only
  PUCHAR             IndBuffer;                  // IND_BUFFER_SIZE, bus thread only
  UCHAR              TxRaw[RX_FRAME_MAX];
  UCHAR              TxFrame[ETH_FRAME_MAX + 8];
} ADAPTER, *PADAPTER;

//
// hw.c
//
NDIS_STATUS HwStart (PADAPTER A);
VOID        HwStop (PADAPTER A);
NDIS_STATUS HwRegisterInterrupt (PADAPTER A);
NDIS_STATUS HwStartThread (PADAPTER A);
VOID        HwStopThread (PADAPTER A);
VOID        HwPowerOff (PADAPTER A);
VOID        WifiResetMib (PADAPTER A);

NDIS_STATUS BusScan (PADAPTER A, PDOT11_SCAN_REQUEST_V2 Request, ULONG Length);
NDIS_STATUS BusConnect (PADAPTER A);
NDIS_STATUS BusDisconnect (PADAPTER A, ULONG Reason);
NDIS_STATUS BusReset (PADAPTER A, BOOLEAN SetDefaultMib);
NDIS_STATUS BusRadio (PADAPTER A, BOOLEAN On);

typedef struct {
  ULONG                   Index;
  DOT11_CIPHER_ALGORITHM  Algorithm;
  UCHAR                   Mac[6];
  BOOLEAN                 Pairwise;
  BOOLEAN                 Delete;
  BOOLEAN                 HaveRsc;
  UCHAR                   Rsc[6];
  ULONG                   Length;
  UCHAR                   Key[32];
} KEY_WORK;

NDIS_STATUS BusSetKey (PADAPTER A, KEY_WORK *Key);

ULONG       BssEnumerate (PADAPTER A, PUCHAR Out, ULONG OutSize, ULONG *Total);
VOID        BssFlush (PADAPTER A);

VOID        WifiIndicate (PADAPTER A, NDIS_STATUS Code, PVOID Buffer, ULONG Size);
VOID        WifiIndicateLink (PADAPTER A, BOOLEAN Connected);
ULONGLONG   NowMs (VOID);

//
// oids.c
//
extern const NDIS_OID  gSupportedOids[];
extern const ULONG     gSupportedOidsSize;
NDIS_STATUS OidRequest (PADAPTER A, PNDIS_OID_REQUEST Request);
BOOLEAN     OidNeedsBus (PNDIS_OID_REQUEST Request);
NDIS_STATUS OidOnBus (PADAPTER A, PNDIS_OID_REQUEST Request);

//
// main.c
//
extern DOT11_PHY_ATTRIBUTES  gPhyAttributes[PHY_COUNT];
VOID        FillPhyAttributes (VOID);
extern const DOT11_AUTH_CIPHER_PAIR  gUnicastPairs[];
extern const ULONG                   gUnicastPairCount;
extern const DOT11_AUTH_CIPHER_PAIR  gMulticastPairs[];
extern const ULONG                   gMulticastPairCount;
extern const DOT11_DATA_RATE_MAPPING_ENTRY  gRateTable[];
extern const ULONG                          gRateCount;

#define HDR(h, t, r, s)  do { (h).Type = (UCHAR)(t); (h).Revision = (UCHAR)(r); (h).Size = (USHORT)(s); } while (0)

#endif
