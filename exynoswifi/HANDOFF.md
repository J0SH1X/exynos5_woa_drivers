# exynoswifi: handoff notes

Goal: a Native 802.11 (ExtSTA) NDIS 6.30 miniport for the Wi-Fi chip of the
Samsung Galaxy Note Pro 12.2 (SM-P900, "v1awifi", Exynos5420) on Windows RT 8.1.
With it, Windows' own WLAN service manages the adapter: network list, saved
profiles, WPA2 handshake and the taskbar UI.

This is a hardware port. All sources are public: the Samsung Android kernel
for this tablet (`~/exynos5/android_kernel_samsung_exynos5420`), Linux
brcmfmac/bcmdhd as references, the Windows WDK/SDK headers, and the project's
own UEFI code.

## Status (2026-11-02, second session)

| File | State |
|---|---|
| `wifi.h` | adapter struct, constants, function list (revised, see "Design changes") |
| `hw.c` | written: board, bring-up, interrupt, bus thread, events, BSS cache, scan / connect / disconnect / reset / radio / keys, 802.11 <-> 802.3 data path |
| `oids.c` | written: general, 802.11 MIB and ExtSTA OIDs; unhandled OIDs are logged |
| `main.c` | written: DriverEntry, adapter init with general + Native 802.11 attributes (2 PHYs, ExtSTA rev 3), send queue, pause / restart, halt |
| `exynoswifi.inf` | written (Net class, `*IfType` 71, `*MediaType` 16, `*PhysicalMediaType` 9, `LowerRange "wlan,ethernet"`) |
| `build.sh` entry | added (also in the Artifacts/ loop, like exynoswlan) |
| `MmcEnv.h`, `bcmsdio.c`, `bcmwpa.c`, `dwmci.c` | unchanged copies from `../exynoswlan` |

It compiles and links with no warnings (`exynoswifi/build/exynoswifi.sys`;
built alone with build-driver.sh, so `.buildnum` and `Artifacts/` are
unchanged). **Never run on the device.** Not staged to `~/exynos5/Artifacts5`,
not committed.

### Design changes against the first notes

- No RunOnBus(). OIDs that need the chip (`SCAN_REQUEST`, `CONNECT_REQUEST`,
  `DISCONNECT_REQUEST`, `CIPHER_DEFAULT_KEY`, `CIPHER_KEY_MAPPING_KEY`,
  `NIC_POWER_STATE` sets, `RESET_REQUEST` method) are parked in
  `A->PendingOid`, `mpOidRequest` returns `NDIS_STATUS_PENDING`, and the bus
  thread runs them (`OidOnBus`) and calls `NdisMOidRequestComplete`. An OID
  that nwifi issues from inside one of our indications just waits for the
  next loop, so it cannot deadlock.
- Bus thread order per wake-up: drain the chip, ack the interrupt, process
  events, send the TX queue, run the pending OID, process events again.
  TX before the OID means a queued 4/4 leaves before the pairwise key. On
  top of that, `BusSetKey` waits until 50 ms after the last EAPOL send, as
  BcmWpa.c does.
- Connect: the best cached BSS (desired SSID, desired / excluded BSSID,
  desired PHY, best RSSI) is joined with its BSSID + chanspec. If the cache
  has none, the firmware joins by SSID. Association start is then
  indicated once the join succeeds, with the BSSID from `WLC_GET_BSSID`.
- Association completion: request body = `assoc_info` fixed fields +
  `assoc_req_ies`; response body = fixed fields + `assoc_resp_ies`; beacon
  body = 12 fixed bytes + IEs from `WLC_GET_BSS_INFO`, which also updates the
  cache entry, the channel and the RSSI. `bPortAuthorized` FALSE. Link speed
  from `WLC_GET_RATE`.
- `State = Connected` is set *before* the association completion is
  indicated (nwifi may send 2/4 from inside it). EAPOL frames received while
  joining are held (4 max) and indicated after the link-up indication.
- RSSI is polled every 5 s while connected (`WLC_GET_RSSI`) and indicated
  as `NDIS_STATUS_DOT11_LINK_QUALITY`.
- Algorithm pairs: unicast open/none, RSNA-PSK/CCMP, RSNA/CCMP; multicast
  also TKIP (mixed-mode APs). TKIP keys are passed as TK + MIC keys
  (PTK / GTK order) and BcmSetKey swaps the MIC halves. **This is the
  least certain part**: whether Windows hands the MIC keys in that order
  is not verified. WEP / WPA1 are not advertised.
- Scans are always broadcast (no SSID list, `uScanSSIDListSize` 1). Hidden
  networks rely on the firmware's join-by-SSID.

### Open points / what to watch in the first test

1. The OID log (`query/set/method OID 0x... not supported`): what nwifi
   still asks for.
2. The ExtSTA attribute revision (3, sized through
   `uMaxNetworkOffloadListSize`) and the association completion revision
   (2): if nwifi refuses the adapter or the association, look here first.
3. `ENUM_BSS_LIST` entries are packed with no padding (per the
   documentation). If nwifi faults on unaligned 64-bit fields on ARM, pad
   them to 8 bytes.
4. Does the firmware's escan data start where `HandleEvent` expects
   (12-byte `wl_escan_result_t` header, then `wl_bss_info_t` v109)? Log
   `bss_count` / SSIDs if the network list stays empty.
5. Power management (OID_PNP_*) is not handled, as in exynoswlan.

## Do not touch

- `../exynoswlan`: the working Ethernet-style Wi-Fi driver. Version 1.0.0.12
  was verified on the device (DHCP, ping, internet). It stays the fallback.
- The shared Broadcom core `~/exynos5/uefi/Exynos5420Pkg/Drivers/BcmWlan/`
  (`BcmSdio.c`, `BcmWpa.c`, `BcmSdio.h`) is also used by UEFI's WlanTestApp
  and by exynoswlan. Only additive changes. One was made for this driver:
  `BcmSetKey()` (plus the `BCM_CRYPTO_ALGO_*` constants), a public wrapper
  around the existing key-install routine in `BcmWpa.c`.

## How the pieces fit

The chip is a Broadcom BCM4354 rev A1 ("FullMAC"). The firmware scans,
authenticates and associates by itself and exchanges 802.3 frames with the host.
Windows' Native Wi-Fi stack expects an 802.11-shaped adapter, so the driver
translates:

| Windows (ExtSTA) | Chip |
|---|---|
| `OID_DOT11_SCAN_REQUEST` | firmware "escan" iovar; results arrive as `WLC_E_ESCAN_RESULT` (69) events, one `wl_bss_info_t` each (status 8 = partial, 0 = done) |
| `OID_DOT11_ENUM_BSS_LIST` | the driver's BSS cache, filled from those events (IEs at `ie_offset`/`ie_length` of the bss info) |
| `OID_DOT11_CONNECT_REQUEST` | `wpa_auth` + `wsec` iovars from the enabled auth/cipher algorithms, then `WLC_SET_SSID` (26) with join params (SSID + BSSID) |
| association completion indication | built from the `assoc_info`, `assoc_req_ies`, `assoc_resp_ies` iovars plus the beacon IEs from the cache |
| `OID_DOT11_CIPHER_KEY_MAPPING_KEY`, `OID_DOT11_CIPHER_DEFAULT_KEY` | `BcmSetKey()` (`WLC_SET_KEY`): pairwise with the peer MAC, group with the key index and the receive counter from `ucIV48Counter` |
| `OID_DOT11_DISCONNECT_REQUEST` | `WLC_DISASSOC` (52) |
| send (802.11 header + LLC/SNAP) | rebuilt as 802.3 (DA = addr3, SA = addr2 for ToDS frames) and sent on SDPCM channel 2 with a 4-byte BDC header |
| receive (802.3 from the chip) | rebuilt as an 802.11 data frame (FromDS: addr1 = DA, addr2 = BSSID, addr3 = SA) + SNAP; every receive carries a `DOT11_EXTSTA_RECV_CONTEXT` in `NET_BUFFER_LIST_INFO(nbl, MediaSpecificInformation)` |

The WPA2 4-way handshake is done by Windows: EAPOL frames travel through the
normal data path. So **do not call `BcmJoin()` or `BcmWpaSetup()`** here.
`Bcm->WpaActive` must stay FALSE, or the core keeps EAPOL frames for its own
supplicant.

Indications on connect: `NDIS_STATUS_DOT11_CONNECTION_START`, then
`ASSOCIATION_START`, `ASSOCIATION_COMPLETION`, `CONNECTION_COMPLETION`, then
`NDIS_STATUS_LINK_STATE`. If association start was indicated, a failure still
needs association completion (failure) plus connection completion. A lost link
gets `NDIS_STATUS_DOT11_DISASSOCIATION` plus link down. A finished scan gets
`NDIS_STATUS_DOT11_SCAN_CONFIRM` (buffer: one NDIS_STATUS).

## Reusable from exynoswlan (copy, keep behaviour)

All in `../exynoswlan/exynoswlan.c`:
- Board: `WlanPower` (WLAN_EN = GPY7(7)), `PinsInit` (GPC1 0,1,3-6 function 2;
  **GPC1(2) is GPS_EN, never touch it**), `ClockInit` (sclk_dwmci1 from CPLL),
  `LoadFile` (firmware from `\SystemRoot\System32\drivers\bcmdhd_sta.bin` and
  `nvram_net.txt`).
- `StartChip`: attach, download, bus init, `cur_etheraddr`, `WLC_UP`.
- Interrupt: `mpIsr`/`mpDpc`/`mpEnable`/`mpDisableInterrupt`/`SyncInterruptDone`
  and `RegisterInterrupt`. The ISR masks INTMASK bit 16; the DPC wakes the bus
  thread.
- Bus thread: `BusThread` (drain with `BcmSdioPoll`, then `BcmSdioInterruptDone`
  under `NdisMSynchronizeWithInterruptEx`, then TX), the TX queue (`mpSend`,
  `TakeQueue`, `SendList`) and the RX NBL setup.

Rules learned the hard way (all in the shared core or exynoswlan already):
- Create the bus thread **after** `BcmSdioEnableInterrupts()`. Only that one
  thread may touch the chip.
- The firmware sends no events until `event_msgs` is set. This driver must
  include 69 (ESCAN_RESULT) on top of the usual set: 0 SET_SSID, 1 JOIN,
  3 AUTH, 5/6 DEAUTH(_IND), 7 ASSOC, 9 REASSOC, 11/12 DISASSOC(_IND),
  16 LINK, 19 ROAM.
- Set iovars `mpc` 0 (radio stays up for scans) and `roam_off` 1 (no silent
  firmware roaming, since the driver does not report roams).
- `bus:rxglom` 0 and the RX resync are in the core already.
- The Samsung NVRAM line `muxenab=0x10` is dropped by the core so the SDIO
  in-band interrupt works (exynoswlan 1.0.0.16 tests that; not verified yet).
- The firmware has no `rand` iovar and no in-firmware supplicant (`sup_wpa`
  returns BCME -23).

## Firmware structure layouts used (little endian)

- `wl_event_msg_t` (big endian, after 24 bytes of ether + bcmeth header):
  version 2, flags 2, event_type 4, status 4, reason 4, auth_type 4,
  datalen 4, addr 6, ifname 16, ifidx 1, bsscfgidx 1. 48 bytes; data follows.
- escan request ("escan" iovar, 72 bytes): version u32 = 1, action u16 = 1
  (start), sync_id u16, then wl_scan_params: ssid {u32 len, 32 bytes},
  bssid ff:ff:ff:ff:ff:ff, bss_type u8 = 2 (any), scan_type u8 = 0,
  nprobes/active/passive/home time s32 = -1, channel_num u32 = 0.
- escan result event data: buflen u32, version u32, sync_id u16,
  bss_count u16, then wl_bss_info.
- `wl_bss_info_t` v109: BSSID @8, beacon_period @14, capability @16,
  SSID_len @18, SSID @19, chanspec @72 (band 0xC000 = 5 GHz), RSSI s16 @78,
  ctl_ch @88, ie_offset u16 @116, ie_length u32 @120 (fixed part 128 bytes).
- join params for `WLC_SET_SSID` (52 bytes): ssid {u32 len, 32 bytes},
  bssid[6], 2 pad, chanspec_num u32 = 0, chanspec_list u16, 2 pad.
- `assoc_info` iovar: req_len @0, resp_len @4, flags @8 (bit 0 = reassoc),
  req capability @12, listen interval @14, reassoc BSSID @16,
  resp capability @22, status @24, AID @26.
- wpa_auth: 0 open, 0x04 WPA-PSK, 0x02 WPA-802.1X, 0x80 WPA2-PSK, 0x40 WPA2-802.1X.
  wsec: 1 WEP, 2 TKIP, 4 AES.
- wl_wsec_key_t is handled by `BcmSetKey()` (164 bytes; algo 2 TKIP, 4 AES).

## Windows side notes

- Headers: `~/s2/winsdk/splat/sdk/include/shared/windot11.h` (ExtSTA OIDs and
  structs; the `NWF_MANDATORY_OID` ones are the required set) and `wlantypes.h`;
  `NDIS_MINIPORT_ADAPTER_NATIVE_802_11_ATTRIBUTES` is in the WDK `km/ndis.h`.
- General attributes: `NdisMediumNative802_11`, `NdisPhysicalMediumNative802_11`,
  IfType 71. Native attributes: OpModeCapability
  `DOT11_OPERATION_MODE_EXTENSIBLE_STATION`, two PHYs planned (ERP 2.4 GHz,
  OFDM 5 GHz) and ExtSTA attributes with the auth/cipher pairs
  (open/none, RSNA-PSK/CCMP).
- `OID_DOT11_ENUM_BSS_LIST` and `OID_DOT11_RESET_REQUEST` are method requests
  (`DATA.METHOD_INFORMATION`).
- Native Wi-Fi may issue OIDs while the driver indicates status: handled by
  the pending-OID design above.
- Build: `../build.sh` (exynoswifi entry), or alone:
  `EXTRA_CFLAGS="-Iexynoswlan/build/ndisfix -Wno-extra-tokens" WDM=1 ./build-driver.sh exynoswifi exynoswifi/main.c exynoswifi/hw.c exynoswifi/oids.c exynoswifi/bcmsdio.c exynoswifi/bcmwpa.c exynoswifi/dwmci.c -- <LIB>/ndis.lib <LIB>/ntstrsafe.lib`
  (the ndisfix header is generated by build.sh).

## Testing (when the user asks)

- Install over exynoswlan with `dism /ForceUnsigned`. The tablet's OS log streams
  over USB (`Tools/usblog.py` in the UEFI tree). Log every OID that is not
  handled; that list is the quickest way to find what Native Wi-Fi still needs.
- Router: OpenWrt at 10.0.0.1 (`logread`, `iw dev phy1-ap0 station get <mac>`)
  shows association, the 4-way handshake and DHCP from the AP side.
- The current MAC is the Broadcom default `00:90:4c:16:70:01`. The real one
  is in Android's `/efs/wifi/.mac.info` (TODO).
