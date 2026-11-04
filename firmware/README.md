# Firmware blobs the drivers need

All files here are **unmodified, extracted from the stock Samsung ROM** of this tablet (not rebuilt, not patched):

| | |
|---|---|
| Device | Samsung Galaxy Note Pro 12.2 Wi-Fi, SM-P900, board `v1awifi` |
| Stock build | **LRX22G.P900XXS0BPL2** (Android 5.0.2, changelist 5726670, built Thu Dec 1 19:55:58 KST 2016) |
| Fingerprint | `samsung/v1awifixx/v1awifi:5.0.2/LRX22G/P900XXS0BPL2:user/release-keys` |
| Source image | `system.raw` = the system partition of `AP_P900XXS0BPL2_CL5726670_QB11754156_REV00_user_low_ship.tar` (`~/Downloads/system.raw`, ext4; read with `debugfs`) |

The blobs are Samsung / Broadcom / Sony / Samsung-LSI property; they are kept in the repository because the drivers
cannot work without them. Keep the repository private.

| File | Stock path | Used by | SHA-256 |
|---|---|---|---|
| `fimc_is_fw2_IMX134.bin` | `/system/vendor/firmware/` | exynoscam (ISP Cortex-A5 firmware, also drives the front sensor) | `6146aeca3b12c1daf223fecf2e42ba0d1b333a6a8729f6fc1cf5f5ab686ca611` |
| `setfile_imx134.bin` | `/system/vendor/firmware/` | exynoscam (rear IMX134 tuning) | `72a79b46494e1813b20e285c7f1ea9aac563ded0d82fcffa58b8dc34ec525edf` |
| `setfile_6b2.bin` | `/system/vendor/firmware/` | exynoscam (front S5K6B2 tuning) | `caed1bbf46aef6caf1814841bf08bf003897b30eb1eaed8e689ad1c4f20f316d` |
| `bcm4350_V0301.0591.hcd` | `/system/vendor/firmware/` | exynosbt (Bluetooth patch RAM) | `1a7838154622ba44e3a5f730e113c956be129e4e9496156cba6db75b2e1e5e38` |
| `bcm4335_V0105.0541.hcd` | `/system/vendor/firmware/` | exynosbt | `8bc0eb6b512d475e576ac808e2af968db9e69b7c214660306387934c3a6f8918` |
| `bcmdhd_sta.bin` | `/system/etc/wifi/` | exynoswlan / exynoswifi (BCM4354 A1 Wi-Fi firmware, 7.26.5.18) | `e6ece55ae151bf2525b29acfa20241b811c6fb6e28944626f95c8f5db85a2bfb` |
| `nvram_net.txt_4354_a1` | `/system/etc/wifi/` | exynoswlan / exynoswifi (installed on the tablet as `nvram_net.txt`) | `634864fac348e98034157ce0d8ff6430e4bd7efb419eaa854bb95011cc441448` |

Re-extract (ext4 image, no mounting needed):

    debugfs -R "dump /vendor/firmware/fimc_is_fw2_IMX134.bin fimc_is_fw2_IMX134.bin" system.raw

`build.sh` copies the camera and Bluetooth files into `Artifacts/exynoscam` and `Artifacts/exynosbt` (the INFs install
them into `System32\drivers`); the Wi-Fi files are copied to the tablet by hand (see `exynoswlan`).
