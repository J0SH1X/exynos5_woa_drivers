#!/bin/sh
# Build the Exynos5420 (v1awifi) Windows RT 8.1 drivers into Artifacts/<name>/
# (<name>.sys + <name>.inf), ready for
#   dism /Image:<mount> /Add-Driver /Driver:Artifacts/<name>/<name>.inf /ForceUnsigned
#
# Same toolchain as ~/s2/ExynosWinDrivers (clang + lld-link, WDK 10.0.19041
# ARM, KMDF 1.13).  Drivers log to the OS log ring (common/oslog.h), streamed
# over USB by exynosdwc3.  LOAD_TEST=1 builds load tests that bugcheck as soon
# as Windows starts them:
#   exynosdwc3  0x55534233 "USB3"     exynosi2c   0x49324321 "I2C!"
#   exynosgpio  0x4750494F "GPIO"     mxt1664s    0x4D585421 "MXT!"
#   wacomg9     0x57414321 "WAC!"     exynosemmc  0x454D4D43 "EMMC"
#   (exynosbatt has no load test)
set -e
cd "$(dirname "$0")"

LIB=${WDK:-$HOME/s2/winsdk/wdk10}/Lib/10.0.19041.0/km/arm
FIRMWARE=${FIRMWARE:-$HOME/Downloads/mount/vendor/firmware}
if [ -n "$LOAD_TEST" ]; then
  export EXTRA_CFLAGS="$EXTRA_CFLAGS -DLOAD_TEST=$LOAD_TEST"
fi

./build-driver.sh exynosdwc3 exynosdwc3/exynosdwc3.c -- "$LIB/ntstrsafe.lib"
./build-driver.sh exynosi2c exynosi2c/exynosi2c.c -- "$LIB/spb/1.0/spbcxstubs.lib" "$LIB/ntstrsafe.lib"
./build-driver.sh exynosgpio exynosgpio/exynosgpio.c -- "$LIB/msgpioclxstub.lib" "$LIB/ntstrsafe.lib"
# SPI controllers (SpbCx); the controller code is UEFI's SpiDxe/SpiCore.c.
./build-driver.sh exynosspi exynosspi/exynosspi.c exynosspi/spicore.c -- "$LIB/spb/1.0/spbcxstubs.lib" "$LIB/ntstrsafe.lib"
./build-driver.sh mxt1664s mxt1664s/mxt1664s.c -- "$LIB/ntstrsafe.lib"
./build-driver.sh wacomg9 wacomg9/wacomg9.c -- "$LIB/ntstrsafe.lib"
./build-driver.sh exynosbtn exynosbtn/exynosbtn.c -- "$LIB/ntstrsafe.lib"
# UART0 (SerCx2, PIO + PL330 system DMA) and the Broadcom Bluetooth serial
# HCI bus driver on top of it (child MS_BTHX_BTHMINI -> inbox BthMini).
./build-driver.sh exynosuart exynosuart/exynosuart.c -- "$LIB/sercx/2.0/sercxstubs.lib" "$LIB/ntstrsafe.lib"
./build-driver.sh exynosbt exynosbt/exynosbt.c -- "$LIB/ntstrsafe.lib"
# Mali-T628 hardware base (power, interrupts, self-test); the core comes
# from the UEFI tree (exynosgpu/cores.c).
./build-driver.sh exynosgpu exynosgpu/exynosgpu.c exynosgpu/cores.c -- "$LIB/ntstrsafe.lib"
# Display-only WDDM 1.2 driver (KMDOD) for the internal panel: own double
# scanout buffers flipped on vsync, hardware cursor; the display cores and
# the bit-banged I2C come from the UEFI tree (exynosdisp2/cores.c).
# Versioned 2.0.0.N (it replaced the single-buffer exynosdisp 1.0.0.x).
EXTRA_CFLAGS="$EXTRA_CFLAGS -DDXGKDDI_INTERFACE_VERSION=0x300E" WDM=1 ./build-driver.sh exynosdisp2 \
  exynosdisp2/exynosdisp2.c exynosdisp2/cores.c exynosdisp2/fimdext.c -- "$LIB/displib.lib" "$LIB/ntstrsafe.lib"
# Full WDDM 1.2 adapter (display from exynosdisp2 + render side), and its
# user-mode driver (exynosumd.dll, plain DLL, no CRT).
# EXW_GPU_MODE=0: present-only (softpipe in the UMD), 1: the T628 renders (panfrost)
EXTRA_CFLAGS="$EXTRA_CFLAGS -DDXGKDDI_INTERFACE_VERSION=0x300E -DEXW_GPU_MODE=${EXW_GPU_MODE:-1}" WDM=1 ./build-driver.sh exynoswddm \
  exynoswddm/exynoswddm.c exynoswddm/render.c exynoswddm/cores.c exynoswddm/fimdext.c exynoswddm/ddtrace.c exynoswddm/oslogfile.c -- "$LIB/ntstrsafe.lib"
./exynosumd/build.sh

# Sensors on I2C3, one HID sensor minidriver each (common/hidsensor.h,
# common/spbreg.h) under mshidkmdf.
for s in bma255 bmg160 ak8963 cm3323; do
  ./build-driver.sh $s $s/$s.c -- "$LIB/ntstrsafe.lib"
done
# MAX77802 PMIC: sole owner of its I2C connection, regulator interface
# (common/ExPmicIf.h) for camera, GPU, key backlight.
./build-driver.sh exynospmic exynospmic/exynospmic.c -- "$LIB/ntstrsafe.lib"
# Camera ISP (FIMC-IS): power, A5 firmware, mailbox interrupt; the core comes
# from the UEFI tree (exynoscam/cores.c).
# AVStream capture minidriver (WDM entry): ks.lib + ksguid.lib
EXTRA_CFLAGS="$EXTRA_CFLAGS -DCDECL= -Wno-missing-braces" WDM=1 ./build-driver.sh exynoscam exynoscam/exynoscam.c exynoscam/cores.c -- "$LIB/ks.lib" "$LIB/ksguid.lib" "$LIB/ntstrsafe.lib"
# Battery class miniport; the gauge / charger code and the bit-banged I2C are
# built unchanged from the UEFI tree (exynosbatt/ExynosEnv.h).
./build-driver.sh exynosbatt exynosbatt/exynosbatt.c exynosbatt/batterycore.c exynosbatt/gpioi2c.c -- \
  "$LIB/battc.lib" "$LIB/ntstrsafe.lib"

# NDIS miniport (not KMDF): Wi-Fi.  The Broadcom core and the MMC host come
# from the UEFI tree (exynoswlan/MmcEnv.h).  ndis.h's NDIS_API_VERSION_AVAILABLE
# pastes 0x onto macro arguments without expanding them (MSVC preprocessor
# behaviour); a patched copy with the usual two-level macro goes into
# exynoswlan/build/ndisfix (generated, not in git).
NDISFIX=exynoswlan/build/ndisfix
mkdir -p "$NDISFIX"
sed -e 's/^#define NDIS_API_VERSION_AVAILABLE(major,minor) \(.*\)$/#define NDIS_API_VERSION_AVAILABLE_(major,minor) \1\n#define NDIS_API_VERSION_AVAILABLE(major,minor) NDIS_API_VERSION_AVAILABLE_(major,minor)/' \
  "${WDK:-$HOME/s2/winsdk/wdk10}/Include/10.0.19041.0/km/ndis.h" > "$NDISFIX/ndis.h"
EXTRA_CFLAGS="$EXTRA_CFLAGS -I$NDISFIX -Wno-extra-tokens" WDM=1 ./build-driver.sh exynoswlan exynoswlan/exynoswlan.c exynoswlan/bcmsdio.c exynoswlan/bcmwpa.c exynoswlan/dwmci.c -- \
  "$LIB/ndis.lib" "$LIB/ntstrsafe.lib"

# The same chip as a Native 802.11 (ExtSTA) miniport: Windows' WLAN service
# manages it.  Same Broadcom core and ndis.h fix (exynoswifi/MmcEnv.h).
EXTRA_CFLAGS="$EXTRA_CFLAGS -I$NDISFIX -Wno-extra-tokens" WDM=1 ./build-driver.sh exynoswifi exynoswifi/main.c exynoswifi/hw.c exynoswifi/oids.c \
  exynoswifi/bcmsdio.c exynoswifi/bcmwpa.c exynoswifi/dwmci.c -- "$LIB/ndis.lib" "$LIB/ntstrsafe.lib"

# PortCls WaveRT miniport (WDM, C++): speakers.  The codec / I2S cores come
# from the UEFI tree (Drivers/Audio), the board through exynosspi / exynosgpio.
WDM=1 ./build-driver.sh exynosaudio exynosaudio/adapter.cpp exynosaudio/wave.cpp exynosaudio/topo.cpp \
  exynosaudio/guids.cpp exynosaudio/cores.c -- "$LIB/portcls.lib" "$LIB/stdunk.lib" "$LIB/ntstrsafe.lib"

# Storport miniport (not KMDF): the eMMC, MMC code built unchanged from the
# UEFI tree (../uefi/Exynos5420Pkg/Drivers/SdCardDxe) against exynosemmc/MmcEnv.h.
WDM=1 ./build-driver.sh exynosemmc exynosemmc/exynosemmc.c exynosemmc/mmccore.c exynosemmc/dwmci.c -- \
  "$LIB/storport.lib" "$LIB/ntstrsafe.lib"

SIGN=${SIGN:-$HOME/s2/winsdk/testsign/testsign}

# Every build gets a new DriverVer (1.0.0.<build>, today's date) in the
# Artifacts INFs, so Device Manager shows which test build is installed.
BUILD=$(( $(cat .buildnum 2>/dev/null || echo 0) + 1 ))
echo "$BUILD" > .buildnum
DATE=$(date +%m/%d/%Y)

for d in exynosdwc3 exynosi2c exynosgpio exynosspi exynosaudio mxt1664s wacomg9 exynosbtn exynosbatt exynoswlan exynoswifi exynosemmc bma255 bmg160 ak8963 cm3323 exynosdisp2 exynoswddm exynosuart exynosbt exynosgpu exynoscam exynospmic; do
  mkdir -p "Artifacts/$d"
  cp "$d/build/$d.sys" "Artifacts/$d/"
  # Bluetooth patch files from the stock image (not in the tree).
  if [ "$d" = exynosbt ]; then
    cp "$FIRMWARE"/bcm4350_V0301.0591.hcd "$FIRMWARE"/bcm4335_V0105.0541.hcd "Artifacts/$d/" 2>/dev/null ||
      echo "exynosbt: patch files missing in $FIRMWARE"
  fi
  # FIMC-IS A5 firmware from the stock image (not in the tree).
  if [ "$d" = exynoscam ]; then
    cp "${CAM_FIRMWARE:-$HOME/exynos5/camera-test}"/fimc_is_fw2_IMX134.bin "${CAM_FIRMWARE:-$HOME/exynos5/camera-test}"/setfile_imx134.bin "Artifacts/$d/" ||
      echo "exynoscam: firmware / setfile missing"
  fi
  # exynoswddm's user-mode driver: Mesa d3d10umd + softpipe / panfrost (ARM32 mingw build, see the
  # mesa-rt notes), test-signed; the exynosumd/ stub (probe) is not shipped.
  if [ "$d" = exynoswddm ]; then
    MESA_UMD=${MESA_UMD:-$HOME/exynos5/ext/mesa-rt/build-rt/src/gallium/targets/d3d10umd/exynosumd.dll}
    rm -f "Artifacts/$d/exynosumd.dll"
    osslsigncode sign -certs "$SIGN.crt" -key "$SIGN.key" -h sha256 -in "$MESA_UMD" -out "Artifacts/$d/exynosumd.dll" > /dev/null
  fi
  MAJOR=1
  [ "$d" = exynosdisp2 ] && MAJOR=2
  sed -e "s|^DriverVer *=.*|DriverVer   = $DATE,$MAJOR.0.0.$BUILD\nCatalogFile = $d.cat|" -e "/^CatalogFile/d" \
    "$d/$d.inf" > "Artifacts/$d/$d.inf"
  # Signed catalog over the INF and the driver (tools/mkcat.py), so pnputil /
  # Device Manager accept the package once the test certificate is trusted.
  if [ -f "$SIGN.key" ]; then
    rm -f "Artifacts/$d/$d.cat"
    python3 -I tools/mkcat.py "Artifacts/$d/$d.unsigned.cat" $(ls "Artifacts/$d"/* | grep -v '\.cat$') > /dev/null
    osslsigncode sign -certs "$SIGN.crt" -key "$SIGN.key" -h sha256 \
      -in "Artifacts/$d/$d.unsigned.cat" -out "Artifacts/$d/$d.cat" > /dev/null
    rm -f "Artifacts/$d/$d.unsigned.cat"
  fi
done
echo "DriverVer $DATE,1.0.0.$BUILD"
echo "Artifacts/: $(ls Artifacts | tr '\n' ' ')"
