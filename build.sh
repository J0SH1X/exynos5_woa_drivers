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

for d in exynosdwc3 exynosi2c exynosgpio exynosspi exynosaudio mxt1664s wacomg9 exynosbtn exynosbatt exynoswlan exynosemmc; do
  mkdir -p "Artifacts/$d"
  cp "$d/build/$d.sys" "Artifacts/$d/"
  sed -e "s|^DriverVer *=.*|DriverVer   = $DATE,1.0.0.$BUILD\nCatalogFile = $d.cat|" -e "/^CatalogFile/d" \
    "$d/$d.inf" > "Artifacts/$d/$d.inf"
  # Signed catalog over the INF and the driver (tools/mkcat.py), so pnputil /
  # Device Manager accept the package once the test certificate is trusted.
  if [ -f "$SIGN.key" ]; then
    rm -f "Artifacts/$d/$d.cat"
    python3 -I tools/mkcat.py "Artifacts/$d/$d.unsigned.cat" "Artifacts/$d/$d.inf" "Artifacts/$d/$d.sys" > /dev/null
    osslsigncode sign -certs "$SIGN.crt" -key "$SIGN.key" -h sha256 \
      -in "Artifacts/$d/$d.unsigned.cat" -out "Artifacts/$d/$d.cat" > /dev/null
    rm -f "Artifacts/$d/$d.unsigned.cat"
  fi
done
echo "DriverVer $DATE,1.0.0.$BUILD"
echo "Artifacts/: $(ls Artifacts | tr '\n' ' ')"
