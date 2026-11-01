#!/bin/sh
# Build the Exynos5420 (v1awifi) Windows RT 8.1 drivers into Artifacts/<name>/
# (<name>.sys + <name>.inf), ready for
#   dism /Image:<mount> /Add-Driver /Driver:Artifacts/<name>/<name>.inf /ForceUnsigned
#
# Same toolchain as ~/s2/ExynosWinDrivers (clang + lld-link, WDK 10.0.19041
# ARM, KMDF 1.13).  Drivers log to the OS log ring (common/oslog.h), streamed
# over USB by exynosdwc3.  LOAD_TEST=1 builds load tests that bugcheck as soon
# as Windows starts them:
#   exynosdwc3  0x55534233 "USB3"
set -e
cd "$(dirname "$0")"

LIB=${WDK:-$HOME/s2/winsdk/wdk10}/Lib/10.0.19041.0/km/arm
if [ -n "$LOAD_TEST" ]; then
  export EXTRA_CFLAGS="$EXTRA_CFLAGS -DLOAD_TEST=$LOAD_TEST"
fi

./build-driver.sh exynosdwc3 exynosdwc3/exynosdwc3.c -- "$LIB/ntstrsafe.lib"

for d in exynosdwc3; do
  mkdir -p "Artifacts/$d"
  cp "$d/build/$d.sys" "$d/$d.inf" "Artifacts/$d/"
done
echo "Artifacts/: $(ls Artifacts | tr '\n' ' ')"
