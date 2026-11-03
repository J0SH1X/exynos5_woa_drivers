#!/bin/sh
# exynospep (prepared, not part of ../build.sh): builds the PEP and puts
# exynospep.inf (fresh DriverVer) + .sys + signed .cat into exynospep/out.
set -e
cd "$(dirname "$0")/.."
LIB=${LIB:-$HOME/s2/winsdk/wdk10/Lib/10.0.19041.0/km/arm}
SIGN=${SIGN:-$HOME/s2/winsdk/testsign/testsign}
WDM=1 ./build-driver.sh exynospep exynospep/exynospep.c -- "$LIB/ntstrsafe.lib"
OUT=exynospep/out
mkdir -p "$OUT"
BUILD=$(( $(cat exynospep/.buildnum 2>/dev/null || echo 0) + 1 ))
echo "$BUILD" > exynospep/.buildnum
cp exynospep/build/exynospep.sys "$OUT/"
sed -e "s|^DriverVer *=.*|DriverVer   = $(date +%m/%d/%Y),1.0.0.$BUILD\nCatalogFile = exynospep.cat|" \
  exynospep/exynospep.inf > "$OUT/exynospep.inf"
rm -f "$OUT/exynospep.cat"
python3 -I tools/mkcat.py "$OUT/unsigned.cat" "$OUT/exynospep.inf" "$OUT/exynospep.sys" > /dev/null
osslsigncode sign -certs "$SIGN.crt" -key "$SIGN.key" -h sha256 -in "$OUT/unsigned.cat" -out "$OUT/exynospep.cat" > /dev/null
rm -f "$OUT/unsigned.cat"
grep DriverVer "$OUT/exynospep.inf"
