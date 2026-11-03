#!/bin/sh
# exynosumd.dll (ARM32, no CRT), test-signed, into build/.
set -e
cd "$(dirname "$0")"
SIGN=${SIGN:-$HOME/s2/winsdk/testsign/testsign}
mkdir -p build
llvm-dlltool -m arm -d kernel32.def -l build/kernel32.lib
clang --target=thumbv7-pc-windows-msvc -O2 -ffreestanding -fno-stack-protector -fno-builtin -Wall -c exynosumd.c -o build/exynosumd.obj
lld-link /DLL /SUBSYSTEM:WINDOWS,6.02 /MACHINE:ARM /ENTRY:_DllMainCRTStartup /NODEFAULTLIB /OUT:build/exynosumd.unsigned.dll build/exynosumd.obj build/kernel32.lib
rm -f build/exynosumd.dll
osslsigncode sign -certs "$SIGN.crt" -key "$SIGN.key" -h sha256 -in build/exynosumd.unsigned.dll -out build/exynosumd.dll > /dev/null
rm build/exynosumd.unsigned.dll
echo build/exynosumd.dll
