#!/bin/sh
# Builds umcitest-signed.exe, umcitest-unsigned.exe and umcidll.dll (test-signed) into out/.
set -e
cd "$(dirname "$0")"
SIGN=${SIGN:-$HOME/s2/winsdk/testsign/testsign}
mkdir -p out
llvm-dlltool -m arm -d kernel32.def -l out/kernel32.lib
llvm-dlltool -m arm -d user32.def -l out/user32.lib
CF="--target=thumbv7-pc-windows-msvc -O2 -ffreestanding -fno-stack-protector -fno-builtin"
clang $CF -c umcitest.c -o out/umcitest.obj
clang $CF -c umcidll.c -o out/umcidll.obj
lld-link /SUBSYSTEM:WINDOWS,6.02 /MACHINE:ARM /ENTRY:mainCRTStartup /NODEFAULTLIB /OUT:out/umcitest-unsigned.exe out/umcitest.obj out/kernel32.lib out/user32.lib
lld-link /DLL /SUBSYSTEM:WINDOWS,6.02 /MACHINE:ARM /ENTRY:_DllMainCRTStartup /NODEFAULTLIB /OUT:out/umcidll.unsigned.dll out/umcidll.obj
osslsigncode sign -certs "$SIGN.crt" -key "$SIGN.key" -h sha256 -in out/umcitest-unsigned.exe -out out/umcitest-signed.exe > /dev/null
osslsigncode sign -certs "$SIGN.crt" -key "$SIGN.key" -h sha256 -in out/umcidll.unsigned.dll -out out/umcidll.dll > /dev/null
ls -la out/*.exe out/umcidll.dll
