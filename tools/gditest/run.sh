#!/bin/sh
# Host test of exynoswddm/gdiops.c (the CPU GDI operations): ./run.sh [cases]
set -e
cd "$(dirname "$0")"
clang -O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=undefined \
  gditest.c ../../exynoswddm/gdiops.c -o /tmp/gditest
/tmp/gditest "${1:-20000}"
# and as the driver compiles it: 32-bit ARM, to catch anything size dependent
clang --target=thumbv7-pc-windows-msvc -O2 -fsyntax-only -Wall -Wextra ../../exynoswddm/gdiops.c \
  -isystem "$HOME/s2/winsdk/splat/crt/include" 2>&1 | head -20
