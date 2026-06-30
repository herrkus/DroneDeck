#!/usr/bin/env bash
# Build the native DroneDeck core (C++ + x86-64 asm, or portable C elsewhere)
# and run the self-test. Relative paths only, so the space in the parent folder
# name never bites us.
set -euo pipefail
cd "$(dirname "$0")"

CXX="${CXX:-g++}"
CXXFLAGS="${CXXFLAGS:--O3 -Wall -Wextra}"
mkdir -p build

ARCH="$(uname -m)"
if [ "$ARCH" = "x86_64" ] || [ "$ARCH" = "amd64" ]; then
    CRC_SRC="core/crc_x25.S"
    CRC_NOTE="x86-64 assembly (slicing-by-8)"
    CXXFLAGS="$CXXFLAGS -march=native"
else
    CRC_SRC="core/crc_portable.c"
    CRC_NOTE="portable C (slicing-by-8) -- no x86-64 asm on $ARCH"
fi

echo "[1/3] building core/libdronecore.so  [CRC: $CRC_NOTE]"
$CXX $CXXFLAGS -fPIC -shared \
    "$CRC_SRC" core/dronecore.cpp \
    -o core/libdronecore.so

echo "[2/3] building build/selftest"
$CXX $CXXFLAGS \
    "$CRC_SRC" core/dronecore.cpp tests/selftest.cpp \
    -o build/selftest

echo "[3/3] running self-test"
./build/selftest

echo
echo "core ready: core/libdronecore.so  ($CRC_NOTE)"
