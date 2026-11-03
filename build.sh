#!/bin/bash
# build.sh <out dir> - builds SimpleI2CTouchpad.kext (x86_64) with the SDK's Kernel.framework headers
set -euo pipefail
OUT=${1:?usage: build.sh <out dir>}
HERE=$(cd "$(dirname "$0")" && pwd)
SDK=$(xcrun --sdk macosx --show-sdk-path)
KF=$SDK/System/Library/Frameworks/Kernel.framework/Headers
K="$OUT/SimpleI2CTouchpad.kext/Contents"; mkdir -p "$K/MacOS"
T=$(mktemp -d)
xcrun clang++ -arch x86_64 -mmacosx-version-min=${MIN_OS:-14.0} -mkernel -fapple-kext -nostdinc -isysroot "$SDK" -I"$KF" \
  -DKERNEL -DKERNEL_PRIVATE -DDRIVER_PRIVATE -DAPPLE -DNeXT -std=c++17 -fno-rtti -fno-exceptions -fno-builtin -fno-common \
  -O2 -g0 -Wall -Wno-unused-parameter -Wno-deprecated-declarations -Wno-inconsistent-missing-override \
  -c "$HERE/SimpleI2CTouchpad.cpp" -o "$T/SimpleI2CTouchpad.o"
xcrun clang -arch x86_64 -mmacosx-version-min=${MIN_OS:-14.0} -fapple-kext -nostdlib -Xlinker -kext \
  "$T/SimpleI2CTouchpad.o" -lkmodc++ -lkmod -lcc_kext -o "$K/MacOS/SimpleI2CTouchpad"
cp "$HERE/Info.plist" "$K/Info.plist"
echo "SimpleI2CTouchpad.kext: $(stat -f %z "$K/MacOS/SimpleI2CTouchpad") bytes"
