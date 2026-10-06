#!/bin/bash
# Compile single translation units of pimax_slam with exactly the DLL's flags (clang-cl, MSVC x64 ABI).
#
#   tools/cc.sh src/common/frame.cpp [more.cpp ...]        syntax check (-fsyntax-only)
#   tools/cc.sh --header src/common/frame.h [...]           check that a header compiles on its own
#   CC_OBJ=1 tools/cc.sh src/x.cpp                          emit build-win/cc/<path>.obj instead
#
# Needs a configured build-win/ (see README) with the glog/gflags ExternalProjects built
# (cmake --build build-win --target glog_ext).
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
B=$ROOT/build-win
[ -f "$B/ps_includes.txt" ] || { echo "configure build-win first" >&2; exit 2; }
XWIN=${XWIN_DIR:-$HOME/.xwin}
FLAGS=(--target=x86_64-pc-windows-msvc -fms-compatibility-version=19.29
       "/imsvc$XWIN/crt/include" "/imsvc$XWIN/sdk/include/ucrt" "/imsvc$XWIN/sdk/include/um"
       "/imsvc$XWIN/sdk/include/shared" -Wno-unused-command-line-argument
       /DWIN32 /D_WINDOWS /EHsc /O2 /Ob2 -std:c++17 -MD /bigobj /utf-8 -Wno-everything
       -ferror-limit=50)
while read -r d; do [ -n "$d" ] && FLAGS+=("/D$d"); done < "$B/ps_defines.txt"
while read -r i; do [ -n "$i" ] && FLAGS+=("/I$i"); done < "$B/ps_includes.txt"

header=0
rc=0
for f in "$@"; do
    if [ "$f" = "--header" ]; then header=1; continue; fi
    src=$(realpath "$f")
    if [ $header = 1 ]; then
        tmp=$(mktemp --suffix=.cpp)
        printf '#include "%s"\n' "$src" > "$tmp"
        clang-cl "${FLAGS[@]}" -fsyntax-only "$tmp" || { rc=1; echo "FAILED: $f"; }
        rm -f "$tmp"
    elif [ "${CC_OBJ:-0}" = 1 ]; then
        rel=${src#$ROOT/}
        out=$B/cc/${rel%.*}.obj
        mkdir -p "$(dirname "$out")"
        clang-cl "${FLAGS[@]}" /c "$src" "/Fo$out" || { rc=1; echo "FAILED: $f"; }
    else
        clang-cl "${FLAGS[@]}" -fsyntax-only "$src" || { rc=1; echo "FAILED: $f"; }
    fi
done
exit $rc
