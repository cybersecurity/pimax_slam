#!/bin/bash
# Replay one trace through one DLL under wine.
#   test/run_one.sh <dll> <trace.bin> <workdir> [loc_mode]
# Env: RUNTIME (Pimax runtime dir with the OpenCV DLLs), CALIB (default test/data/device_calibration.xml),
#      VOC (default ../slam/voc_GEN_8X4.dbow).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(dirname "$HERE")
DLL=$(realpath "$1"); TRACE=$(realpath "$2"); WORK=$(realpath -m "$3"); LOC=${4:-0}
RUNTIME=${RUNTIME:?set RUNTIME to the Pimax runtime directory (opencv_*453.dll)}
CALIB=${CALIB:-$HERE/data/device_calibration.xml}
if [ ! -f "$CALIB" ]; then
    # The shipped calibration has deviceUID="device_not_configured" ("unknown device"); give it a Crystal Light
    # serial so that the device-specific setup runs as on real hardware.
    mkdir -p "$(dirname "$CALIB")"
    sed 's/deviceUID="device_not_configured"/deviceUID="P90SYNTHETIC0001"/' \
        "${SLAM:-$(dirname "$ROOT")/slam}/device_calibration.xml" > "$CALIB"
fi
VOC=${VOC:-$(dirname "$ROOT")/slam/voc_GEN_8X4.dbow}
winpath() { printf '%s' "Z:$(realpath -m "$1" | tr / '\\')"; }
rm -rf "$WORK/out"; mkdir -p "$WORK/out"   # the DLL reloads pimax_prior_position.txt and the PlatMap from here
# PRESEED=<dir>: copy its files (e.g. pimax_database.bin, pimax_prior_position.txt of an earlier run) into out/
[ -n "${PRESEED:-}" ] && cp -r "$PRESEED"/. "$WORK/out/"
REPLAY=$ROOT/build-test/replay.exe
if [ ! -f "$REPLAY" ] || [ "$REPLAY" -ot "$HERE/replay.c" ]; then
    mkdir -p "$ROOT/build-test"
    x86_64-w64-mingw32-gcc -O2 -Wall "$HERE/replay.c" -o "$REPLAY"
fi
# Each DLL gets its own directory so that wine never picks up the wrong copy.
mkdir -p "$WORK/bin" && cp "$DLL" "$WORK/bin/pimax_slam.pi.dll"
WINEPATH="$(winpath "$RUNTIME")" WINEDEBUG=-all timeout 3600 wine "$REPLAY" "$(winpath "$WORK/bin/pimax_slam.pi.dll")" \
    "$(winpath "$CALIB")" "$(winpath "$WORK/out")" "$(winpath "$VOC")" "$LOC" "$(winpath "$TRACE")" \
    "$(winpath "$WORK/poses.bin")" > "$WORK/stdout.txt" 2>&1
cat "$WORK/stdout.txt" | tail -3
