#!/bin/bash
# Run one DLL on one trace under winedbg's gdb stub and execute a gdb-python probe script.
#   test/gdb_probe.sh <dll> <trace.bin> <workdir> <probe.py>
# probe.py is exec'd inside gdb after the DLL is loaded; it gets `base` (the load address; image VAs
# are relocated with  va - 0x180000000 + base), helper functions from test/gdb_probe_lib.py, and must
# call probe(va, callback) for each breakpoint. Output: <workdir>/probe.txt
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); ROOT=$(dirname "$HERE")
DLL=$(realpath "$1"); TRACE=$(realpath "$2"); WORK=$(realpath -m "$3"); PROBE=$(realpath "$4")
RUNTIME=${RUNTIME:?set RUNTIME}
CALIB=${CALIB:-$HERE/data/device_calibration.xml}
if [ ! -f "$CALIB" ]; then   # same as run_one.sh: shipped calibration with a Crystal Light serial
    mkdir -p "$(dirname "$CALIB")"
    sed 's/deviceUID="device_not_configured"/deviceUID="P90SYNTHETIC0001"/' \
        "${SLAM:-$(dirname "$ROOT")/slam}/device_calibration.xml" > "$CALIB"
fi
VOC=${VOC:-$(dirname "$ROOT")/slam/voc_GEN_8X4.dbow}
winpath() { printf '%s' "Z:$(realpath -m "$1" | tr / '\\')"; }
REPLAY=$ROOT/build-test/replay_dbg.exe
if [ ! -f "$REPLAY" ] || [ "$REPLAY" -ot "$HERE/replay_dbg.c" ]; then
    (cd "$HERE" && x86_64-w64-mingw32-gcc -O2 -Wall replay_dbg.c -o "$REPLAY")
fi
rm -rf "$WORK/out"; mkdir -p "$WORK/out" "$WORK/bin"; cp "$DLL" "$WORK/bin/pimax_slam.pi.dll"
rm -f "$WORK/wdbg.txt"
PS_BREAK=1 PS_CV_THREADS=${PS_CV_THREADS:-1} WINEPATH="$(winpath "$RUNTIME")" WINEDEBUG=-all \
    timeout 3600 winedbg --gdb --no-start "$REPLAY" "$(winpath "$WORK/bin/pimax_slam.pi.dll")" \
    "$(winpath "$CALIB")" "$(winpath "$WORK/out")" "$(winpath "$VOC")" 0 "$(winpath "$TRACE")" \
    "$(winpath "$WORK/poses.bin")" > "$WORK/wdbg.txt" 2>&1 &
for i in $(seq 100); do grep -q 'target remote' "$WORK/wdbg.txt" 2>/dev/null && break; sleep 0.2; done
PORT=$(grep -o 'localhost:[0-9]*' "$WORK/wdbg.txt" | head -1)
cat > "$WORK/run.gdb" <<G
set pagination off
set confirm off
target remote $PORT
continue
python exec(open("$HERE/gdb_probe_lib.py").read()); base = int(gdb.parse_and_eval("\$rbx")); OUT = open("$WORK/probe.txt", "w"); exec(open("$PROBE").read())
continue
G
timeout 3600 gdb -batch -x "$WORK/run.gdb" > "$WORK/gdb.txt" 2>&1 || true
wait || true
tail -3 "$WORK/gdb.txt"
