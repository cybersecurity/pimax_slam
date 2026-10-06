#!/bin/bash
# Run the deterministic test copies of the original and the rebuilt DLL on one trace and diff their
# info-level logs (timing lines removed) and pose outputs.
#   test/ab_log_diff.sh <trace.bin> [workdir]
# Needs: build-win-det configured with -DPIMAX_SLAM_TEST_DETERMINISTIC=ON -DPIMAX_SLAM_TEST_LOG_LEVEL=2 and built;
#        RUNTIME (Pimax runtime dir).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); ROOT=$(dirname "$HERE")
TRACE=$1; W=${2:-$ROOT/build-test/ab}
mkdir -p "$W"
ORIG=${ORIG_DLL:-$RUNTIME/pimax_slam.pi.dll}
# LOGLEVEL=1 (debug lines too) needs a rebuilt DLL configured with -DPIMAX_SLAM_TEST_LOG_LEVEL=1: NEW_DLL=...
LV=${LOGLEVEL:-2}
NEW=${NEW_DLL:-$ROOT/build-win-det/pimax_slam.pi.dll}
[ -f "$W/orig_det_l$LV.dll" ] && [ "$W/orig_det_l$LV.dll" -nt "$HERE/make_deterministic_orig.py" ] ||
    python3 "$HERE/make_deterministic_orig.py" --log-level $LV "$ORIG" "$W/orig_det_l$LV.dll" > /dev/null
PS_CV_THREADS=1 "$HERE/run_one.sh" "$W/orig_det_l$LV.dll" "$TRACE" "$W/orig" ${LOC:-0} > /dev/null &
PS_CV_THREADS=1 "$HERE/run_one.sh" "$NEW" "$TRACE" "$W/new" ${LOC:-0} > /dev/null || true
wait || true
norm() { sed 's/^[^[]*\[//; s/[0-9]\{4\}_[0-9_]*\[[a-z]*\]/<ts>/g; s/\\orig\\out/\\X/; s/\\new\\out/\\X/' "$1" | grep -v 'image process time\|total time'; }
norm "$W"/orig/out/6DOF_*.txt > "$W/orig.log"; norm "$W"/new/out/6DOF_*.txt > "$W/new.log"
if diff -q "$W/orig.log" "$W/new.log" > /dev/null; then echo "logs: identical ($(wc -l < "$W/orig.log") lines)";
else echo "logs differ (orig $(wc -l < "$W/orig.log"), new $(wc -l < "$W/new.log") lines); first difference:";
     diff "$W/orig.log" "$W/new.log" | head -${DIFF_LINES:-20}; fi
so() { grep -v 'fixme\|^frames ' "$1"; }   # stdout/stderr of the replayer (Ceres miniglog etc.)
if diff -q <(so "$W/orig/stdout.txt") <(so "$W/new/stdout.txt") > /dev/null; then echo "stdout/stderr: identical ($(so "$W/orig/stdout.txt" | wc -l) lines)";
else echo "stdout/stderr differ (orig $(so "$W/orig/stdout.txt" | wc -l), new $(so "$W/new/stdout.txt" | wc -l) lines)"; fi
grep -q 'Unhandled\|Check failed\|wine: ' "$W/new/stdout.txt" && { echo "rebuilt DLL stdout:"; grep -v '^ ' "$W/new/stdout.txt" | grep -v fixme | head -15; }
python3 "$HERE/compare.py" "$W/orig/poses.bin" "$W/new/poses.bin" || true
