#!/bin/bash
# Final A/B table: for each trace run original + rebuilt (test/ab_log_diff.sh) and a second original run
# (run-to-run determinism horizon of the original), then print a summary line per trace.
#   RUNTIME=... test/ab_all.sh t4s t8 ...      (traces build-test/<name>.bin, truth build-test/<truth>.npy)
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); ROOT=$(dirname "$HERE"); B=$ROOT/build-test
for t in "$@"; do
    W=$B/abf_$t
    out=$("$HERE/ab_log_diff.sh" "$B/$t.bin" "$W" 2>&1)
    PS_CV_THREADS=1 "$HERE/run_one.sh" "$W/orig_det_l2.dll" "$B/$t.bin" "$W/orig2" > /dev/null 2>&1
    logs=$(echo "$out" | grep -m1 '^logs')
    so=$(echo "$out" | grep -m1 '^stdout')
    nb=$(python3 "$HERE/compare.py" "$W/orig/poses.bin" "$W/new/poses.bin")
    oo=$(python3 "$HERE/compare.py" "$W/orig/poses.bin" "$W/orig2/poses.bin")
    tr=$B/${t%[a-z]}_truth.npy; [ -f "$B/${t}_truth.npy" ] && tr=$B/${t}_truth.npy
    case $t in t20b|t20m) tr=$B/t20_truth.npy;; t4s) tr=$B/t4_truth.npy;; esac
    ate=$(python3 "$HERE/truth_err.py" "$tr" "$W/orig/poses.bin" "$W/new/poses.bin" "$W/orig2/poses.bin" 2>/dev/null | sed 's/.*ATE rmse \([0-9.]*\) m, max \([0-9.]*\) m/\1\/\2/' | tr '\n' ' ')
    echo "== $t"
    echo "   $logs | $so"
    echo "   orig vs new : $(echo "$nb" | sed -n 2p | sed 's/bit-exact poses: //') | $(echo "$nb" | sed -n 3p | sed 's/.*max/max/')"
    echo "   orig vs orig: $(echo "$oo" | sed -n 2p | sed 's/bit-exact poses: //') | $(echo "$oo" | sed -n 3p | sed 's/.*max/max/')"
    echo "   ATE rmse/max [m] orig new orig2: $ate"
done
