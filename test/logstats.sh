#!/bin/bash
# Summary counters of a run's 6DOF log: test/logstats.sh <workdir>...
for d in "$@"; do
    f=$(echo "$d"/out/6DOF_*.txt)
    printf '%-28s init-ok %2d  not-good %2d  attempts %3d  resetLC %d  lost/reset %3d  lines %d\n' "$d" \
        "$(grep -c 'imu_initial true' $f)" "$(grep -c 'not good' $f)" "$(grep -c 'ImuInitial||frame' $f)" \
        "$(grep -c resetLoopClosing $f)" "$(grep -c 'Reset b_m_lost' $f)" "$(wc -l < $f)"
done
