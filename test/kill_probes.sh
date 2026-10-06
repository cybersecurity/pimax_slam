#!/bin/bash
# Kill leftover gdb_probe.sh processes (winedbg, replay_dbg.exe, gdb) without matching the caller's shell.
for p in $(pgrep -x replay_dbg.exe) $(pgrep -f '^C:.windows.system32.winedbg.exe') $(pgrep -f '^timeout 3600 winedbg') $(pgrep -f '^start.exe /exec winedbg') $(pgrep -f '^gdb -batch'); do kill "$p" 2>/dev/null; done
true
