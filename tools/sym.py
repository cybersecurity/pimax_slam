#!/usr/bin/env python3
"""Look up functions of the rebuilt DLL by (substring of) mangled public name in its PDB.
   python3 tools/sym.py <substring> [build-dir]   -> prints  VA  name"""
import re, subprocess, sys
import pefile
pat = sys.argv[1]; bd = sys.argv[2] if len(sys.argv) > 2 else "build-win-det"
pe = pefile.PE(bd + "/pimax_slam.pi.dll", fast_load=True)
secs = [s.VirtualAddress for s in pe.sections]
txt = subprocess.run(["llvm-pdbutil", "dump", "--publics", bd + "/pimax_slam.pi.pdb"], capture_output=True, text=True).stdout
name = None
for line in txt.splitlines():
    m = re.search(r"S_PUB32 \[size = \d+\] `(.*)`", line)
    if m: name = m.group(1); continue
    m = re.search(r"addr = (\d+):(\d+)", line)
    if m and name and pat in name:
        print("0x%x %s" % (pe.OPTIONAL_HEADER.ImageBase + secs[int(m.group(1)) - 1] + int(m.group(2)), name))
        name = None
