#!/usr/bin/env python3
"""Compare the __LINE__ of every miniglog log site (MessageLogger ctor 0x1801B8B90) in the original binary's
Ceres with third_party/ceres-solver (+ the third_party/ceres-pimax overlay). A site whose line in our source is not a LOG/CHECK statement points at a
Pimax change in that file (see third_party/ceres-pimax/PIMAX_PATCH.md)."""
import json, os, re, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
names = {n: ea for ea, n, _ in json.load(open(os.path.join(ROOT, "re_dump/names.json")))}
strs = {}
for l in open(os.path.join(ROOT, "re_dump/strings.txt")):
    a, s = l.split(" ", 1)
    strs[int(a, 16)] = eval(s)
asm = open(os.path.join(ROOT, "re_dump/all.asm")).read().split("\n")
sites = {}
for i, l in enumerate(asm):
    if "call    sub_1801B8B90" not in l:
        continue
    fname = line = None
    for j in range(i - 1, max(0, i - 25), -1):
        m = re.search(r"lea\s+rdx, (\w+)", asm[j])
        if m and fname is None and m.group(1) in names:
            s = strs.get(names[m.group(1)], "")
            if "thirdparty\\ceres" in s:
                fname = s.split("thirdparty\\ceres\\")[1].replace("\\", "/")
        m = re.search(r"mov\s+r8d, ([0-9A-F]+)(h?)(?:\s|;|$)", asm[j])
        if m and line is None:
            line = int(m.group(1), 16) if m.group(2) else int(m.group(1))
    if fname and line:
        sites.setdefault((fname, line), asm[i].split()[0])
bad = 0
for (f, line), ea in sorted(sites.items()):
    p = os.path.join(ROOT, "third_party/ceres-pimax", f)
    if not os.path.exists(p):
        p = os.path.join(ROOT, "third_party/ceres-solver", f)
    if not os.path.exists(p):
        print("missing", f); continue
    src = open(p).read().split("\n")
    text = src[line - 1] if 0 < line <= len(src) else "<EOF>"
    ctx = "\n".join(src[max(0, line - 5):line])   # multi-line statements report their last line? (MSVC: first)
    if not re.search(r"\bLOG|CHECK|VLOG", text) and not re.search(r"\bLOG|CHECK|VLOG", "\n".join(src[line - 1:line + 3])):
        bad += 1
        print("%-50s %4d @%s: %s" % (f, line, ea, text.strip()[:90]))
print("checked %d sites, %d mismatches" % (len(sites), bad))
