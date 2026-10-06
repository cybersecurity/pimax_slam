#!/usr/bin/env python3
"""Line-by-line numeric diff of two probe outputs: prints lines whose numbers differ, with max rel diff."""
import re, sys
num = re.compile(r"[-+]?(?:\d+\.?\d*(?:[eE][-+]?\d+)?|nan|inf)")
A = open(sys.argv[1]).read().splitlines(); B = open(sys.argv[2]).read().splitlines()
lim = int(sys.argv[3]) if len(sys.argv) > 3 else 40
shown = 0
for i, (a, b) in enumerate(zip(A, B)):
    if a == b: continue
    na = [float(x) for x in num.findall(a)]; nb = [float(x) for x in num.findall(b)]
    if len(na) != len(nb) or num.sub("#", a) != num.sub("#", b):
        print("%d STRUCT\n  %s\n  %s" % (i + 1, a[:300], b[:300])); shown += 1
    else:
        r = max((abs(x - y) / max(abs(x), abs(y), 1e-300) for x, y in zip(na, nb) if x != y), default=0)
        k = [j for j, (x, y) in enumerate(zip(na, nb)) if x != y]
        print("%d maxrel %.3g (%d/%d values differ, first #%d) %s" % (i + 1, r, len(k), len(na), k[0], a[:80])); shown += 1
    if shown >= lim: break
if len(A) != len(B): print("lengths differ: %d vs %d" % (len(A), len(B)))
