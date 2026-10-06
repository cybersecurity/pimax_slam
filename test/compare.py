#!/usr/bin/env python3
"""Compare two replay outputs (poses.bin) record by record; optional ground truth (gen_scenario --truth)."""
import argparse, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import poses

ap = argparse.ArgumentParser()
ap.add_argument("a"); ap.add_argument("b"); ap.add_argument("--truth", default="")
a = ap.parse_args()
A, FA = poses.load(a.a); B, FB = poses.load(a.b)
n = min(len(A), len(B))
print("records: orig %d, new %d; frames: orig %d, new %d" % (len(A), len(B), len(FA), len(FB)))
same_ret = (A["ret"][:n] == B["ret"][:n])
same_state = (A["state"][:n] == B["state"][:n]) & (A["six"][:n] == B["six"][:n])
# all meaningful pose fields (padding bytes excluded, see poses.py)
FIELDS = [f for f in A.dtype.names if f not in ("ts", "ret") and not f.startswith("pad")]
exact = np.ones(n, bool)
for f in FIELDS:
    a, b = A[f][:n], B[f][:n]
    a = np.ascontiguousarray(a).view(np.uint8).reshape(n, -1)
    b = np.ascontiguousarray(b).view(np.uint8).reshape(n, -1)
    exact &= (a == b).all(axis=1)
dp = np.abs(A["p"][:n] - B["p"][:n]).max(axis=1)
first = int(np.argmax(~exact)) if (~exact).any() else n
print("bit-exact poses: %d / %d (first difference at record %d, t=%.3f s)" % (exact.sum(), n, first,
      A["ts"][min(first, n - 1)] * 1e-9))
print("return-code mismatches: %d, state mismatches: %d, max |dp| %.6g m, mean |dp| %.6g m" % (
    (~same_ret).sum(), (~same_state).sum(), dp.max(), dp.mean()))
m = min(len(FA), len(FB))
if m:
    FF = [f for f in FA.dtype.names if f != "gpad"]  # padding of HeadsetGroundState is uninitialised
    fexact = np.array([all(np.array_equal(FA[i][f], FB[i][f]) for f in FF) for i in range(m)])
    ffirst = int(np.argmax(~fexact)) if (~fexact).any() else m
    print("frame records (put-image ret, queue, ground state, loc state) identical: %d / %d (first difference at frame %d)"
          % (fexact.sum(), m, ffirst))
