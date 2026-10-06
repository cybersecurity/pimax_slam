#!/usr/bin/env python3
"""Show which pose fields differ between two replay outputs: first differing record per field and max abs diff."""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import poses
A, _ = poses.load(sys.argv[1]); B, _ = poses.load(sys.argv[2])
n = min(len(A), len(B))
for f in A.dtype.names:
    if f.startswith("pad"): continue
    a = A[f][:n]; b = B[f][:n]
    d = (a != b) if a.ndim == 1 else np.any(a != b, axis=1)
    if d.any():
        i = int(np.argmax(d))
        md = np.abs(a.astype(float) - b.astype(float)).max()
        print("%-6s first diff at record %d (t=%.3f): %s vs %s; differing records %d, max |diff| %.6g" % (f, i, A["ts"][i] * 1e-9, a[i], b[i], d.sum(), md))
