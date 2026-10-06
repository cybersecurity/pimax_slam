#!/usr/bin/env python3
"""Parse a replay.exe output file (poses.bin) into numpy arrays.

  poses.py <poses.bin>            summary (tracking states over time)
"""
import struct
import sys

import numpy as np

POSE_DT = np.dtype([("ts", "<u8"), ("ret", "<i4"), ("q", "<f4", 4), ("p", "<f4", 3), ("w", "<f4", 3),
                    ("v", "<f4", 3), ("aa", "<f4", 3), ("la", "<f4", 3), ("pad0", "u1", 4), ("pts", "<u8"), ("state", "u1"),
                    ("flag", "u1"), ("pad", "u1", 2), ("conf", "<f4"), ("six", "u1"), ("pad2", "u1", 3)])
# HeadsetPose is 104 bytes but only the 97 bytes up to is_6dof are meaningful; the padding bytes
# (pose +76..79, +90..91, +97..103) are copied from an uninitialised stack temporary in the DLL.
assert POSE_DT.itemsize == 12 + 100
FRAME_DT = np.dtype([("marker", "<u8"), ("ret", "<i4"), ("queued", "<u4"), ("gvalid", "u1"), ("gpad", "u1", 3), ("g0", "<f4", 3),
                     ("g1", "<f4", 3), ("gret", "<i4"), ("loc", "u1"), ("lret", "<i4")], align=False)


def load(path):
    d = open(path, "rb").read()
    assert d[:4] == b"PSLO"
    p = 8
    poses, frames = [], []
    while p < len(d):
        (ts,) = struct.unpack_from("<Q", d, p)
        if ts == 0xFFFFFFFFFFFFFFFF:
            frames.append(np.frombuffer(d, FRAME_DT, 1, p)[0]); p += FRAME_DT.itemsize
        else:
            raw = d[p:p + 12] + d[p + 12:p + 12 + 104]
            poses.append(np.frombuffer(raw, POSE_DT, 1)[0]); p += 116
    return np.array(poses, POSE_DT), np.array(frames, FRAME_DT)


if __name__ == "__main__":
    P, F = load(sys.argv[1])
    ok = P[P["ret"] == 1]
    print("imu records %d, poses %d, frames %d" % (len(P), len(ok), len(F)))
    prev = None
    for r in ok:
        key = (int(r["state"]), int(r["flag"]), int(r["six"]))
        if key != prev:
            print("  t=%.3f state=%d flag=%d six=%d conf=%.2f p=%s" % (r["ts"] * 1e-9, *key, r["conf"],
                                                                       np.round(r["p"], 3)))
            prev = key
    print("last p", ok[-1]["p"], "q", ok[-1]["q"])
