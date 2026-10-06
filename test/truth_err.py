#!/usr/bin/env python3
"""Absolute trajectory error of a replay output (poses.bin) against gen_scenario ground truth (.npy:
t, qx, qy, qz, qw, px, py, pz of the HMD IMU per frame). The output pose of the IMU record with the
frame's timestamp is used (6DoF records only); the DLL's world frame is aligned to the truth with a
rigid (Umeyama, no scale) fit over all matched samples.
   truth_err.py <truth.npy> <poses.bin> [<poses.bin> ...]"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import poses


def umeyama(src, dst):
    ms, md = src.mean(0), dst.mean(0)
    H = (src - ms).T @ (dst - md)
    U, S, Vt = np.linalg.svd(H)
    D = np.eye(3); D[2, 2] = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ D @ U.T
    return R, md - R @ ms


def ate(truth, path):
    P, _ = poses.load(path)
    ts = P["ts"].astype(np.int64)
    est, ref = [], []
    for row in truth:
        tns = int(round(row[0] * 1e9))
        i = np.searchsorted(ts, tns)
        if i < len(ts) and abs(int(ts[i]) - tns) < 500000 and P["ret"][i] == 1 and P["six"][i]:
            est.append(P["p"][i].astype(float)); ref.append(row[5:8])
    est, ref = np.array(est), np.array(ref)
    if len(est) < 3:
        return len(est), float("nan"), float("nan")
    R, t = umeyama(est, ref)
    e = np.linalg.norm((est @ R.T + t) - ref, axis=1)
    return len(est), float(np.sqrt((e ** 2).mean())), float(e.max())


truth = np.load(sys.argv[1])
for p in sys.argv[2:]:
    n, rmse, mx = ate(truth, p)
    print("%s: %d/%d frames with 6DoF pose, ATE rmse %.4f m, max %.4f m" % (p, n, len(truth), rmse, mx))
