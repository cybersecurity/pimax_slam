#!/usr/bin/env python3
"""Rewrite a replay trace (see gen_scenario.py TRACE FORMAT) to exercise extra code paths.
   trace_edit.py in.bin out.bin [--blackout T0:T1 ...] [--freeze T0:T1 ...] [--drop-imu T0:T1 ...]
                 [--event T:M:<mode> | T:E:<0|1> ...] [--settle-ms N]
 --blackout: images of frames with T0 <= t < T1 are set to 0 (cameras covered -> tracking lost)
 --freeze:   frames with T0 <= t < T1 repeat the last image before T0 (vision says "static")
 --drop-imu: IMU records with T0 <= t < T1 are removed
 --event:    insert a HeadsetSetTrackingMode ('M') / HeadsetSetDeviceEnable ('E') record before the
             first record with timestamp >= T
 --settle-ms: rewrite every 'W' record (replayer sleep after each frame)"""
import argparse, struct

ap = argparse.ArgumentParser()
ap.add_argument("inp"); ap.add_argument("out")
ap.add_argument("--blackout", action="append", default=[])
ap.add_argument("--freeze", action="append", default=[])
ap.add_argument("--drop-imu", action="append", default=[])
ap.add_argument("--event", action="append", default=[])
ap.add_argument("--settle-ms", type=int, default=None)
a = ap.parse_args()
rng = lambda L: [tuple(float(x) for x in s.split(":")) for s in L]
black, freeze, dropi = rng(a.blackout), rng(a.freeze), rng(a.drop_imu)
events = sorted((float(t), k, int(v)) for t, k, v in (e.split(":") for e in a.event))
inside = lambda t, R: any(t0 <= t < t1 for t0, t1 in R)

f = open(a.inp, "rb"); o = open(a.out, "wb")
hdr = f.read(16); o.write(hdr)
W, H = struct.unpack_from("<II", hdr, 8)
img_bytes = 4 * W * H
last_imgs = None
def flush_events(t):
    while events and events[0][0] <= t:
        _, k, v = events.pop(0)
        o.write(b"M" + struct.pack("<i", v) if k == "M" else b"E" + struct.pack("<B", v))
while True:
    c = f.read(1)
    if not c: break
    if c == b"I":
        rec = f.read(40); t = struct.unpack_from("<Q", rec)[0] * 1e-9
        flush_events(t)
        if not inside(t, dropi): o.write(c + rec)
    elif c == b"F":
        h = f.read(16); imgs = f.read(img_bytes); t = struct.unpack_from("<Q", h)[0] * 1e-9
        flush_events(t)
        if inside(t, black): imgs = bytes(img_bytes)
        elif inside(t, freeze) and last_imgs is not None: imgs = last_imgs
        else: last_imgs = imgs
        o.write(c + h + imgs)
    elif c == b"W":
        ms = struct.unpack("<I", f.read(4))[0]
        o.write(c + struct.pack("<I", a.settle_ms if a.settle_ms is not None else ms))
    elif c == b"M": o.write(c + f.read(4))
    elif c == b"E": o.write(c + f.read(1))
    else: raise SystemExit("bad record %r" % c)
o.close()
