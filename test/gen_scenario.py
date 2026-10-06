#!/usr/bin/env python3
"""Synthetic headset-SLAM scenario for pimax_slam.pi.dll differential tests.

Renders a textured room into the four fisheye tracking cameras described by
slam/device_calibration.xml along a known HMD trajectory, and synthesizes the matching HMD IMU
stream. Writes a binary trace (see TRACE FORMAT) that test/replay.c feeds to either the original
or the rebuilt DLL through the Headset* C API.

World frame: gravity-aligned, +Z up, room = axis-aligned box. The DLL estimates gravity itself, so
the world frame only matters for the ground truth.
Camera i: T_imu_ci = T_imu_c0 * T_rig_i^-1, T_imu_c0 = [Rodrigues(ombc) | tbc] (calibration XML).

TRACE FORMAT (little endian):
  magic "PSLT", u32 version=1, u32 width, u32 height
  records: u8 type
    'I': u64 tsNs, f32 acc[3], f32 gyr[3], u64 extra          (HeadsetPutHmdImuData)
    'F': u64 tsNs, u32 shutter_ns, u32 gain, 4 * (width*height u8)   (HeadsetPutCameraImage)
    'M': i32 mode                                              (HeadsetSetTrackingMode)
    'E': u8 enable                                             (HeadsetSetDeviceEnable)
    'W': u32 milliseconds                                      (replayer sleeps)
"""
import argparse
import math
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation as Rot

G = 9.80665


def rodrigues(v):
    return Rot.from_rotvec(np.asarray(v, float)).as_matrix()


def load_cameras(path):
    root = ET.parse(path).getroot()
    si = root.find("SFConfig/Stateinit")
    T_i_c0 = np.eye(4)
    T_i_c0[:3, :3] = rodrigues([float(x) for x in si.get("ombc").split()])
    T_i_c0[:3, 3] = [float(x) for x in si.get("tbc").split()]
    cams = []
    for c in root.findall("Camera"):
        cal = c.find("Calibration")
        rig = c.find("Rig")
        T_rig = np.eye(4)
        T_rig[:3, :3] = np.array([float(x) for x in rig.get("rowMajorRotationMat").split()]).reshape(3, 3)
        T_rig[:3, 3] = [float(x) for x in rig.get("translation").split()]
        T_i_c = T_i_c0 @ np.linalg.inv(T_rig)
        fx, fy = [float(x) for x in cal.get("focal_length").split()]
        cx, cy = [float(x) for x in cal.get("principal_point").split()]
        k = [float(x) for x in cal.get("radial_distortion").split()][:4]
        cams.append({"id": int(c.get("id")), "T_i_c": T_i_c, "f": (fx, fy), "c": (cx, cy), "k": k,
                     "limit": float(cal.get("undistortion_limit"))})
    return cams


def unproject_rays(cam, W, H):
    """Unit bearing vectors (H, W, 3) in the camera frame for the KB4 fisheye model."""
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float64)
    mx = (xx - cam["c"][0]) / cam["f"][0]
    my = (yy - cam["c"][1]) / cam["f"][1]
    thd = np.hypot(mx, my)
    th = thd.copy()
    k1, k2, k3, k4 = cam["k"]
    for _ in range(20):
        t2 = th * th
        f = th * (1 + k1 * t2 + k2 * t2 ** 2 + k3 * t2 ** 3 + k4 * t2 ** 4) - thd
        df = 1 + 3 * k1 * t2 + 5 * k2 * t2 ** 2 + 7 * k3 * t2 ** 3 + 9 * k4 * t2 ** 4
        th = th - f / df
    valid = th < cam["limit"]
    th = np.clip(th, 0, cam["limit"])
    s = np.where(thd > 1e-12, np.sin(th) / np.maximum(thd, 1e-12), 1.0)
    rays = np.stack([mx * s, my * s, np.cos(th)], axis=-1)
    return rays / np.linalg.norm(rays, axis=-1, keepdims=True), valid


def make_texture(rng, n=1024):
    """High-contrast texture with plenty of corners and edges (FAST/edgelet friendly)."""
    tex = np.full((n, n), 110.0)
    for _ in range(900):
        w, h = rng.integers(8, 90, 2)
        x, y = rng.integers(0, n - 1, 2)
        tex[y:y + h, x:x + w] = rng.uniform(15, 240)
    for _ in range(250):
        r = rng.integers(4, 30)
        x, y = rng.integers(r, n - r, 2)
        yy, xx = np.ogrid[-r:r, -r:r]
        m = xx * xx + yy * yy <= r * r
        tex[y - r:y + r, x - r:x + r][m] = rng.uniform(15, 240)
    # mild blur so the image is band limited (sub-pixel alignment needs gradients)
    k = np.array([1, 4, 6, 4, 1], float) / 16
    tex = np.apply_along_axis(lambda v: np.convolve(v, k, "same"), 0, tex)
    tex = np.apply_along_axis(lambda v: np.convolve(v, k, "same"), 1, tex)
    return tex


class Room:
    """Box [-X, X] x [-Y, Y] x [0, Z]; one texture per face, 1 texel = scale metres."""

    def __init__(self, rng, half=(3.0, 3.0), height=2.8, scale=0.005):
        self.lo = np.array([-half[0], -half[1], 0.0])
        self.hi = np.array([half[0], half[1], height])
        self.tex = [make_texture(rng) for _ in range(6)]
        self.scale = scale

    def shade(self, o, d):
        """o: (3,), d: (N, 3) unit world rays -> (N,) intensities."""
        tmax = np.full(d.shape[0], np.inf)
        face = np.zeros(d.shape[0], int)
        for ax in range(3):
            with np.errstate(divide="ignore", invalid="ignore"):
                for side, bound in ((0, self.lo[ax]), (1, self.hi[ax])):
                    t = (bound - o[ax]) / d[:, ax]
                    ok = (t > 0) & (t < tmax)
                    tmax = np.where(ok, t, tmax)
                    face = np.where(ok, ax * 2 + side, face)
        p = o + d * tmax[:, None]
        out = np.empty(d.shape[0])
        for fi in range(6):
            m = face == fi
            if not m.any():
                continue
            ax = fi // 2
            u_ax, v_ax = [a for a in range(3) if a != ax]
            u = (p[m, u_ax] - self.lo[u_ax]) / self.scale
            v = (p[m, v_ax] - self.lo[v_ax]) / self.scale
            tex = self.tex[fi]
            n = tex.shape[0]
            u0 = np.floor(u).astype(int); v0 = np.floor(v).astype(int)
            fu = u - u0; fv = v - v0
            u0 %= n; v0 %= n; u1 = (u0 + 1) % n; v1 = (v0 + 1) % n
            out[m] = ((1 - fu) * (1 - fv) * tex[v0, u0] + fu * (1 - fv) * tex[v0, u1] +
                      (1 - fu) * fv * tex[v1, u0] + fu * fv * tex[v1, u1])
        return out


class Trajectory:
    """Smooth HMD IMU pose in the world (sum of sinusoids), standing person looking around."""

    def __init__(self, speed=1.0, still=False, still_start=0.0):
        self.s = 0.0 if still else speed
        self.t_start = still_start
        self.p0 = np.array([0.0, 0.0, 1.65])
        # IMU axes: the DLL's gravity convention is estimated; start level, looking along +X.
        self.R0 = Rot.from_euler("ZYX", [0.0, 0.0, 0.0]).as_matrix()

    def _w(self, t):
        """motion time: 0 during the still start, then a smooth (C2) ramp-in over 1 s"""
        t = t - self.T0 - self.t_start
        if t <= 0:
            return 0.0
        if t >= 1.0:
            return t - 0.5
        return t ** 3 - 0.5 * t ** 4   # w(1)=0.5, w'(1)=1, w''(1)=0

    T0 = 0.0

    def pos(self, t):
        s = self.s
        t = self._w(t)
        return self.p0 + np.array([0.25 * math.sin(0.6 * s * t), 0.20 * math.sin(0.45 * s * t),
                                   0.05 * math.sin(0.9 * s * t)])

    def acc(self, t, h=1e-3):
        return (self.pos(t + h) - 2 * self.pos(t) + self.pos(t - h)) / (h * h)

    def rot(self, t):
        s = self.s
        t = self._w(t)
        rv = [0.10 * math.sin(0.7 * s * t), 0.15 * math.sin(0.5 * s * t), 0.6 * math.sin(0.35 * s * t)]
        return self.R0 @ rodrigues(rv)

    def gyro(self, t, h=1e-4):
        dR = self.rot(t - h).T @ self.rot(t + h)
        return Rot.from_matrix(dR).as_rotvec() / (2 * h)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--slam", default="../slam", help="dir with device_calibration.xml (Pimax runtime slam/)")
    ap.add_argument("--out", default="trace.bin")
    ap.add_argument("--truth", default="", help="ground truth (.npy): t, q(xyzw), p of the HMD IMU per frame")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--imu-hz", type=float, default=1000.0)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--still", action="store_true")
    ap.add_argument("--settle-ms", type=int, default=40, help="replayer sleep after each frame")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--noise", type=float, default=2.0, help="image noise std-dev (grey levels)")
    ap.add_argument("--gyro-noise", type=float, default=0.0)
    ap.add_argument("--acc-noise", type=float, default=0.0)
    ap.add_argument("--shutter-ns", type=int, default=2000000)
    ap.add_argument("--brightness", type=float, default=0.35, help="scale of the room texture (IR images are dark)")
    ap.add_argument("--still-start", type=float, default=1.0, help="seconds of no motion at the start")
    ap.add_argument("--preview", default="")
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    cams = load_cameras(args.slam + "/device_calibration.xml")
    W, H = 640, 480
    rv = [unproject_rays(c, W, H) for c in cams]
    rays = [r.reshape(-1, 3) for r, _ in rv]
    masks = [m for _, m in rv]
    room = Room(rng)
    traj = Trajectory(args.speed, args.still, args.still_start)
    # Pimax IMU convention is not needed for rendering; the DLL is given T_imu_cam via the XML.

    out = open(args.out, "wb")
    out.write(b"PSLT" + struct.pack("<III", 1, W, H))
    t0 = 10.0
    traj.T0 = t0
    imu_dt = 1.0 / args.imu_hz
    next_imu = t0 - 0.5
    truth = []
    nf = int(args.seconds * args.fps)
    for f in range(nf):
        tf = t0 + f / args.fps
        while next_imu <= tf + 0.005:
            t = next_imu
            R = traj.rot(t)
            acc = R.T @ (traj.acc(t) + np.array([0, 0, G]))
            gyr = traj.gyro(t)
            acc = acc + rng.normal(0, args.acc_noise, 3) if args.acc_noise else acc
            gyr = gyr + rng.normal(0, args.gyro_noise, 3) if args.gyro_noise else gyr
            out.write(b"I" + struct.pack("<Q6fQ", int(round(t * 1e9)), *acc, *gyr, 0))
            next_imu = t + imu_dt
        T_w_i = np.eye(4)
        T_w_i[:3, :3] = traj.rot(tf)
        T_w_i[:3, 3] = traj.pos(tf)
        truth.append([tf, *Rot.from_matrix(T_w_i[:3, :3]).as_quat(), *T_w_i[:3, 3]])
        imgs = []
        for ci, cam in enumerate(cams):
            T_w_c = T_w_i @ cam["T_i_c"]
            d = rays[ci] @ T_w_c[:3, :3].T
            img = room.shade(T_w_c[:3, 3], d).reshape(H, W) * masks[ci] * args.brightness
            if args.noise > 0:
                img = img + rng.normal(0, args.noise, img.shape)
            imgs.append(np.clip(img, 0, 255).astype(np.uint8))
        out.write(b"F" + struct.pack("<QII", int(round(tf * 1e9)), args.shutter_ns, 16))
        for im in imgs:
            out.write(im.tobytes())
        if args.settle_ms > 0:
            out.write(b"W" + struct.pack("<I", args.settle_ms))
        if f == 0 and args.preview:
            from PIL import Image
            Image.fromarray(np.vstack([np.hstack(imgs[:2]), np.hstack(imgs[2:])])).save(args.preview)
    out.close()
    if args.truth:
        np.save(args.truth, np.array(truth))
    print("wrote %d frames (%.1f s) to %s" % (nf, nf / args.fps, args.out))


if __name__ == "__main__":
    main()
