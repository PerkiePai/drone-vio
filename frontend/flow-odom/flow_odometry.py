#!/usr/bin/env python3
"""Altitude-scaled optical-flow odometry for the nadir drone camera.

Why this exists
---------------
Monocular MSCKF VIO (OpenVINS) diverges to km-scale on this dataset because the
drone flies near-constant-velocity cruise (accel std ~1.1 m/s^2): metric scale
is unobservable from the accelerometer.  A downward camera over (roughly flat)
ground does not need the accelerometer for scale -- the *height* gives it:

    v_ground = (translational_flow_normalized / dt) * Z,   Z ~= height_AGL

This is the PX4Flow / optical-flow + rangefinder approach used on real
GPS-denied drones.  Scale comes from height (observable via baro), so the
estimate cannot blow up the way the filter scale does.

Pipeline per consecutive camera-frame pair (t0 -> t1):
  1. Track features with pyramidal Lucas-Kanade.
  2. Normalize pixels with the pinhole intrinsics (distortion is zero here).
  3. De-rotate: predict where each ray goes under camera rotation alone
     (relative attitude), subtract it -> pure translational flow.
  4. Per-point ground depth Z from height + attitude (ray-ground intersection).
  5. Least-squares solve camera-frame translation t_cam from the standard
     translational-flow equations:  Z*du = -tx + x*tz ,  Z*dv = -ty + y*tz.
  6. Rotate t_cam into the world (ENU) frame and accumulate XY.  Z uses baro.

Attitude source: GT quaternions as a stand-in for a good AHRS (a real drone has
a solid IMU attitude estimate; the hard/unobservable part is metric XY scale,
which is exactly what flow+height fixes).  Height source: baro (subtracting the
takeoff sample).  Both are sensor-realistic; nothing here uses GT *position*.
"""
import argparse
import csv
import json
import os

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DEFAULT_DIR = "/home/innovation/pai/drone-vio/_in/isaac-sim-20260623"


def quat_xyzw_to_R(qx, qy, qz, qw):
    """Body->world rotation from an (x,y,z,w) quaternion."""
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def load_dataset(d):
    calib = json.load(open(os.path.join(d, "cam_calib.json")))
    fx = calib["intrinsics"]["fx"]
    fy = calib["intrinsics"]["fy"]
    cx = calib["intrinsics"]["cx"]
    cy = calib["intrinsics"]["cy"]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

    # camera->body rotation from extrinsic quaternion (w,x,y,z).
    qw, qx, qy, qz = calib["extrinsic_body_to_cam"]["quaternion_wxyz"]
    R_CtoI = quat_xyzw_to_R(qx, qy, qz, qw)
    # Convention fix (verified empirically against GT motion): the extrinsic is
    # defined w.r.t. the FRD IMU body, but the GT attitude quaternions in
    # poses.csv are for an FLU body in ENU world.  FLU and FRD differ by 180 deg
    # about x.  R_body_cam = R_CtoI @ Rx(180).  This makes the camera optical
    # axis point DOWN in world (as it must for a nadir cam) and the recovered
    # per-frame motion match GT direction & magnitude.
    R_CtoI = R_CtoI @ np.diag([1.0, -1.0, -1.0])  # body = R_CtoI @ cam

    frames = list(csv.DictReader(open(os.path.join(d, "frames.csv"))))
    poses = {int(r["frame"]): r for r in csv.DictReader(open(os.path.join(d, "poses.csv")))}
    baro = {int(r["frame"]): float(r["pressure_altitude_m"])
            for r in csv.DictReader(open(os.path.join(d, "baro.csv")))}
    baro0 = baro[min(baro)]  # takeoff pressure altitude

    img_dir = os.path.join(d, "images", "cam0")
    recs = []
    for fr in frames:
        fi = int(fr["frame"])
        if fi not in poses or fi not in baro:
            continue
        p = poses[fi]
        recs.append({
            "ts": int(fr["ts_ns"]),
            "img": os.path.join(img_dir, os.path.basename(fr["image_path"])),
            "R_wb": quat_xyzw_to_R(float(p["qx"]), float(p["qy"]), float(p["qz"]), float(p["qw"])),
            "h": baro[fi] - baro0,                 # height above takeoff (baro)
            "gt": np.array([float(p["x"]), float(p["y"]), float(p["z"])]),
        })
    return K, R_CtoI, recs


def compute_ahrs_attitude(d, recs, Kp=1.0, mag_gain=0.0, mag_noise_deg=0.0):
    """Per-frame body attitude from the IMU ALONE (deployment-realistic; no GT).
    Mahony complementary filter integrated at the IMU rate in the FRD body frame:
    the gyro integrates orientation, the accelerometer corrects roll/pitch toward
    gravity. Initialised from GT frame-0; yaw drifts with no compass, exactly as on
    hardware. If mag_gain>0, a MAGNETOMETER stand-in nudges yaw toward the GT heading
    (+ optional gaussian noise mag_noise_deg) at each frame — projecting what a real
    gyro+accel+compass suite would achieve. Returns R_wb (FLU->ENU) per rec ts."""
    from scipy.spatial.transform import Rotation as Rot
    rows = list(csv.DictReader(open(os.path.join(d, "imu.csv"))))
    ts = np.array([int(r["ts_ns"]) for r in rows]) / 1e9
    gyro = np.array([[float(r["wx"]), float(r["wy"]), float(r["wz"])] for r in rows])  # FRD rad/s
    acc = np.array([[float(r["ax"]), float(r["ay"]), float(r["az"])] for r in rows])   # FRD specific force
    flip = np.diag([1.0, -1.0, -1.0])      # FLU<->FRD (self-inverse)
    R = recs[0]["R_wb"] @ flip              # init FRD->ENU from GT frame 0
    g_up = np.array([0.0, 0.0, 9.81])      # specific-force-at-rest points UP in ENU
    rec_ts = [r["ts"] / 1e9 for r in recs]
    gt_yaw = [np.arctan2(r["R_wb"][1, 0], r["R_wb"][0, 0]) for r in recs]  # heading per rec
    rng = np.random.default_rng(0)
    out = [recs[0]["R_wb"]]                 # frame 0 = GT init
    ri = 1
    for k in range(1, len(rows)):
        dt = ts[k] - ts[k - 1]
        if dt <= 0 or dt > 0.1:
            dt = 0.004
        w = gyro[k].copy()
        a = acc[k]; an = np.linalg.norm(a)
        if an > 1e-3:                       # tilt correction from gravity direction
            v_meas = a / an
            v_pred = R.T @ g_up; v_pred /= np.linalg.norm(v_pred)
            w = w + Kp * np.cross(v_meas, v_pred)
        R = R @ Rot.from_rotvec(w * dt).as_matrix()
        while ri < len(recs) and ts[k] >= rec_ts[ri]:
            if mag_gain > 0.0:             # compass stand-in: pull yaw toward GT heading
                yaw = np.arctan2(R[1, 0], R[0, 0])
                meas = gt_yaw[ri] + np.radians(mag_noise_deg) * rng.standard_normal()
                dpsi = np.arctan2(np.sin(meas - yaw), np.cos(meas - yaw))
                R = Rot.from_rotvec([0, 0, mag_gain * dpsi]).as_matrix() @ R   # about world up
            out.append(R @ flip)           # back to FLU->ENU for load_dataset parity
            ri += 1
    while len(out) < len(recs):
        out.append(R @ flip)
    return out


def _triangulate_midpoint(C0, d0, C1, d1):
    """Closest 3D point to two world rays (origin C, unit dir d). None if degenerate."""
    r = C0 - C1
    a, bb, c = d0 @ d0, d0 @ d1, d1 @ d1
    M = np.array([[a, -bb], [bb, -c]])
    rhs = np.array([-(d0 @ r), -(d1 @ r)])
    if abs(np.linalg.det(M)) < 1e-9 or abs(bb) > 0.99995:   # near-parallel rays
        return None
    l0, l1 = np.linalg.solve(M, rhs)
    if l0 <= 0 or l1 <= 0:                                   # behind a camera
        return None
    return 0.5 * ((C0 + l0 * d0) + (C1 + l1 * d1))


def compute_true_agl(recs, K, R_CtoI, scale, W=10, step=4, min_pts=20):
    """Per-frame true AGL = camera_altitude - terrain_elevation, the honest sim
    stand-in for `baro - DEM(lat,lon)`.  Terrain elevation is reconstructed by
    triangulating tracked features with the GT camera poses (structure from known
    motion) over a W-frame baseline, taking the median ground z, then linearly
    interpolating across all frames (terrain varies smoothly).  This uses GT poses
    only to synthesise the 'DEM'; on the real drone AGL = baro_altitude - DEM, both
    real sensors.  Returns an array aligned to recs (one AGL per index)."""
    Ks = K.copy(); Ks[:2, :] *= scale
    Kinv = np.linalg.inv(Ks)
    lk = dict(winSize=(21, 21), maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    feat = dict(maxCorners=400, qualityLevel=0.01, minDistance=10, blockSize=7)

    def load(p):
        im = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        return cv2.resize(im, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale != 1 else im

    idxs, agls = [], []
    for i in range(0, len(recs) - W, step):
        r0, r1 = recs[i], recs[i + W]
        im0, im1 = load(r0["img"]), load(r1["img"])
        p0 = cv2.goodFeaturesToTrack(im0, mask=None, **feat)
        if p0 is None:
            continue
        p1, stt, _ = cv2.calcOpticalFlowPyrLK(im0, im1, p0, None, **lk)
        stt = stt.reshape(-1).astype(bool)
        p0g, p1g = p0.reshape(-1, 2)[stt], p1.reshape(-1, 2)[stt]
        C0, C1 = r0["gt"], r1["gt"]
        R0, R1 = r0["R_wb"] @ R_CtoI, r1["R_wb"] @ R_CtoI
        terr = []
        for (u0, v0), (u1, v1) in zip(p0g, p1g):
            d0 = R0 @ (Kinv @ np.array([u0, v0, 1.0])); d0 /= np.linalg.norm(d0)
            d1 = R1 @ (Kinv @ np.array([u1, v1, 1.0])); d1 /= np.linalg.norm(d1)
            X = _triangulate_midpoint(C0, d0, C1, d1)
            if X is not None and X[2] < C0[2] - 1.0:        # ground below camera
                terr.append(X[2])
        if len(terr) >= min_pts:
            idxs.append(i)
            agls.append(C0[2] - np.median(terr))
    if len(idxs) < 2:
        raise RuntimeError("AGL triangulation failed — too few valid frames")
    return np.interp(np.arange(len(recs)), idxs, agls)


def run(d, scale, max_frames, min_track, depth_source="baro", stride=1, fb_check=True,
        agl_arr=None, attitude_R=None):
    K, R_CtoI, recs = load_dataset(d)
    if max_frames:
        recs = recs[:max_frames]
    if attitude_R is not None:              # use IMU-AHRS attitude instead of GT
        for i in range(len(recs)):
            recs[i]["R_wb"] = attitude_R[i]
    Ks = K.copy()
    Ks[:2, :] *= scale
    Kinv = np.linalg.inv(Ks)
    fxs = Ks[0, 0]

    agl = None
    if depth_source == "agl":
        if agl_arr is not None:               # reuse a precomputed AGL (fast sweeps)
            agl = agl_arr
        else:
            print("  computing true AGL (GT-pose triangulation; stand-in for baro-DEM)...")
            agl = compute_true_agl(recs, K, R_CtoI, scale)
        print(f"  AGL: median {np.median(agl):.0f} m, range [{agl.min():.0f}, {agl.max():.0f}] m")

    lk = dict(winSize=(21, 21), maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    feat = dict(maxCorners=600, qualityLevel=0.01, minDistance=8, blockSize=7)

    def load(path):
        im = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if scale != 1.0:
            im = cv2.resize(im, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        return im

    pos = recs[0]["gt"].copy()             # start at GT origin (0,0,0)
    pos[2] = recs[0]["h"]
    est = [pos.copy()]
    proc = [0]                              # recs indices that have an est entry
    n_used = []
    impl_depth = [np.nan]                   # depth implied by flow + GT motion (validation)

    prev = load(recs[0]["img"])
    # process every `stride`-th frame: at 30 fps the 1-frame flow is ~0.3 px (poor
    # SNR); a larger baseline gives a bigger, cleaner displacement -> less drift.
    for i in range(stride, len(recs), stride):
        cur = load(recs[i]["img"])
        r0, r1 = recs[i - stride], recs[i]
        dt = (r1["ts"] - r0["ts"]) / 1e9

        p0 = cv2.goodFeaturesToTrack(prev, mask=None, **feat)
        used = 0
        idep = np.nan
        if p0 is not None and len(p0) >= min_track:
            p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, cur, p0, None, **lk)
            st = st.reshape(-1).astype(bool)
            if fb_check:                     # forward-backward consistency: re-track
                p0b, st2, _ = cv2.calcOpticalFlowPyrLK(cur, prev, p1, None, **lk)
                fbe = np.linalg.norm(p0.reshape(-1, 2) - p0b.reshape(-1, 2), axis=1)
                st = st & st2.reshape(-1).astype(bool) & (fbe < 1.0)
            p0g, p1g = p0.reshape(-1, 2)[st], p1.reshape(-1, 2)[st]

            # camera frames cam0->world via attitude and the rigid extrinsic
            R_wc0 = r0["R_wb"] @ R_CtoI
            R_wc1 = r1["R_wb"] @ R_CtoI
            R_c1c0 = R_wc1.T @ R_wc0          # rotates a cam0 direction into cam1
            # depth scale source: baro height-above-takeoff (wrong over terrain) or
            # true AGL = camera_alt - terrain_elev (the metric fix we are testing).
            h0 = max(agl[i - 1] if agl is not None else r0["h"], 0.3)

            A, b, tfl = [], [], []
            for (u0, v0), (u1, v1) in zip(p0g, p1g):
                n0 = Kinv @ np.array([u0, v0, 1.0])     # normalized ray cam0
                n1 = Kinv @ np.array([u1, v1, 1.0])
                x0, y0 = n0[0], n0[1]
                # predicted location under rotation only (point at infinity)
                pr = R_c1c0 @ n0
                if pr[2] <= 1e-6:
                    continue
                pr = pr / pr[2]
                du, dv = n1[0] - pr[0], n1[1] - pr[1]   # translational flow (norm)
                # ground depth along the ray: C_z + Z*(R_wc0 @ n0)_z = 0, C_z = h0 (up)
                dz = (R_wc0 @ np.array([x0, y0, 1.0]))[2]
                if abs(dz) < 1e-3:
                    continue
                Z = -h0 / dz
                if Z <= 0 or Z > 50 * h0:
                    continue
                # Z*du = -tx + x0*tz ;  Z*dv = -ty + y0*tz
                A.append([-1, 0, x0]); b.append(Z * du)
                A.append([0, -1, y0]); b.append(Z * dv)
                tfl.append(np.hypot(du, dv))

            if len(A) >= 2 * min_track:
                A = np.asarray(A); b = np.asarray(b)
                t_cam, *_ = np.linalg.lstsq(A, b, rcond=None)
                # one IRLS reweight to suppress moving objects / mismatches
                resid = (A @ t_cam - b).reshape(-1, 2)
                rn = np.linalg.norm(resid, axis=1)
                med = np.median(rn) + 1e-9
                w = np.repeat((rn < 3 * med).astype(float), 2)
                if w.sum() >= 2 * min_track:
                    Aw, bw = A * w[:, None], b * w
                    t_cam, *_ = np.linalg.lstsq(Aw, bw, rcond=None)
                dC = R_wc0 @ t_cam            # camera = body origin offset ignored (4cm)
                pos[0] += dC[0]
                pos[1] += dC[1]
                used = int(w.sum() // 2) if 'w' in dir() else len(b) // 2
                # validation only: depth implied by flow + GT horizontal step.
                # (compares the TRUE camera-to-ground depth against baro height.)
                gstep = np.linalg.norm((r1["gt"] - r0["gt"])[:2])
                mtf = np.median(tfl) if tfl else 0.0
                if gstep > 0.4 and mtf > 1e-4:
                    idep = gstep / mtf

        pos[2] = r1["h"]                      # altitude straight from baro
        est.append(pos.copy())
        proc.append(i)
        n_used.append(used)
        impl_depth.append(idep)
        prev = cur
        if i % 600 == 0:
            print(f"  frame {i}/{len(recs)}  used~{used} pts")

    est = np.array(est)
    recs = [recs[k] for k in proc]            # align recs/GT to processed frames
    gt = np.array([r["gt"] for r in recs])
    return est, gt, np.array(n_used), np.array(impl_depth), recs


def plot(est, gt, n_used, impl_depth, recs, out, depth_source="baro"):
    t = np.array([(r["ts"] - recs[0]["ts"]) / 1e9 for r in recs])
    err = np.linalg.norm(est[:, :2] - gt[:, :2], axis=1)
    path_len = np.sum(np.linalg.norm(np.diff(gt[:, :2], axis=0), axis=1))
    est_len = np.sum(np.linalg.norm(np.diff(est[:, :2], axis=0), axis=1))

    fig, ax = plt.subplots(2, 2, figsize=(13, 10))
    a = ax[0, 0]
    a.plot(gt[:, 0], gt[:, 1], "b-", lw=1.5, label="GT")
    a.plot(est[:, 0], est[:, 1], "r-", lw=1.2, label="flow-odom")
    a.plot(gt[0, 0], gt[0, 1], "go", ms=8); a.plot(gt[-1, 0], gt[-1, 1], "ks", ms=7)
    a.plot(est[-1, 0], est[-1, 1], "r*", ms=12)
    a.set_title("Top-down trajectory (ENU)"); a.set_xlabel("East (m)"); a.set_ylabel("North (m)")
    a.axis("equal"); a.legend(); a.grid(alpha=0.3)

    a = ax[0, 1]
    a.plot(t, err, "r-")
    a.set_title(f"Horizontal error vs GT  (final {err[-1]:.1f} m = "
                f"{100*err[-1]/path_len:.1f}% of {path_len:.0f} m path)")
    a.set_xlabel("time (s)"); a.set_ylabel("error (m)"); a.grid(alpha=0.3)

    a = ax[1, 0]
    a.plot(t, gt[:, 2], "b-", label="GT z")
    a.plot(t, est[:, 2], "r--", label="baro z")
    a.set_title("Altitude"); a.set_xlabel("time (s)"); a.set_ylabel("up (m)")
    a.legend(); a.grid(alpha=0.3)

    # The key diagnosis: TRUE camera-to-ground depth (from flow + GT) vs the baro
    # height-above-takeoff that the scaling assumes.  On Cesium real terrain the
    # ground falls away into a valley, so depth >> baro height -> scale too small.
    a = ax[1, 1]
    baro_h = np.array([r["h"] for r in recs])
    m = np.isfinite(impl_depth)
    a.plot(t[m], impl_depth[m], "m.", ms=3, label="true depth (flow+GT)")
    a.plot(t, baro_h, "g-", lw=1.5, label="baro height a.g.t.")
    ratio = np.nanmedian(impl_depth[m] / baro_h[m])
    a.set_title(f"Why scale is off: ground is ~{ratio:.0f}x deeper than baro height\n"
                "(terrain falls away — baro a.g.t. != camera-ground depth)")
    a.set_xlabel("time (s)"); a.set_ylabel("metres"); a.legend(); a.grid(alpha=0.3)

    fig.suptitle(f"Altitude-scaled optical-flow odometry vs GT — Isaac Sim nadir "
                 f"(depth source: {depth_source})", fontsize=13)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"\nsaved {out}")
    print(f"depth source      : {depth_source}")
    print(f"final horiz error : {err[-1]:.1f} m  ({100*err[-1]/path_len:.1f}% of path)")
    print(f"mean  horiz error : {err.mean():.1f} m")
    print(f"path length  GT   : {path_len:.0f} m")
    print(f"path length  est  : {est_len:.0f} m   (scale est/GT = {est_len/path_len:.2f})")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=DEFAULT_DIR)
    ap.add_argument("--scale", type=float, default=0.5)
    ap.add_argument("--max_frames", type=int, default=0)
    ap.add_argument("--min_track", type=int, default=30)
    ap.add_argument("--depth", choices=["baro", "agl"], default="baro",
                    help="depth/scale source: 'baro' height-above-takeoff (wrong over "
                         "terrain) or 'agl' true camera-alt - terrain-elev (stand-in for baro-DEM)")
    ap.add_argument("--stride", type=int, default=1,
                    help="process every Nth frame (bigger baseline -> better flow SNR at high fps)")
    ap.add_argument("--no_fb", action="store_true", help="disable forward-backward LK consistency check")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    out = args.out or os.path.join(args.dir, f"flow_vs_gt_{args.depth}.png")
    est, gt, n_used, impl_depth, recs = run(args.dir, args.scale, args.max_frames,
                                            args.min_track, args.depth, args.stride,
                                            not args.no_fb)
    plot(est, gt, n_used, impl_depth, recs, out, args.depth)
