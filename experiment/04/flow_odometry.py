#!/usr/bin/env python3
"""Altitude-scaled optical-flow odometry — tracker comparison variant.

Exp04 extension: adds --tracker {lk, fast_lk, farneback, dis, orb, sparse_raft}
so six flow/matching methods can be compared on identical odometry math.

  lk          : Shi-Tomasi + pyramidal calcOpticalFlowPyrLK + FB check (baseline)
  fast_lk     : FAST corners (OpenVINS style) + same pyramidal LK + FB check
  farneback   : calcOpticalFlowFarneback (dense), sampled at Shi-Tomasi points + FB
  dis         : DISOpticalFlow medium preset (dense), sampled at Shi-Tomasi points + FB
  orb         : ORB_create detect+describe + BFMatcher (Hamming) + RANSAC (F-mat)
  sparse_raft : RAFT-Small dense flow (torchvision), sampled at Shi-Tomasi points + FB

All odometry math (de-rotate, ray-ground depth, LS solve, IRLS) is identical across
methods.  Benchmark sub-steps: load | detect | track (flow/match+filter) | solve.

ORB paradigm differs from the others: detection happens on the CURRENT frame (cached
and reused as prev next iteration); bm_detect times cur-frame detection, bm_track
times BFMatcher + RANSAC.  All other methods detect on prev and flow to cur.

sparse_raft requires torchvision >= 0.13 (optical_flow module).  If unavailable the
script exits with a clear message rather than crashing mid-run.
"""
import argparse
import csv
import json
import os
import sys
import time

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DEFAULT_DIR = "/home/innovation/pai/drone-vio/_in/isaac-sim-20260623"

FRAME_BUDGET_MS = 80.0
LK_PARAMS  = dict(winSize=(21, 21), maxLevel=3,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
FEAT_PARAMS = dict(maxCorners=600, qualityLevel=0.01, minDistance=8, blockSize=7)
FB_THRESH   = 1.0  # forward-backward consistency (px)
ORB_N       = 600  # max ORB keypoints (matched to FEAT_PARAMS maxCorners)


# ─── tracker helpers ─────────────────────────────────────────────────────────

def detect_points(gray, tracker):
    """Detect feature points (non-ORB methods only); returns (N,1,2) float32 or None."""
    if tracker == "fast_lk":
        fast = cv2.FastFeatureDetector_create(threshold=20)
        kps  = fast.detect(gray, None)
        if not kps:
            return None
        kps = sorted(kps, key=lambda k: -k.response)[:FEAT_PARAMS["maxCorners"]]
        return np.array([k.pt for k in kps], np.float32).reshape(-1, 1, 2)
    return cv2.goodFeaturesToTrack(gray, mask=None, **FEAT_PARAMS)


def track_lk(prev, cur, p0):
    """Pyramidal LK + bidirectional FB check.  Returns (p0g, p1g) inlier pairs."""
    p1,  st,  _ = cv2.calcOpticalFlowPyrLK(prev, cur, p0, None, **LK_PARAMS)
    p0b, st2, _ = cv2.calcOpticalFlowPyrLK(cur, prev, p1, None, **LK_PARAMS)
    fbe  = np.linalg.norm(p0.reshape(-1, 2) - p0b.reshape(-1, 2), axis=1)
    good = st.reshape(-1).astype(bool) & st2.reshape(-1).astype(bool) & (fbe < FB_THRESH)
    return p0.reshape(-1, 2)[good], p1.reshape(-1, 2)[good]


def _sample_dense(flow, pts):
    """Read (H,W,2) dense flow at integer-rounded point locations."""
    h, w = flow.shape[:2]
    ix = np.clip(pts[:, 0].astype(int), 0, w - 1)
    iy = np.clip(pts[:, 1].astype(int), 0, h - 1)
    return flow[iy, ix]


def track_dense(prev, cur, p0, compute_flow):
    """Generic dense-flow tracker with FB check.  compute_flow(a,b) -> (H,W,2)."""
    h, w  = prev.shape
    pts0  = p0.reshape(-1, 2)
    fwd   = compute_flow(prev, cur)
    pts1  = pts0 + _sample_dense(fwd, pts0)
    bk    = compute_flow(cur, prev)
    p0b   = pts1 + _sample_dense(bk, pts1)
    fbe   = np.linalg.norm(pts0 - p0b, axis=1)
    inbnd = (pts1[:, 0] >= 0) & (pts1[:, 0] < w) & (pts1[:, 1] >= 0) & (pts1[:, 1] < h)
    good  = inbnd & (fbe < FB_THRESH)
    return pts0[good], pts1[good]


def make_farneback():
    def compute(a, b):
        return cv2.calcOpticalFlowFarneback(a, b, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    return compute


def make_dis():
    obj = cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_MEDIUM)
    def compute(a, b):
        return obj.calc(a, b, None)
    return compute


# ─── ORB + RANSAC ────────────────────────────────────────────────────────────

def make_orb():
    det = cv2.ORB_create(nfeatures=ORB_N, fastThreshold=20)
    bf  = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    return det, bf


def _orb_match(kp0, des0, kp1, des1, bf):
    """BFMatcher + RANSAC fundamental-matrix filter.  Returns (p0g, p1g)."""
    empty = np.empty((0, 2), np.float32)
    if des0 is None or des1 is None or len(des0) < 8 or len(des1) < 8:
        return empty, empty
    matches = bf.match(des0, des1)
    if len(matches) < 8:
        return empty, empty
    p0g = np.float32([kp0[m.queryIdx].pt for m in matches])
    p1g = np.float32([kp1[m.trainIdx].pt for m in matches])
    try:
        _, mask = cv2.findFundamentalMat(p0g, p1g, cv2.FM_RANSAC, 1.0, 0.999)
    except cv2.error:
        return p0g, p1g
    if mask is None:
        return p0g, p1g
    mask = mask.reshape(-1).astype(bool)
    return p0g[mask], p1g[mask]


# ─── Sparse RAFT ─────────────────────────────────────────────────────────────

def make_sparse_raft():
    """Load RAFT-Small from torchvision.  Requires torchvision >= 0.13."""
    try:
        import torch
        from torchvision.models.optical_flow import raft_small
    except ImportError as e:
        sys.exit(f"sparse_raft requires torchvision >= 0.13 with optical_flow module: {e}")
    device = "cuda" if torch.cuda.is_available() else "cpu"
    try:
        from torchvision.models.optical_flow import Raft_Small_Weights
        model = raft_small(weights=Raft_Small_Weights.DEFAULT, progress=False)
    except (ImportError, AttributeError):
        model = raft_small(pretrained=True, progress=False)
    model = model.eval().to(device)
    print(f"  RAFT-Small loaded on {device}")
    return model, device


def _raft_flow(model, device, gray_a, gray_b):
    """Run RAFT-Small on a grayscale pair; returns (H,W,2) float32 flow."""
    import torch
    h, w = gray_a.shape
    # pad to nearest multiple of 8 (RAFT requirement)
    ph = (8 - h % 8) % 8
    pw = (8 - w % 8) % 8
    if ph or pw:
        ga = np.pad(gray_a, ((0, ph), (0, pw)))
        gb = np.pad(gray_b, ((0, ph), (0, pw)))
    else:
        ga, gb = gray_a, gray_b
    # RAFT expects float32 [0,255] (B,3,H,W)
    ta = torch.from_numpy(ga).float()[None, None].repeat(1, 3, 1, 1).to(device)
    tb = torch.from_numpy(gb).float()[None, None].repeat(1, 3, 1, 1).to(device)
    with torch.no_grad():
        flows = model(ta, tb)
    return flows[-1][0].permute(1, 2, 0).cpu().numpy()[:h, :w]


# ─── dataset loading ─────────────────────────────────────────────────────────

def quat_xyzw_to_R(qx, qy, qz, qw):
    n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
    ])


def load_dataset(d):
    calib = json.load(open(os.path.join(d, "cam_calib.json")))
    K = np.array([[calib["intrinsics"]["fx"], 0, calib["intrinsics"]["cx"]],
                  [0, calib["intrinsics"]["fy"], calib["intrinsics"]["cy"]],
                  [0, 0, 1]])
    qw, qx, qy, qz = calib["extrinsic_body_to_cam"]["quaternion_wxyz"]
    R_CtoI = quat_xyzw_to_R(qx, qy, qz, qw) @ np.diag([1.0, -1.0, -1.0])
    frames = list(csv.DictReader(open(os.path.join(d, "frames.csv"))))
    poses  = {int(r["frame"]): r for r in csv.DictReader(open(os.path.join(d, "poses.csv")))}
    baro   = {int(r["frame"]): float(r["pressure_altitude_m"])
               for r in csv.DictReader(open(os.path.join(d, "baro.csv")))}
    baro0  = baro[min(baro)]
    img_dir = os.path.join(d, "images", "cam0")
    recs = []
    for fr in frames:
        fi = int(fr["frame"])
        if fi not in poses or fi not in baro:
            continue
        p = poses[fi]
        recs.append({
            "ts":   int(fr["ts_ns"]),
            "img":  os.path.join(img_dir, os.path.basename(fr["image_path"])),
            "R_wb": quat_xyzw_to_R(float(p["qx"]), float(p["qy"]),
                                    float(p["qz"]), float(p["qw"])),
            "h":    baro[fi] - baro0,
            "gt":   np.array([float(p["x"]), float(p["y"]), float(p["z"])]),
        })
    return K, R_CtoI, recs


def compute_ahrs_attitude(d, recs, Kp=1.0, mag_gain=0.0, mag_noise_deg=0.0):
    from scipy.spatial.transform import Rotation as Rot
    rows  = list(csv.DictReader(open(os.path.join(d, "imu.csv"))))
    ts    = np.array([int(r["ts_ns"]) for r in rows]) / 1e9
    gyro  = np.array([[float(r["wx"]), float(r["wy"]), float(r["wz"])] for r in rows])
    acc   = np.array([[float(r["ax"]), float(r["ay"]), float(r["az"])] for r in rows])
    flip  = np.diag([1.0, -1.0, -1.0])
    R     = recs[0]["R_wb"] @ flip
    g_up  = np.array([0.0, 0.0, 9.81])
    rec_ts  = [r["ts"] / 1e9 for r in recs]
    gt_yaw  = [np.arctan2(r["R_wb"][1, 0], r["R_wb"][0, 0]) for r in recs]
    rng   = np.random.default_rng(0)
    out   = [recs[0]["R_wb"]]
    ri    = 1
    for k in range(1, len(rows)):
        dt = ts[k] - ts[k-1]
        if dt <= 0 or dt > 0.1:
            dt = 0.004
        w  = gyro[k].copy()
        a  = acc[k]; an = np.linalg.norm(a)
        if an > 1e-3:
            v_meas = a / an
            v_pred = R.T @ g_up; v_pred /= np.linalg.norm(v_pred)
            w = w + Kp * np.cross(v_meas, v_pred)
        R = R @ Rot.from_rotvec(w * dt).as_matrix()
        while ri < len(recs) and ts[k] >= rec_ts[ri]:
            if mag_gain > 0.0:
                yaw  = np.arctan2(R[1, 0], R[0, 0])
                meas = gt_yaw[ri] + np.radians(mag_noise_deg) * rng.standard_normal()
                dpsi = np.arctan2(np.sin(meas - yaw), np.cos(meas - yaw))
                R = Rot.from_rotvec([0, 0, mag_gain * dpsi]).as_matrix() @ R
            out.append(R @ flip)
            ri += 1
    while len(out) < len(recs):
        out.append(R @ flip)
    return out


def _triangulate_midpoint(C0, d0, C1, d1):
    r  = C0 - C1
    a, bb, c = d0@d0, d0@d1, d1@d1
    M  = np.array([[a, -bb], [bb, -c]])
    rhs = np.array([-(d0@r), -(d1@r)])
    if abs(np.linalg.det(M)) < 1e-9 or abs(bb) > 0.99995:
        return None
    l0, l1 = np.linalg.solve(M, rhs)
    if l0 <= 0 or l1 <= 0:
        return None
    return 0.5 * ((C0 + l0*d0) + (C1 + l1*d1))


def compute_true_agl(recs, K, R_CtoI, scale, W=10, step=4, min_pts=20):
    Ks   = K.copy(); Ks[:2, :] *= scale
    Kinv = np.linalg.inv(Ks)
    lk   = dict(winSize=(21, 21), maxLevel=3,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    feat = dict(maxCorners=400, qualityLevel=0.01, minDistance=10, blockSize=7)

    def _load(p):
        im = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        return cv2.resize(im, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale != 1 else im

    idxs, agls = [], []
    for i in range(0, len(recs) - W, step):
        r0, r1 = recs[i], recs[i + W]
        im0, im1 = _load(r0["img"]), _load(r1["img"])
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
            X  = _triangulate_midpoint(C0, d0, C1, d1)
            if X is not None and X[2] < C0[2] - 1.0:
                terr.append(X[2])
        if len(terr) >= min_pts:
            idxs.append(i); agls.append(C0[2] - np.median(terr))
    if len(idxs) < 2:
        raise RuntimeError("AGL triangulation failed — too few valid frames")
    return np.interp(np.arange(len(recs)), idxs, agls)


# ─── benchmark printer ───────────────────────────────────────────────────────

def _print_benchmark(bm_load, bm_detect, bm_track, bm_solve, bm_total, tracker):
    steps = [("load", bm_load), ("detect", bm_detect),
             ("track", bm_track), ("solve", bm_solve), ("TOTAL", bm_total)]
    w = 70
    print(f"\n{'─'*w}")
    print(f"  BENCHMARK [{tracker}] — per-frame latency (ms)  [budget {FRAME_BUDGET_MS:.0f} ms]")
    print(f"{'─'*w}")
    print(f"  {'Sub-step':<14} {'Mean':>8} {'P95':>8} {'Max':>8}")
    print(f"{'─'*w}")
    for name, vals in steps:
        if not vals:
            continue
        a = np.array(vals)
        print(f"  {name:<14} {a.mean():>8.1f} {np.percentile(a,95):>8.1f} {a.max():>8.1f}")
    print(f"{'─'*w}")
    tot    = np.array(bm_total)
    margin = FRAME_BUDGET_MS - tot.mean()
    over   = int((tot > FRAME_BUDGET_MS).sum())
    print(f"  Real-time margin  : {margin:.1f} ms  ({'OK' if margin > 0 else 'FAIL'})")
    print(f"  Over-budget frames: {over}/{len(tot)}  ({100*over/max(len(tot),1):.1f}%)")
    print(f"{'─'*w}\n")


# ─── main odometry loop ──────────────────────────────────────────────────────

def run(d, scale, max_frames, min_track, depth_source="baro", stride=1,
        agl_arr=None, attitude_R=None, skip_frames=0, tracker="lk", benchmark=False):
    K, R_CtoI, recs = load_dataset(d)
    if skip_frames:
        recs = recs[skip_frames:]
    if max_frames:
        recs = recs[:max_frames]
    if attitude_R is not None:
        for i in range(len(recs)):
            recs[i]["R_wb"] = attitude_R[i]
    Ks   = K.copy(); Ks[:2, :] *= scale
    Kinv = np.linalg.inv(Ks)

    agl = None
    if depth_source == "agl":
        if agl_arr is not None:
            agl = agl_arr
        else:
            cache_path = os.path.join(d, "agl_cache.npz")
            if os.path.exists(cache_path):
                agl = np.load(cache_path)["agl"]
                print(f"  loaded cached AGL from {cache_path}")
            else:
                print("  computing true AGL (GT-pose triangulation)...")
                agl = compute_true_agl(recs, K, R_CtoI, scale)
                np.savez(cache_path, agl=agl)
                print(f"  saved AGL cache → {cache_path}")
        print(f"  AGL: median {np.median(agl):.0f} m, range [{agl.min():.0f}, {agl.max():.0f}] m")

    # one-time initialisation per tracker type
    flow_fn       = None
    raft_model    = raft_device = None
    orb_det = orb_bf = None
    _prev_orb_kp  = _prev_orb_des = None

    if tracker == "farneback":
        flow_fn = make_farneback()
    elif tracker == "dis":
        flow_fn = make_dis()
    elif tracker == "sparse_raft":
        raft_model, raft_device = make_sparse_raft()
        flow_fn = lambda a, b: _raft_flow(raft_model, raft_device, a, b)
    elif tracker == "orb":
        orb_det, orb_bf = make_orb()

    def load(path):
        im = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if scale != 1.0:
            im = cv2.resize(im, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        return im

    pos = recs[0]["gt"].copy(); pos[2] = recs[0]["h"]
    est  = [pos.copy()]
    proc = [0]
    n_used = []

    if benchmark:
        bm_load, bm_detect, bm_track, bm_solve, bm_total = [], [], [], [], []

    prev = load(recs[0]["img"])
    if tracker == "orb":
        _prev_orb_kp, _prev_orb_des = orb_det.detectAndCompute(prev, None)

    for i in range(stride, len(recs), stride):
        if benchmark:
            _tf = time.perf_counter()

        # ── load current frame ──
        if benchmark:
            _t0 = time.perf_counter()
        cur = load(recs[i]["img"])
        if benchmark:
            bm_load.append((time.perf_counter() - _t0) * 1000)

        r0, r1 = recs[i - stride], recs[i]

        # ── detect / extract features ──
        if tracker == "orb":
            if benchmark:
                _t0 = time.perf_counter()
            cur_orb_kp, cur_orb_des = orb_det.detectAndCompute(cur, None)
            if benchmark:
                bm_detect.append((time.perf_counter() - _t0) * 1000)
        else:
            if benchmark:
                _t0 = time.perf_counter()
            p0 = detect_points(prev, tracker)
            if benchmark:
                bm_detect.append((time.perf_counter() - _t0) * 1000)

        # ── track / match ──
        p0g = p1g = np.empty((0, 2), np.float32)

        if tracker == "orb":
            if benchmark:
                _t0 = time.perf_counter()
            p0g, p1g = _orb_match(_prev_orb_kp, _prev_orb_des,
                                   cur_orb_kp, cur_orb_des, orb_bf)
            if benchmark:
                bm_track.append((time.perf_counter() - _t0) * 1000)
            _prev_orb_kp, _prev_orb_des = cur_orb_kp, cur_orb_des
        elif p0 is not None and len(p0) >= min_track:
            if benchmark:
                _t0 = time.perf_counter()
            if tracker in ("lk", "fast_lk"):
                p0g, p1g = track_lk(prev, cur, p0)
            else:
                p0g, p1g = track_dense(prev, cur, p0, flow_fn)
            if benchmark:
                bm_track.append((time.perf_counter() - _t0) * 1000)
        else:
            if benchmark:
                bm_track.append(0.0)

        # ── odometry: de-rotate + depth + LS solve ──
        used = 0
        if len(p0g) >= min_track:
            R_wc0  = r0["R_wb"] @ R_CtoI
            R_wc1  = r1["R_wb"] @ R_CtoI
            R_c1c0 = R_wc1.T @ R_wc0
            h0     = max(agl[i - 1] if agl is not None else r0["h"], 0.3)

            if benchmark:
                _t0 = time.perf_counter()
            A, b = [], []
            for (u0, v0), (u1, v1) in zip(p0g, p1g):
                n0 = Kinv @ np.array([u0, v0, 1.0])
                n1 = Kinv @ np.array([u1, v1, 1.0])
                x0, y0 = n0[0], n0[1]
                pr = R_c1c0 @ n0
                if pr[2] <= 1e-6:
                    continue
                pr = pr / pr[2]
                du, dv = n1[0] - pr[0], n1[1] - pr[1]
                dz = (R_wc0 @ np.array([x0, y0, 1.0]))[2]
                if abs(dz) < 1e-3:
                    continue
                Z = -h0 / dz
                if Z <= 0 or Z > 50 * h0:
                    continue
                A.append([-1, 0, x0]); b.append(Z * du)
                A.append([0, -1, y0]); b.append(Z * dv)

            if len(A) >= 2 * min_track:
                A = np.asarray(A); b = np.asarray(b)
                t_cam, *_ = np.linalg.lstsq(A, b, rcond=None)
                resid = (A @ t_cam - b).reshape(-1, 2)
                rn    = np.linalg.norm(resid, axis=1)
                med   = np.median(rn) + 1e-9
                w     = np.repeat((rn < 3 * med).astype(float), 2)
                if w.sum() >= 2 * min_track:
                    t_cam, *_ = np.linalg.lstsq(A * w[:, None], b * w, rcond=None)
                dC = R_wc0 @ t_cam
                pos[0] += dC[0]; pos[1] += dC[1]
                used = int(w.sum() // 2)
            if benchmark:
                bm_solve.append((time.perf_counter() - _t0) * 1000)
        else:
            if benchmark:
                bm_solve.append(0.0)

        pos[2] = r1["h"]
        est.append(pos.copy()); proc.append(i); n_used.append(used)
        prev = cur

        if benchmark:
            bm_total.append((time.perf_counter() - _tf) * 1000)
        if i % 600 == 0:
            print(f"  frame {i}/{len(recs)}  used~{used} pts")

    if benchmark:
        _print_benchmark(bm_load, bm_detect, bm_track, bm_solve, bm_total, tracker)

    est  = np.array(est)
    recs = [recs[k] for k in proc]
    gt   = np.array([r["gt"] for r in recs])
    return est, gt, np.array(n_used), recs


# ─── plot ────────────────────────────────────────────────────────────────────

def plot(est, gt, n_used, recs, out, depth_source, tracker):
    t        = np.array([(r["ts"] - recs[0]["ts"]) / 1e9 for r in recs])
    err      = np.linalg.norm(est[:, :2] - gt[:, :2], axis=1)
    path_len = np.sum(np.linalg.norm(np.diff(gt[:, :2], axis=0), axis=1))
    est_len  = np.sum(np.linalg.norm(np.diff(est[:, :2], axis=0), axis=1))

    fig, ax = plt.subplots(2, 2, figsize=(13, 10))
    a = ax[0, 0]
    a.plot(gt[:, 0], gt[:, 1], "b-", lw=1.5, label="GT")
    a.plot(est[:, 0], est[:, 1], "r-", lw=1.2, label=f"flow-odom [{tracker}]")
    a.plot(gt[0, 0], gt[0, 1], "go", ms=8); a.plot(gt[-1, 0], gt[-1, 1], "ks", ms=7)
    a.plot(est[-1, 0], est[-1, 1], "r*", ms=12)
    a.set_title("Top-down trajectory (ENU)"); a.set_xlabel("East (m)"); a.set_ylabel("North (m)")
    a.axis("equal"); a.legend(); a.grid(alpha=0.3)

    a = ax[0, 1]
    a.plot(t, err, "r-")
    a.set_title(f"Horizontal error — final {err[-1]:.1f} m = {100*err[-1]/path_len:.1f}% of {path_len:.0f} m")
    a.set_xlabel("time (s)"); a.set_ylabel("error (m)"); a.grid(alpha=0.3)

    a = ax[1, 0]; a.axis("off")
    stats = (
        f"{'FLIGHT STATISTICS':^38}\n{'─'*38}\n"
        f"{'Tracker':<22} {tracker}\n"
        f"{'Depth source':<22} {depth_source}\n"
        f"{'Duration':<22} {t[-1]-t[0]:.0f} s  ({(t[-1]-t[0])/60:.1f} min)\n"
        f"{'Frames processed':<22} {len(recs)}\n{'─'*38}\n"
        f"{'PATH LENGTH':<22}\n"
        f"  GT                   {path_len:.1f} m\n"
        f"  Estimated            {est_len:.1f} m\n"
        f"  Scale (est/GT)       {est_len/path_len:.3f}\n{'─'*38}\n"
        f"{'HORIZONTAL ERROR':<22}\n"
        f"  Final                {err[-1]:.1f} m  ({100*err[-1]/path_len:.2f}%)\n"
        f"  Mean                 {err.mean():.1f} m  ({100*err.mean()/path_len:.2f}%)\n"
        f"  Max                  {err.max():.1f} m\n"
        f"  RMSE                 {np.sqrt((err**2).mean()):.1f} m\n{'─'*38}\n"
        f"{'Mean inliers/frame':<22} {np.mean(n_used):.0f}\n"
    )
    a.text(0.05, 0.97, stats, transform=a.transAxes, va="top", ha="left",
           fontsize=9.5, fontfamily="monospace",
           bbox=dict(boxstyle="round,pad=0.5", facecolor="#f7f7f7", edgecolor="#aaaaaa"))

    a = ax[1, 1]
    a.plot(t[1:], n_used, "m.", ms=2)
    a.set_title("Tracked/matched inlier points per frame")
    a.set_xlabel("time (s)"); a.set_ylabel("inlier count"); a.grid(alpha=0.3)

    fig.suptitle(f"Optical-flow odometry [{tracker}] vs GT — Isaac Sim nadir "
                 f"(depth: {depth_source})", fontsize=13)
    fig.tight_layout(); fig.savefig(out, dpi=110)
    print(f"\nsaved {out}")
    print(f"tracker           : {tracker}")
    print(f"final horiz error : {err[-1]:.1f} m  ({100*err[-1]/path_len:.1f}% of path)")
    print(f"RMSE              : {np.sqrt((err**2).mean()):.1f} m")
    print(f"mean inliers/frame: {np.mean(n_used):.0f}")
    print(f"path GT / est / scale: {path_len:.0f} m / {est_len:.0f} m / {est_len/path_len:.2f}")


# ─── entry point ─────────────────────────────────────────────────────────────

TRACKERS = ["lk", "fast_lk", "farneback", "dis", "orb", "sparse_raft"]

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir",        default=DEFAULT_DIR)
    ap.add_argument("--scale",      type=float, default=0.5)
    ap.add_argument("--max_frames", type=int, default=0)
    ap.add_argument("--min_track",  type=int, default=30)
    ap.add_argument("--depth",      choices=["baro", "agl"], default="baro")
    ap.add_argument("--stride",     type=int, default=1)
    ap.add_argument("--skip_frames",type=int, default=0)
    ap.add_argument("--attitude",   choices=["gt", "ahrs", "ahrs_compass"], default="gt")
    ap.add_argument("--tracker",    choices=TRACKERS, default="lk",
                    help="lk | fast_lk | farneback | dis | orb | sparse_raft")
    ap.add_argument("--out",        default=None)
    ap.add_argument("--benchmark",  action="store_true",
                    help="profile per-frame sub-step latency")
    args = ap.parse_args()

    EXP_DIR = os.path.dirname(os.path.abspath(__file__))
    out = args.out or os.path.join(
        EXP_DIR, os.path.basename(args.dir),
        f"flow_{args.depth}_{args.attitude}_{args.tracker}_s{args.stride}.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    att_R = None
    if args.attitude == "ahrs":
        _, _, recs0 = load_dataset(args.dir)
        att_R = compute_ahrs_attitude(args.dir, recs0)
        print("  using Mahony AHRS (gyro+accel only)")
    elif args.attitude == "ahrs_compass":
        _, _, recs0 = load_dataset(args.dir)
        att_R = compute_ahrs_attitude(args.dir, recs0, mag_gain=1.0)
        print("  using Mahony AHRS + compass")

    est, gt, n_used, recs = run(
        args.dir, args.scale, args.max_frames, args.min_track,
        args.depth, args.stride,
        attitude_R=att_R, skip_frames=args.skip_frames,
        tracker=args.tracker, benchmark=args.benchmark,
    )

    tvec  = np.array([(r["ts"] - recs[0]["ts"]) / 1e9 for r in recs])
    cache = os.path.join(args.dir,
                         f"tracker_trajs_{args.attitude}_{args.depth}_{args.tracker}.npz")
    np.savez(cache, est=est[:, :2], GT=gt[:, :2], t_vec=tvec)
    print(f"saved trajectory cache → {cache}")

    plot(est, gt, n_used, recs, out, args.depth, args.tracker)
