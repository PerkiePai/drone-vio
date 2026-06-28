#!/usr/bin/env python3
"""Reproduction of the *RaD-VIO* homography core on our nadir Isaac-Sim flight.

Paper: "RaD-VIO: Rangefinder-aided Downward Visual-Inertial Odometry"
       (arXiv:1810.08704).

What that paper does (the part we reproduce)
--------------------------------------------
A downward camera looking at the ground sees (approximately) a single planar surface.
RaD-VIO estimates the inter-frame motion from the **ground-plane homography**, then
fixes the otherwise-unobservable monocular scale with a **downward rangefinder**: the
homography decomposition yields translation only up to the plane distance d (it returns
t/d), and the rangefinder supplies the metric d (= AGL).  This is a genuinely different
estimation backbone from PX4FLOW (global average flow) and from our flow-odom
(per-point ray--ground least-squares): here a single homography models the whole plane.

Pipeline per consecutive frame pair:
  1. LK-track features  ->  pixel correspondences.
  2. RANSAC homography H (calibrated, on Ks).
  3. cv2.decomposeHomographyMat(H, Ks) -> 4 candidate {R, t/d, n}.
  4. Disambiguate: keep the physically-visible solutions, pick the rotation closest to
     the IMU/attitude-derived relative rotation R_c1c0 (RaD-VIO is IMU-aided).
  5. Metric translation t_cam = (t/d) * d, with d = rangefinder AGL (plane distance).
  6. Resolve the global translation sign from flow physics (camera moves opposite to the
     mean image flow -- no GT used).  Rotate to world (ENU) and integrate XY; Z = baro.

What is reproduced vs adapted
-----------------------------
* Reproduced: ground-plane homography decomposition, IMU-aided solution selection, and
  metric scaling of the translation by the rangefinder (the RaD-VIO scale fix).
* Adapted to our rig (identical to our flow-odom harness for a fair comparison):
  LK pyramidal tracking, AGL as the rangefinder stand-in (same `compute_true_agl` /
  `agl_cache.npz`), attitude from GT or the Mahony AHRS.  Nothing uses GT *position*.

Run:
  conda run -n cv python frontend/rad-vio/rad_vio.py \
      --dir _in/isaac-sim-20260624_2337 --scale 0.5 --depth agl --stride 5
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "flow-odom"))
from flow_odometry import (load_dataset, compute_true_agl, compute_ahrs_attitude)  # noqa: E402
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "downfacing-vio"))
from downfacing_vio import kitti_segment_drift, evaluate_and_plot  # noqa: E402

DEFAULT_DIR = "/home/innovation/pai/drone-vio/_in/isaac-sim-20260624_2337"


def _rot_angle(Ra, Rb):
    """Geodesic angle (rad) between two rotations."""
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.arccos(np.clip(c, -1.0, 1.0)))


def run(d, scale, max_frames, min_track, depth_source, stride, fb_check,
        agl_arr=None, attitude_R=None):
    K, R_CtoI, recs = load_dataset(d)
    if max_frames:
        recs = recs[:max_frames]
    if attitude_R is not None:
        for i in range(len(recs)):
            recs[i]["R_wb"] = attitude_R[i]
    Ks = K.copy(); Ks[:2, :] *= scale

    agl = None
    if depth_source == "agl":
        agl = agl_arr if agl_arr is not None else compute_true_agl(recs, K, R_CtoI, scale)
        print(f"  AGL (rangefinder stand-in): median {np.median(agl):.0f} m, "
              f"range [{agl.min():.0f}, {agl.max():.0f}] m")

    lk = dict(winSize=(21, 21), maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    feat = dict(maxCorners=600, qualityLevel=0.01, minDistance=8, blockSize=7)
    Kinv = np.linalg.inv(Ks)

    def load(p):
        im = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        return cv2.resize(im, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale != 1 else im

    pos = recs[0]["gt"].copy(); pos[2] = recs[0]["h"]
    est = [pos.copy()]; proc = [0]; n_used = []
    prev = load(recs[0]["img"])
    for i in range(stride, len(recs), stride):
        cur = load(recs[i]["img"])
        r0, r1 = recs[i - stride], recs[i]
        used = 0
        p0 = cv2.goodFeaturesToTrack(prev, mask=None, **feat)
        if p0 is not None and len(p0) >= min_track:
            p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, cur, p0, None, **lk)
            st = st.reshape(-1).astype(bool)
            if fb_check:
                p0b, st2, _ = cv2.calcOpticalFlowPyrLK(cur, prev, p1, None, **lk)
                fbe = np.linalg.norm(p0.reshape(-1, 2) - p0b.reshape(-1, 2), axis=1)
                st = st & st2.reshape(-1).astype(bool) & (fbe < 1.0)
            p0g, p1g = p0.reshape(-1, 2)[st], p1.reshape(-1, 2)[st]

            if len(p0g) >= min_track:
                H, hmask = cv2.findHomography(p0g, p1g, cv2.RANSAC, 3.0)
                if H is not None and hmask is not None and hmask.sum() >= min_track:
                    inl = hmask.ravel().astype(bool)
                    R_wc0 = r0["R_wb"] @ R_CtoI
                    R_wc1 = r1["R_wb"] @ R_CtoI
                    R_c1c0 = R_wc1.T @ R_wc0
                    Z = max(agl[i - 1] if agl is not None else r0["h"], 0.3)

                    num, Rs, Ts, Ns = cv2.decomposeHomographyMat(H, Ks)
                    # keep only physically-plausible decompositions (points in front)
                    poss = cv2.filterHomographyDecompByVisibleRefpoints(
                        Rs, Ns, p0g[inl].reshape(-1, 1, 2), p1g[inl].reshape(-1, 1, 2))
                    cand = poss.ravel().tolist() if poss is not None and len(poss) else list(range(num))
                    # IMU-aided selection: rotation closest to the attitude-derived one
                    best = min(cand, key=lambda c: _rot_angle(Rs[c], R_c1c0))
                    t_dir = Ts[best].ravel()
                    t_cam = t_dir * Z                     # rangefinder metric scale fix

                    # resolve global sign from flow physics (camera moves opposite the
                    # mean image flow); avoids the homography sign ambiguity, no GT.
                    n0 = Kinv @ np.c_[p0g[inl], np.ones(inl.sum())].T
                    n1 = Kinv @ np.c_[p1g[inl], np.ones(inl.sum())].T
                    mean_flow = (n1[:2] - n0[:2]).mean(axis=1)
                    if np.dot(t_cam[:2], -mean_flow) < 0:
                        t_cam = -t_cam
                    dC = R_wc0 @ t_cam
                    pos[0] += dC[0]; pos[1] += dC[1]
                    used = int(inl.sum())
        pos[2] = r1["h"]
        est.append(pos.copy()); proc.append(i); n_used.append(used)
        prev = cur
        if i % 1200 == 0:
            print(f"  frame {i}/{len(recs)} used~{used}")
    est = np.array(est)
    recs = [recs[k] for k in proc]
    gt = np.array([r["gt"] for r in recs])
    return est, gt, np.array(n_used), recs


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=DEFAULT_DIR)
    ap.add_argument("--scale", type=float, default=0.5)
    ap.add_argument("--max_frames", type=int, default=0)
    ap.add_argument("--min_track", type=int, default=30)
    ap.add_argument("--depth", choices=["baro", "agl"], default="agl")
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--no_fb", action="store_true")
    ap.add_argument("--attitude", choices=["gt", "ahrs"], default="gt")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    agl_arr = None
    cache = os.path.join(args.dir, "agl_cache.npz")
    if args.depth == "agl" and os.path.exists(cache):
        agl_arr = np.load(cache)["agl"]
        print(f"  loaded cached AGL ({len(agl_arr)} frames) from {cache}")

    att = None
    if args.attitude == "ahrs":
        _, _, recs0 = load_dataset(args.dir)
        att = compute_ahrs_attitude(args.dir, recs0)
        print("  using Mahony AHRS attitude (no GT)")

    out = args.out or os.path.join(args.dir,
                                   f"rad_vio_{args.depth}_{args.attitude}_s{args.stride}.png")
    est, gt, n_used, recs = run(args.dir, args.scale, args.max_frames, args.min_track,
                                args.depth, args.stride, not args.no_fb,
                                agl_arr=agl_arr, attitude_R=att)
    m = evaluate_and_plot(est, gt, recs, out, args.depth,
                          f"RaD-VIO (homography + rangefinder) vs GT — Isaac Sim nadir "
                          f"[{args.depth}, {args.attitude}, stride {args.stride}]",
                          traj_label="RaD-VIO (homography + rangefinder)")
    res = os.path.splitext(out)[0] + "_metrics.json"
    json.dump({"paper": "RaD-VIO (homography + rangefinder), arXiv:1810.08704",
               "config": vars(args), "metrics": m}, open(res, "w"), indent=2)
    print(f"saved {res}")
