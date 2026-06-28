#!/usr/bin/env python3
"""Reproduction of the *Downfacing VIO* PX4FLOW core on our nadir Isaac-Sim flight.

Paper: "Efficient and Accurate Downfacing Visual Inertial Odometry"
       (Rutishauser et al., arXiv:2509.10021, IEEE 2025).

What that paper does (the part we reproduce)
--------------------------------------------
A downward monocular camera + IMU + **rangefinder** for metric scale, optimised for
nano-UAVs.  Its accuracy backbone is the classic **PX4FLOW** model: assume the ground
is a single flat plane directly below the camera at distance Z (the rangefinder
reading), gyro-de-rotate the image flow, take ONE robust *average* flow vector for the
whole frame, and convert it to a metric velocity:

    v_xy = -(mean_derotated_flow / dt) * Z              (Z = AGL from rangefinder)

i.e. a **rigid planar-motion model** with a single global flow and a single uniform
depth.  This is deliberately simpler than our flow-odom, which solves a per-point
ray--ground least-squares with a *per-point* depth.  The paper's headline tracker
finding is that PX4FLOW matches ORB accuracy in feature-rich scenes for moderate
pixel speeds -- exactly the front-end-invariance we saw (frontend/CLAUDE.md, Finding 2).

What is reproduced vs adapted
-----------------------------
* Reproduced: the PX4FLOW estimation core (global average de-rotated flow, uniform
  nadir depth, metric scale from the rangefinder), gyro de-rotation, robust averaging.
* Adapted to our rig (identical to our flow-odom harness so the comparison is fair):
  LK pyramidal tracking as the flow source, AGL (rangefinder stand-in, the same
  `compute_true_agl`/`agl_cache.npz` our flow-odom uses), and attitude from GT or the
  Mahony AHRS.  Nothing here uses GT *position*.

Run:
  conda run -n cv python frontend/downfacing-vio/downfacing_vio.py \
      --dir _in/isaac-sim-20260624_2337 --scale 0.5 --depth agl --stride 5
"""
import argparse
import os
import sys

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# reuse the validated data / AGL / attitude / metric infrastructure from flow-odom
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "flow-odom"))
from flow_odometry import (load_dataset, compute_true_agl, compute_ahrs_attitude)  # noqa: E402

DEFAULT_DIR = "/home/innovation/pai/drone-vio/_in/isaac-sim-20260624_2337"


def kitti_segment_drift(gt_xy, est_xy, lengths=(50, 100, 200, 400, 800)):
    """KITTI-style relative translation drift (%) averaged over fixed path-length
    segments -- the honest dead-reckoner metric (endpoint ATE is luck-prone; see
    frontend/CLAUDE.md Finding 2). Translation-only (trajectories start aligned)."""
    d = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(gt_xy, axis=0), axis=1))])
    errs = []
    for L in lengths:
        for i in range(len(gt_xy)):
            j = np.searchsorted(d, d[i] + L)
            if j >= len(gt_xy):
                break
            gseg = gt_xy[j] - gt_xy[i]
            eseg = est_xy[j] - est_xy[i]
            errs.append(np.linalg.norm(eseg - gseg) / L)
    return 100.0 * float(np.mean(errs)) if errs else float("nan")


def run(d, scale, max_frames, min_track, depth_source, stride, fb_check,
        agl_arr=None, attitude_R=None):
    K, R_CtoI, recs = load_dataset(d)
    if max_frames:
        recs = recs[:max_frames]
    if attitude_R is not None:
        for i in range(len(recs)):
            recs[i]["R_wb"] = attitude_R[i]
    Ks = K.copy(); Ks[:2, :] *= scale
    Kinv = np.linalg.inv(Ks)

    agl = None
    if depth_source == "agl":
        agl = agl_arr if agl_arr is not None else compute_true_agl(recs, K, R_CtoI, scale)
        print(f"  AGL (rangefinder stand-in): median {np.median(agl):.0f} m, "
              f"range [{agl.min():.0f}, {agl.max():.0f}] m")

    lk = dict(winSize=(21, 21), maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    feat = dict(maxCorners=600, qualityLevel=0.01, minDistance=8, blockSize=7)

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
                R_wc0 = r0["R_wb"] @ R_CtoI
                R_wc1 = r1["R_wb"] @ R_CtoI
                R_c1c0 = R_wc1.T @ R_wc0           # gyro de-rotation (attitude-derived)
                Z = max(agl[i - 1] if agl is not None else r0["h"], 0.3)
                # --- PX4FLOW core: single global average of the de-rotated flow ---
                dus, dvs = [], []
                for (u0, v0), (u1, v1) in zip(p0g, p1g):
                    n0 = Kinv @ np.array([u0, v0, 1.0])
                    n1 = Kinv @ np.array([u1, v1, 1.0])
                    pr = R_c1c0 @ n0                # where the ray goes under rotation alone
                    if pr[2] <= 1e-6:
                        continue
                    pr = pr / pr[2]
                    dus.append(n1[0] - pr[0]); dvs.append(n1[1] - pr[1])
                if len(dus) >= min_track:
                    # robust single flow vector (PX4FLOW uses a histogram peak / robust
                    # average; the median is the standard robust stand-in)
                    mdu, mdv = float(np.median(dus)), float(np.median(dvs))
                    # uniform-depth nadir model: flow du = -tx/Z  ->  tx = -Z*du
                    t_cam = np.array([-Z * mdu, -Z * mdv, 0.0])
                    dC = R_wc0 @ t_cam
                    pos[0] += dC[0]; pos[1] += dC[1]
                    used = len(dus)
        pos[2] = r1["h"]
        est.append(pos.copy()); proc.append(i); n_used.append(used)
        prev = cur
        if i % 1200 == 0:
            print(f"  frame {i}/{len(recs)} used~{used}")
    est = np.array(est)
    recs = [recs[k] for k in proc]
    gt = np.array([r["gt"] for r in recs])
    return est, gt, np.array(n_used), recs


def evaluate_and_plot(est, gt, recs, out, depth_source, title, traj_label="estimate"):
    t = np.array([(r["ts"] - recs[0]["ts"]) / 1e9 for r in recs])
    err = np.linalg.norm(est[:, :2] - gt[:, :2], axis=1)
    path_len = float(np.sum(np.linalg.norm(np.diff(gt[:, :2], axis=0), axis=1)))
    est_len = float(np.sum(np.linalg.norm(np.diff(est[:, :2], axis=0), axis=1)))
    rpe = kitti_segment_drift(gt[:, :2], est[:, :2])
    m = dict(final_m=float(err[-1]), final_pct=100 * err[-1] / path_len,
             mean_m=float(err.mean()), mean_pct=100 * err.mean() / path_len,
             rmse_m=float(np.sqrt((err ** 2).mean())), max_m=float(err.max()),
             scale=est_len / path_len, rpe_pct=rpe, path_len_m=path_len, frames=len(recs))

    fig, ax = plt.subplots(1, 2, figsize=(14, 6))
    a = ax[0]
    a.plot(gt[:, 0], gt[:, 1], "b-", lw=1.5, label="GT")
    a.plot(est[:, 0], est[:, 1], "r-", lw=1.2, label=traj_label)
    a.plot(gt[0, 0], gt[0, 1], "go", ms=8); a.plot(est[-1, 0], est[-1, 1], "r*", ms=12)
    a.set_title("Top-down trajectory (ENU)"); a.set_xlabel("East (m)"); a.set_ylabel("North (m)")
    a.axis("equal"); a.legend(); a.grid(alpha=0.3)
    a = ax[1]
    a.plot(t, err, "r-")
    a.set_title(f"Horizontal error  (final {m['final_m']:.1f} m = {m['final_pct']:.2f}%, "
                f"RPE {rpe:.2f}%)")
    a.set_xlabel("time (s)"); a.set_ylabel("error (m)"); a.grid(alpha=0.3)
    txt = (f"depth source   {depth_source}\n"
           f"path len GT    {path_len:.0f} m\n"
           f"scale est/GT   {m['scale']:.3f}\n"
           f"final          {m['final_m']:.1f} m ({m['final_pct']:.2f}%)\n"
           f"mean           {m['mean_m']:.1f} m ({m['mean_pct']:.2f}%)\n"
           f"RMSE           {m['rmse_m']:.1f} m\n"
           f"KITTI RPE      {rpe:.2f}%")
    a.text(0.03, 0.97, txt, transform=a.transAxes, va="top", fontsize=9.5,
           fontfamily="monospace",
           bbox=dict(boxstyle="round,pad=0.5", facecolor="#f7f7f7", edgecolor="#aaa"))
    fig.suptitle(title, fontsize=13)
    fig.tight_layout(); fig.savefig(out, dpi=110)
    print(f"\nsaved {out}")
    for k, v in m.items():
        print(f"  {k:12} {v:.3f}" if isinstance(v, float) else f"  {k:12} {v}")
    return m


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=DEFAULT_DIR)
    ap.add_argument("--scale", type=float, default=0.5)
    ap.add_argument("--max_frames", type=int, default=0)
    ap.add_argument("--min_track", type=int, default=30)
    ap.add_argument("--depth", choices=["baro", "agl"], default="agl")
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--no_fb", action="store_true")
    ap.add_argument("--attitude", choices=["gt", "ahrs"], default="gt",
                    help="gt = GT quaternions (good-AHRS stand-in); ahrs = Mahony gyro+accel, no GT")
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
                                   f"downfacing_vio_{args.depth}_{args.attitude}_s{args.stride}.png")
    est, gt, n_used, recs = run(args.dir, args.scale, args.max_frames, args.min_track,
                                args.depth, args.stride, not args.no_fb,
                                agl_arr=agl_arr, attitude_R=att)
    m = evaluate_and_plot(est, gt, recs, out, args.depth,
                          f"Downfacing-VIO (PX4FLOW core) vs GT — Isaac Sim nadir "
                          f"[{args.depth}, {args.attitude}, stride {args.stride}]",
                          traj_label="Downfacing-VIO (PX4FLOW core)")
    import json
    res = os.path.splitext(out)[0] + "_metrics.json"
    json.dump({"paper": "Downfacing VIO (PX4FLOW core), arXiv:2509.10021",
               "config": vars(args), "metrics": m}, open(res, "w"), indent=2)
    print(f"saved {res}")
