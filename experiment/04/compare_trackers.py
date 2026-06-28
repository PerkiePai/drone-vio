#!/usr/bin/env python3
"""Compare all six trackers on accuracy AND latency in a single run.

Runs each tracker with the standard best-config settings
  (--depth agl --stride 5 --attitude ahrs_compass)
then produces:
  - Terminal summary table (RMSE | final error | mean inliers | mean latency | margin)
  - combined_trajectories.png   — all six trajectories + GT overlaid
  - combined_errors.png         — horizontal error over time for all six
  - latency_breakdown.png       — stacked bar: detect / track / solve per tracker

Usage:
  conda run -n drone python experiment/04/compare_trackers.py \\
      --dir _in/isaac-sim-20260624_2337

Output goes to experiment/04/<dataset-name>/compare_*.png
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# import from sibling script
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from flow_odometry import (
    run, load_dataset, compute_ahrs_attitude, compute_true_agl,
    TRACKERS, make_farneback, make_dis, make_sparse_raft,
    make_orb, detect_points, track_lk, track_dense, _orb_match,
    _raft_flow, LK_PARAMS, FEAT_PARAMS, FB_THRESH, FRAME_BUDGET_MS,
)

COLORS = {
    "lk":          "#1f77b4",   # blue
    "fast_lk":     "#ff7f0e",   # orange
    "farneback":   "#2ca02c",   # green
    "dis":         "#d62728",   # red
    "orb":         "#9467bd",   # purple
    "sparse_raft": "#8c564b",   # brown
}


def _latency_one_tracker(tracker, recs_sub, scale, agl, Kinv, R_CtoI, min_track=30):
    """Time detect+track+solve for a short subsequence; return (det_ms, trk_ms, slv_ms)."""
    flow_fn = raft_model = raft_device = None
    orb_det = orb_bf = None
    prev_orb_kp = prev_orb_des = None

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

    stride = 5
    prev = load(recs_sub[0]["img"])
    if tracker == "orb":
        prev_orb_kp, prev_orb_des = orb_det.detectAndCompute(prev, None)

    det_ms, trk_ms, slv_ms = [], [], []

    for i in range(stride, len(recs_sub), stride):
        cur = load(recs_sub[i]["img"])
        r0, r1 = recs_sub[i - stride], recs_sub[i]

        if tracker == "orb":
            t0 = time.perf_counter()
            cur_kp, cur_des = orb_det.detectAndCompute(cur, None)
            det_ms.append((time.perf_counter() - t0) * 1000)

            t0 = time.perf_counter()
            p0g, p1g = _orb_match(prev_orb_kp, prev_orb_des, cur_kp, cur_des, orb_bf)
            trk_ms.append((time.perf_counter() - t0) * 1000)
            prev_orb_kp, prev_orb_des = cur_kp, cur_des
        else:
            t0 = time.perf_counter()
            p0 = detect_points(prev, tracker)
            det_ms.append((time.perf_counter() - t0) * 1000)

            t0 = time.perf_counter()
            if p0 is not None and len(p0) >= min_track:
                if tracker in ("lk", "fast_lk"):
                    p0g, p1g = track_lk(prev, cur, p0)
                else:
                    p0g, p1g = track_dense(prev, cur, p0, flow_fn)
            else:
                p0g = p1g = np.empty((0, 2), np.float32)
            trk_ms.append((time.perf_counter() - t0) * 1000)

        # minimal solve (same math as run())
        if len(p0g) >= min_track and agl is not None:
            h0 = max(float(agl[i - 1]), 0.3)
            R_wc0 = r0["R_wb"] @ R_CtoI
            R_wc1 = r1["R_wb"] @ R_CtoI
            R_c1c0 = R_wc1.T @ R_wc0
            t0 = time.perf_counter()
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
                A = np.asarray(A); b_arr = np.asarray(b)
                np.linalg.lstsq(A, b_arr, rcond=None)
            slv_ms.append((time.perf_counter() - t0) * 1000)
        else:
            slv_ms.append(0.0)

        prev = cur

    return (np.mean(det_ms) if det_ms else 0.0,
            np.mean(trk_ms) if trk_ms else 0.0,
            np.mean(slv_ms) if slv_ms else 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir",      default="_in/isaac-sim-20260624_2337")
    ap.add_argument("--scale",    type=float, default=0.5)
    ap.add_argument("--depth",    choices=["baro", "agl"], default="agl")
    ap.add_argument("--stride",   type=int, default=5)
    ap.add_argument("--attitude", choices=["gt", "ahrs", "ahrs_compass"],
                    default="ahrs_compass")
    ap.add_argument("--trackers", nargs="+", default=TRACKERS,
                    choices=TRACKERS, metavar="T",
                    help="subset of trackers to run (default: all six)")
    ap.add_argument("--latency_frames", type=int, default=200,
                    help="number of frames to use for latency benchmarking (default 200)")
    args = ap.parse_args()

    EXP_DIR  = os.path.dirname(os.path.abspath(__file__))
    OUT_DIR  = os.path.join(EXP_DIR, os.path.basename(args.dir))
    os.makedirs(OUT_DIR, exist_ok=True)

    # ── shared setup ──────────────────────────────────────────────────────────
    K, R_CtoI, recs_full = load_dataset(args.dir)
    Ks   = K.copy(); Ks[:2, :] *= args.scale
    Kinv = np.linalg.inv(Ks)

    att_R = None
    if args.attitude == "ahrs":
        att_R = compute_ahrs_attitude(args.dir, recs_full)
        print("  AHRS (gyro+accel only)")
    elif args.attitude == "ahrs_compass":
        att_R = compute_ahrs_attitude(args.dir, recs_full, mag_gain=1.0)
        print("  AHRS + compass")

    # AGL: shared cache
    agl = None
    if args.depth == "agl":
        cache_path = os.path.join(args.dir, "agl_cache.npz")
        if os.path.exists(cache_path):
            agl = np.load(cache_path)["agl"]
            print(f"  loaded AGL cache from {cache_path}")
        else:
            print("  computing AGL (this takes a moment)...")
            agl = compute_true_agl(recs_full, K, R_CtoI, args.scale)
            np.savez(cache_path, agl=agl)
            print(f"  saved AGL cache → {cache_path}")

    # ── Exp1: accuracy — run each tracker ────────────────────────────────────
    print("\n" + "="*70)
    print(f"  Exp1: ACCURACY — {args.attitude} / {args.depth} / stride {args.stride}")
    print("="*70)

    results = {}
    for tk in args.trackers:
        print(f"\n  [{tk}]")
        est, gt, n_used, recs = run(
            args.dir, args.scale, 0, 30,
            args.depth, args.stride,
            agl_arr=agl, attitude_R=att_R,
            tracker=tk, benchmark=False,
        )
        err = np.linalg.norm(est[:, :2] - gt[:, :2], axis=1)
        path_len = np.sum(np.linalg.norm(np.diff(gt[:, :2], axis=0), axis=1))
        tvec = np.array([(r["ts"] - recs[0]["ts"]) / 1e9 for r in recs])
        results[tk] = dict(
            est=est, gt=gt, err=err, n_used=n_used,
            recs=recs, tvec=tvec, path_len=path_len,
            rmse=float(np.sqrt((err**2).mean())),
            final=float(err[-1]),
            mean_pts=float(np.mean(n_used)),
        )
        print(f"    RMSE {results[tk]['rmse']:.1f} m  final {results[tk]['final']:.1f} m  "
              f"inliers {results[tk]['mean_pts']:.0f}")

    # ── Exp2: latency — time a short subsequence per tracker ─────────────────
    print("\n" + "="*70)
    print(f"  Exp2: LATENCY — first {args.latency_frames} frames")
    print("="*70)

    recs_sub = recs_full[:args.latency_frames]
    if att_R is not None:
        for i in range(len(recs_sub)):
            recs_sub[i]["R_wb"] = att_R[i]
    agl_sub = agl[:args.latency_frames] if agl is not None else None

    for tk in args.trackers:
        print(f"\n  [{tk}]", end=" ", flush=True)
        det, trk, slv = _latency_one_tracker(
            tk, recs_sub, args.scale, agl_sub, Kinv, R_CtoI)
        total = det + trk + slv
        results[tk]["lat_det"] = det
        results[tk]["lat_trk"] = trk
        results[tk]["lat_slv"] = slv
        results[tk]["lat_total"] = total
        print(f"detect {det:.1f} ms  track {trk:.1f} ms  solve {slv:.1f} ms  "
              f"→ total {total:.1f} ms  margin {FRAME_BUDGET_MS-total:.1f} ms")

    # ── summary table ─────────────────────────────────────────────────────────
    print("\n" + "="*70)
    print(f"  SUMMARY — {args.attitude} / {args.depth} / stride {args.stride}")
    print("="*70)
    hdr = f"  {'Tracker':<14} {'RMSE':>8} {'Final':>8} {'Inlrs':>7} {'Det':>7} {'Trk':>7} {'Slv':>7} {'Total':>8} {'Margin':>8}"
    print(hdr)
    print("  " + "─" * (len(hdr)-2))
    for tk in args.trackers:
        r = results[tk]
        print(f"  {tk:<14} {r['rmse']:>7.1f}m {r['final']:>7.1f}m "
              f"{r['mean_pts']:>6.0f} "
              f"{r['lat_det']:>6.1f}ms {r['lat_trk']:>6.1f}ms {r['lat_slv']:>6.1f}ms "
              f"{r['lat_total']:>7.1f}ms {FRAME_BUDGET_MS-r['lat_total']:>7.1f}ms")

    # ── combined trajectory plot ───────────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    ax = axes[0]
    first_tk = args.trackers[0]
    gt_ref = results[first_tk]["gt"]
    ax.plot(gt_ref[:, 0], gt_ref[:, 1], "k-", lw=2, label="GT", zorder=10)
    ax.plot(gt_ref[0, 0], gt_ref[0, 1], "ko", ms=8, zorder=11)
    ax.plot(gt_ref[-1, 0], gt_ref[-1, 1], "ks", ms=8, zorder=11)
    for tk in args.trackers:
        est = results[tk]["est"]
        ax.plot(est[:, 0], est[:, 1], "-", color=COLORS[tk], lw=1.2,
                alpha=0.8, label=tk)
        ax.plot(est[-1, 0], est[-1, 1], "*", color=COLORS[tk], ms=10)
    ax.set_title("Top-down trajectories (ENU) — all trackers")
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.axis("equal"); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = axes[1]
    for tk in args.trackers:
        tvec = results[tk]["tvec"]
        err  = results[tk]["err"]
        ax.plot(tvec, err, "-", color=COLORS[tk], lw=1.2, alpha=0.85,
                label=f"{tk}  RMSE={results[tk]['rmse']:.0f}m")
    ax.set_title("Horizontal error vs GT — all trackers")
    ax.set_xlabel("time (s)"); ax.set_ylabel("error (m)")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    fig.suptitle(f"Tracker comparison — {args.attitude} / {args.depth} / stride {args.stride}",
                 fontsize=13)
    fig.tight_layout()
    traj_out = os.path.join(OUT_DIR, "compare_trajectories.png")
    fig.savefig(traj_out, dpi=110); plt.close(fig)
    print(f"\nsaved {traj_out}")

    # ── latency breakdown bar chart ────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(10, 5))
    x     = np.arange(len(args.trackers))
    width = 0.5
    dets  = [results[tk]["lat_det"] for tk in args.trackers]
    trks  = [results[tk]["lat_trk"] for tk in args.trackers]
    slvs  = [results[tk]["lat_slv"] for tk in args.trackers]
    ax.bar(x, dets,               width=width, label="detect",          color="#4C72B0")
    ax.bar(x, trks, bottom=dets,  width=width, label="track/match",     color="#DD8452")
    ax.bar(x, slvs,
           bottom=[d+t for d,t in zip(dets,trks)], width=width,
           label="solve",          color="#55A868")
    ax.axhline(FRAME_BUDGET_MS, color="red", ls="--", lw=1.5, label=f"budget ({FRAME_BUDGET_MS:.0f} ms)")
    ax.set_xticks(x); ax.set_xticklabels(args.trackers, fontsize=10)
    ax.set_ylabel("mean latency (ms)"); ax.set_title("Per-frame latency breakdown by sub-step")
    ax.legend(); ax.grid(axis="y", alpha=0.3)
    for xi, tk in enumerate(args.trackers):
        tot = results[tk]["lat_total"]
        ax.text(xi, tot + 0.5, f"{tot:.1f}", ha="center", va="bottom", fontsize=8)
    fig.tight_layout()
    lat_out = os.path.join(OUT_DIR, "compare_latency.png")
    fig.savefig(lat_out, dpi=110); plt.close(fig)
    print(f"saved {lat_out}")


if __name__ == "__main__":
    main()
