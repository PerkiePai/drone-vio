#!/usr/bin/env python3
"""Compare attitude sources for flow-odom on an Isaac Sim dataset.

Runs three modes (GT / Mahony / Mahony+compass) with AGL depth and plots
trajectory + error curves side by side.
"""
import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(__file__))
from flow_odometry import run, compute_ahrs_attitude, compute_true_agl, load_dataset

DEFAULT_DIR = "/home/innovation/pai/drone-vio/_in/isaac-sim-20260625"

MODES = [
    ("GT attitude",          dict(mag_gain=0.0,  mag_noise_deg=0.0),  True,  "b"),
    ("Mahony (gyro+accel)",  dict(mag_gain=0.0,  mag_noise_deg=0.0),  False, "r"),
    ("Mahony + compass",     dict(mag_gain=0.05, mag_noise_deg=5.0),  False, "g"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=DEFAULT_DIR)
    ap.add_argument("--scale", type=float, default=0.5)
    ap.add_argument("--max_frames", type=int, default=0)
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--skip_frames", type=int, default=0)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    out = args.out or os.path.join(args.dir, "attitude_compare.png")

    # precompute AGL once (GT-pose triangulation, same for all attitude modes)
    K, R_CtoI, recs_base = load_dataset(args.dir)
    if args.skip_frames:
        recs_base = recs_base[args.skip_frames:]
    if args.max_frames:
        recs_base = recs_base[:args.max_frames]
    print("Precomputing AGL (shared across all modes)...")
    agl_shared = compute_true_agl(recs_base, K, R_CtoI, args.scale)
    print(f"  AGL: median {np.median(agl_shared):.0f} m, range [{agl_shared.min():.0f}, {agl_shared.max():.0f}] m")

    results = []
    for label, ahrs_kw, use_gt, color in MODES:
        print(f"\n=== {label} ===")
        att_R = None if use_gt else compute_ahrs_attitude(args.dir, recs_base, **ahrs_kw)
        est, gt, n_used, impl_depth, recs = run(
            args.dir, args.scale, args.max_frames, 30, "agl", args.stride, True,
            agl_arr=agl_shared, attitude_R=att_R, skip_frames=args.skip_frames,
        )
        err = np.linalg.norm(est[:, :2] - gt[:, :2], axis=1)
        path_len = np.linalg.norm(np.diff(gt[:, :2], axis=0), axis=1).sum()
        est_len  = np.linalg.norm(np.diff(est[:, :2], axis=0), axis=1).sum()
        t = np.array([(r["ts"] - recs[0]["ts"]) / 1e9 for r in recs])
        print(f"  path GT / est : {path_len:.0f} m / {est_len:.0f} m  (scale {est_len/path_len:.3f})")
        print(f"  final error   : {err[-1]:.1f} m  ({100*err[-1]/path_len:.2f}%)")
        print(f"  mean  error   : {err.mean():.1f} m  ({100*err.mean()/path_len:.2f}%)")
        print(f"  max   error   : {err.max():.1f} m  @{t[np.argmax(err)]:.0f}s")
        print(f"  RMSE          : {np.sqrt((err**2).mean()):.1f} m")
        results.append((label, color, est, gt, err, t, path_len, est_len))

    # ── plot ──────────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    # top-down trajectory
    a = axes[0]
    gt_ref = results[0][3]
    a.plot(gt_ref[:, 0], gt_ref[:, 1], "k-", lw=2, label="GT", zorder=5)
    a.plot(gt_ref[0, 0], gt_ref[0, 1], "go", ms=8, zorder=6)
    for label, color, est, gt, err, t, path_len, est_len in results:
        a.plot(est[:, 0], est[:, 1], "-", color=color, lw=1.0, alpha=0.8, label=label)
    a.axis("equal"); a.legend(fontsize=8); a.grid(alpha=0.3)
    a.set_title("Top-down trajectory (ENU)")
    a.set_xlabel("East (m)"); a.set_ylabel("North (m)")

    # error over time
    a = axes[1]
    for label, color, est, gt, err, t, path_len, est_len in results:
        a.plot(t, err, "-", color=color, lw=1.2,
               label=f"{label}  (final {err[-1]:.0f} m = {100*err[-1]/path_len:.2f}%)")
    a.set_title("Horizontal error vs GT")
    a.set_xlabel("time (s)"); a.set_ylabel("error (m)")
    a.legend(fontsize=8); a.grid(alpha=0.3)

    # stats table
    a = axes[2]
    a.axis("off")
    col_labels = ["Attitude", "Final (m)", "Final (%)", "Mean (m)", "Max (m)", "Scale"]
    rows_data = []
    for label, color, est, gt, err, t, path_len, est_len in results:
        rows_data.append([
            label,
            f"{err[-1]:.1f}",
            f"{100*err[-1]/path_len:.2f}",
            f"{err.mean():.1f}",
            f"{err.max():.1f}",
            f"{est_len/path_len:.3f}",
        ])
    tbl = a.table(
        cellText=rows_data, colLabels=col_labels,
        loc="center", cellLoc="center",
    )
    tbl.auto_set_font_size(False); tbl.set_fontsize(9)
    tbl.scale(1.1, 2.2)
    for (r, c), cell in tbl.get_celld().items():
        if r == 0:
            cell.set_facecolor("#dddddd")
        elif c == 0:
            import matplotlib.colors as mcolors
            rgba = list(mcolors.to_rgba(results[r - 1][1])); rgba[3] = 0.18
            cell.set_facecolor(rgba)
    a.set_title("Summary — AGL depth, stride=5", pad=14)

    fig.suptitle(
        f"Flow-odom attitude source comparison — {os.path.basename(args.dir)}  "
        f"({'full' if not args.max_frames else str(args.max_frames)+' frames'})",
        fontsize=12,
    )
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"\nsaved {out}")


if __name__ == "__main__":
    main()
