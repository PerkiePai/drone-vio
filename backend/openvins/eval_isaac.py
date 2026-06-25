#!/usr/bin/env python3
"""Compare an OpenVINS Isaac-Sim trajectory against GT (poses.csv).

Reports the Umeyama-fit SCALE factor (the key monocular-VIO metric: ~1.0 means
the IMU locked metric scale) plus rigid and Sim3 ATE, and plots est vs GT.
"""
import argparse
import csv
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

D = "/home/innovation/pai/drone-vio/_in/isaac-sim-20260623"


def load_est(p):
    ts, xyz = [], []
    for ln in open(p):
        if ln.startswith("#"):
            continue
        f = ln.split()
        ts.append(float(f[0])); xyz.append([float(f[1]), float(f[2]), float(f[3])])
    return np.array(ts), np.array(xyz)


def load_gt(d):
    ts = {int(r["frame"]): int(r["ts_ns"]) for r in csv.DictReader(open(os.path.join(d, "imu.csv")))}
    rows = [r for r in csv.DictReader(open(os.path.join(d, "poses.csv"))) if int(r["frame"]) in ts]
    t = np.array([ts[int(r["frame"])] for r in rows]) / 1e9
    xyz = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])
    return t, xyz


def umeyama(src, dst, with_scale=True):
    """Fit dst ~= s*R*src + t. src,dst: (N,3)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s0, d0 = src - mu_s, dst - mu_d
    C = d0.T @ s0 / len(src)
    U, Dv, Vt = np.linalg.svd(C)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    if with_scale:
        var = (s0 ** 2).sum() / len(src)
        s = np.trace(np.diag(Dv) @ S) / var
    else:
        s = 1.0
    t = mu_d - s * R @ mu_s
    return s, R, t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--est", default="/home/innovation/pai/drone-vio/backend/openvins/_out/traj_est_isaac.txt")
    ap.add_argument("--dir", default=D, help="GT dataset dir (imu.csv + poses.csv)")
    ap.add_argument("--out", default="/home/innovation/pai/drone-vio/backend/openvins/_out/isaac_est_vs_gt.png")
    args = ap.parse_args()

    et, ex = load_est(args.est)
    gt, gx = load_gt(args.dir)
    # time-align: nearest GT for each est sample
    idx = np.searchsorted(gt, et).clip(1, len(gt) - 1)
    idx = np.where(np.abs(gt[idx] - et) < np.abs(gt[idx - 1] - et), idx, idx - 1)
    gxa = gx[idx]
    rel = et - et[0]

    s, R, t = umeyama(ex, gxa, with_scale=True)
    est_s = (s * (R @ ex.T).T + t)
    s1, R1, t1 = umeyama(ex, gxa, with_scale=False)
    est_r = (R1 @ ex.T).T + t1

    ate_sim3 = np.sqrt(((est_s - gxa) ** 2).sum(1).mean())
    ate_rigid = np.sqrt(((est_r - gxa) ** 2).sum(1).mean())
    plen = np.linalg.norm(np.diff(gxa, axis=0), axis=1).sum()

    print(f"samples              : {len(et)}  (est {rel[-1]:.0f}s, init at t={et[0]-gt[0]:.1f}s into flight)")
    print(f"Umeyama SCALE factor : {s:.3f}   (1.0 = perfect metric; >1 means est too small)")
    print(f"ATE (Sim3, scaled)   : {ate_sim3:.1f} m")
    print(f"ATE (rigid, no scale): {ate_rigid:.1f} m   on {plen:.0f} m GT path")

    # divergence onset: rigid error over time
    err_r = np.linalg.norm(est_r - gxa, axis=1)

    fig, ax = plt.subplots(1, 3, figsize=(17, 5.2))
    a = ax[0]
    a.plot(gxa[:, 0], gxa[:, 1], "b-", lw=1.5, label="GT")
    a.plot(est_s[:, 0], est_s[:, 1], "r-", lw=1.0, label=f"OV (Sim3-aligned, s={s:.2f})")
    a.plot(gxa[0, 0], gxa[0, 1], "go"); a.axis("equal"); a.legend(); a.grid(alpha=0.3)
    a.set_title("Top-down (Sim3-aligned to GT)"); a.set_xlabel("E"); a.set_ylabel("N")

    a = ax[1]
    a.semilogy(rel, np.maximum(err_r, 1e-3), "r-")
    a.set_title(f"Rigid-aligned error vs time (ATE {ate_rigid:.0f} m)")
    a.set_xlabel("time since init (s)"); a.set_ylabel("error (m, log)"); a.grid(alpha=0.3)

    a = ax[2]
    a.plot(rel, np.linalg.norm(ex - ex[0], axis=1), "r-", label="OV |pos| (raw)")
    a.plot(rel, np.linalg.norm(gxa - gxa[0], axis=1), "b-", label="GT |pos|")
    a.set_title("Raw distance from start (scale check)")
    a.set_xlabel("time since init (s)"); a.set_ylabel("m"); a.legend(); a.grid(alpha=0.3)

    fig.tight_layout(); fig.savefig(args.out, dpi=110)
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
