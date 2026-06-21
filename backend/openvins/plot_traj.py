#!/usr/bin/env python3
# Plot an OpenVINS estimated trajectory vs ground truth (SE3-aligned, no scale).
# Usage: python plot_traj.py <est.txt> <gt.txt> <out.png> [title]
# Files: rows of "timestamp tx ty tz qx qy qz qw ..." (ov_eval / pose_to_file format).
import sys, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

est_p, gt_p, out_p = sys.argv[1], sys.argv[2], sys.argv[3]
title = sys.argv[4] if len(sys.argv) > 4 else "OpenVINS vs ground truth"

est = np.loadtxt(est_p); gt = np.loadtxt(gt_p)
# associate gt to each est pose by nearest timestamp
gi = np.searchsorted(gt[:, 0], est[:, 0]); gi = np.clip(gi, 1, len(gt) - 1)
left = np.abs(gt[gi - 1, 0] - est[:, 0]) < np.abs(gt[gi, 0] - est[:, 0])
gi[left] -= 1
ok = np.abs(gt[gi, 0] - est[:, 0]) < 0.05   # within 50 ms
E = est[ok, 1:4]; G = gt[gi[ok], 1:4]

# Umeyama (rotation + translation only, metric)
mE, mG = E.mean(0), G.mean(0)
H = (E - mE).T @ (G - mG)
U, _, Vt = np.linalg.svd(H)
d = np.sign(np.linalg.det(Vt.T @ U.T))
R = Vt.T @ np.diag([1, 1, d]) @ U.T
Ea = (R @ (E - mE).T).T + mG
ate = np.sqrt(((Ea - G) ** 2).sum(1).mean())

fig, ax = plt.subplots(1, 2, figsize=(11, 5))
ax[0].plot(G[:, 0], G[:, 1], "k-", lw=2, label="ground truth")
ax[0].plot(Ea[:, 0], Ea[:, 1], "r-", lw=1, label="OpenVINS (mono)")
ax[0].scatter([G[0, 0]], [G[0, 1]], c="g", s=40, zorder=5, label="start")
ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]"); ax[0].axis("equal")
ax[0].grid(alpha=0.3); ax[0].legend(); ax[0].set_title(f"top-down (x-y)  ATE={ate*100:.1f} cm")
t = est[ok, 0] - est[ok, 0][0]
ax[1].plot(t, G[:, 2], "k-", lw=2, label="gt z")
ax[1].plot(t, Ea[:, 2], "r-", lw=1, label="est z")
ax[1].set_xlabel("time [s]"); ax[1].set_ylabel("z [m]"); ax[1].grid(alpha=0.3); ax[1].legend()
ax[1].set_title("height vs time")
fig.suptitle(title); fig.tight_layout()
fig.savefig(out_p, dpi=120)
print(f"ATE(pos) = {ate*100:.2f} cm over {ok.sum()} associated poses -> {out_p}")
