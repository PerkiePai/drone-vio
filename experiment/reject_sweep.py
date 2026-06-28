#!/usr/bin/env python3
"""Sweep --reject threshold vs flight length and find the optimal reject per length.

Three panels:
  1. Fused RMSE vs reject threshold (both datasets).
  2. DSMAC fix acceptance rate vs reject threshold (both datasets).
  3. Best reject (RMSE-minimising) vs cumulative flight distance (long dataset).

Strategy (fast, ~2-3 min):
  For each dataset, run DSMAC once with large reject so positions stay near truth
  → cache (k, eE, eN) for all successful fixes → sweep reject analytically.
  Approximation valid while position error << search window radius (~138 m).
"""
import csv, json, math, os, sys

import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
EXP03 = os.path.join(HERE, "03")
sys.path.insert(0, EXP03)
import flow_odometry as fo
from compare_tracking import LG
from dsmac_match import build_ortho, warp_north_up

DATASETS = [
    ("Short  1.77 km · 7.5 min",
     os.path.join(ROOT, "_in/isaac-sim-20260624_2337"),
     "steelblue"),
    ("Long   22.4 km · 77 min",
     os.path.join(ROOT, "_in/isaac-sim-20260625"),
     "tomato"),
]

REJECT_VALUES = [5, 10, 15, 20, 30, 45, 60, 80, 100, 150, 200, 300]
STRIDE = 5
FIX_EVERY = 30
SKIP_BELOW = 13.0
BLEND = 0.8
WIN = 420
MIN_INLIERS = 15
ATTITUDE = "ahrs_compass"
DEPTH = "agl"


def collect_fixes(D, lg):
    """DSMAC baseline pass (large reject). Returns LK, GTtr, tvec, inc, fixes."""
    K_cam, R_CtoI, recs = fo.load_dataset(D)
    fx = K_cam[0, 0]
    N = len(recs)
    agl = np.load(os.path.join(D, "agl_cache.npz"))["agl"]
    geo = list(csv.DictReader(open(os.path.join(D, "geo.csv"))))
    ortho, meta = build_ortho(geo, 19, 0.0016, os.path.join(D, "ortho_tiles"))
    orthog = cv2.cvtColor(ortho, cv2.COLOR_BGR2GRAY)
    n = 2 ** meta["z"]
    g = json.load(open(os.path.join(D, "georef.json")))
    lat0, lon0 = g["origin"]["latitude"], g["origin"]["longitude"]
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat0))
    GSD = 156543.03392 * math.cos(math.radians(lat0)) / n

    def enu_to_px(E, Nn):
        lat = lat0 + Nn / mlat; lon = lon0 + E / mlon
        gx = (lon + 180) / 360 * n
        gy = (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n
        return (gx - meta["xa"]) * 256, (gy - meta["ya"]) * 256

    def px_to_enu(px, py):
        gx = meta["xa"] + px / 256; gy = meta["ya"] + py / 256
        lon = gx / n * 360 - 180
        lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * gy / n))))
        return (lon - lon0) * mlon, (lat - lat0) * mlat

    traj_file = os.path.join(D, f"tracker_trajs_{ATTITUDE}_{DEPTH}.npz")
    z = np.load(traj_file, allow_pickle=True)
    LK, GTtr, tvec = z["LK"], z["GT"], z["t_vec"]
    proc = list(range(0, N, STRIDE))[:len(LK)]
    inc = np.vstack([[0, 0], np.diff(LK, axis=0)])

    pos = LK[0].copy()
    drift_since = 0.0
    fixes = []  # (k, eE, eN, inliers)

    for k in range(1, len(LK)):
        pos = pos + inc[k]
        drift_since += float(np.linalg.norm(inc[k]))
        if k % FIX_EVERY == 0 and drift_since >= SKIP_BELOW:
            r = recs[proc[k]]
            im = cv2.imread(r["img"], cv2.IMREAD_GRAYSCALE)
            if im is None:
                continue
            yaw = math.degrees(math.atan2(r["R_wb"][1, 0], r["R_wb"][0, 0]))
            f = (agl[proc[k]] / fx) / GSD
            cx, cy = enu_to_px(pos[0], pos[1])
            x0 = max(0, int(cx - WIN)); y0 = max(0, int(cy - WIN))
            win = orthog[y0:y0 + 2 * WIN, x0:x0 + 2 * WIN]
            if win.shape[0] < 50 or win.shape[1] < 50:
                continue
            q, cpt = warp_north_up(im, yaw, f)
            try:
                k0, k1, _ = lg.match(lg.extract(q), lg.extract(win))
                if len(k0) < 8:
                    continue
                Hm, mask = cv2.findHomography(k0, k1, cv2.RANSAC, 5.0)
                if Hm is None or mask is None or int(mask.sum()) < MIN_INLIERS:
                    continue
                p = Hm @ np.array([cpt[0], cpt[1], 1.0]); p /= p[2]
                eE, eN = px_to_enu(x0 + p[0], y0 + p[1])
                fixes.append((k, eE, eN, int(mask.sum())))
                pos = np.array([pos[0] + BLEND * (eE - pos[0]),
                                pos[1] + BLEND * (eN - pos[1])])
                drift_since = 0.0
            except Exception:
                continue

    return LK, GTtr, tvec, inc, fixes


def replay(LK, GTtr, inc, fixes, reject, max_step=None):
    """Replay trajectory up to max_step with a given reject threshold.
    Returns (fused_array, n_accepted, rmse)."""
    steps = len(LK) if max_step is None else min(max_step, len(LK))
    pos = LK[0].copy()
    drift_since = 0.0
    fused = [pos.copy()]
    fix_map = {f[0]: f for f in fixes}
    n_accepted = 0
    for k in range(1, steps):
        pos = pos + inc[k]
        drift_since += float(np.linalg.norm(inc[k]))
        if k in fix_map:
            _, eE, eN, _ = fix_map[k]
            d = math.hypot(eE - pos[0], eN - pos[1])
            if d <= reject:
                pos = np.array([pos[0] + BLEND * (eE - pos[0]),
                                pos[1] + BLEND * (eN - pos[1])])
                drift_since = 0.0
                n_accepted += 1
        fused.append(pos.copy())
    fused = np.array(fused)
    err = np.linalg.norm(fused - GTtr[:steps], axis=1)
    rmse = float(np.sqrt(np.mean(err ** 2)))
    return fused, n_accepted, rmse


def main():
    lg = LG()
    out = os.path.join(HERE, "reject_sweep.png")

    # ── collect data ─────────────────────────────────────────────────────────
    results = {}
    for label, D, color in DATASETS:
        print(f"\n=== {label} ===")
        print("  Running DSMAC baseline pass…", flush=True)
        LK, GTtr, tvec, inc, fixes = collect_fixes(D, lg)
        print(f"  Fixes cached: {len(fixes)}")

        fo_err = np.linalg.norm(LK - GTtr, axis=1)
        fo_rmse = float(np.sqrt(np.mean(fo_err ** 2)))
        path_km = float(np.sum(np.linalg.norm(np.diff(GTtr, axis=0), axis=1)) / 1000)

        rmses, accs = [], []
        for rej in REJECT_VALUES:
            _, n_acc, rmse = replay(LK, GTtr, inc, fixes, rej)
            rmses.append(rmse)
            accs.append(100.0 * n_acc / max(1, len(fixes)))
            print(f"  reject={rej:5.0f}m  RMSE={rmse:6.1f}m  accept={accs[-1]:.0f}%")

        results[label] = dict(
            LK=LK, GTtr=GTtr, inc=inc, fixes=fixes,
            fo_rmse=fo_rmse, path_km=path_km,
            rmses=rmses, accs=accs, color=color,
        )

    # ── best reject vs cumulative flight distance (long dataset) ──────────
    long_label = DATASETS[1][0]
    r = results[long_label]
    LK, GTtr, inc, fixes = r["LK"], r["GTtr"], r["inc"], r["fixes"]

    # cumulative GT distance at each step
    gt_dist_km = np.concatenate([[0], np.cumsum(
        np.linalg.norm(np.diff(GTtr, axis=0), axis=1))]) / 1000

    # evaluate at 12 length cutoffs spread across the full flight
    cutoff_km = np.linspace(0.5, r["path_km"], 15)
    best_rejects, best_rmses = [], []
    for target_km in cutoff_km:
        max_step = int(np.searchsorted(gt_dist_km, target_km))
        if max_step < 30:
            continue
        rms_per_rej = []
        for rej in REJECT_VALUES:
            _, _, rmse = replay(LK, GTtr, inc, fixes, rej, max_step=max_step)
            rms_per_rej.append(rmse)
        best_i = int(np.argmin(rms_per_rej))
        best_rejects.append(REJECT_VALUES[best_i])
        best_rmses.append(rms_per_rej[best_i])
        print(f"  @{target_km:.1f} km  best_reject={REJECT_VALUES[best_i]}m  RMSE={rms_per_rej[best_i]:.1f}m")
    cutoff_km_used = [km for km in cutoff_km if int(np.searchsorted(gt_dist_km, km)) >= 30]

    # ── plot ─────────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(3, 1, figsize=(11, 13))
    fig.suptitle("Reject threshold analysis  (flow-odom + DSMAC, AHRS+compass+AGL)",
                 fontsize=13, fontweight="bold")

    # Panel 1: RMSE vs reject
    ax = axes[0]
    for label, _, _ in DATASETS:
        d = results[label]
        ax.plot(REJECT_VALUES, d["rmses"], "o-", color=d["color"], lw=1.8, ms=5,
                label=f"{label}")
        ax.axhline(d["fo_rmse"], color=d["color"], lw=0.8, ls=":", alpha=0.6)
    ax.axvline(45, color="gray", lw=0.9, ls="--", alpha=0.7, label="baseline rej=45 m")
    ax.set_ylabel("Fused RMSE (m)")
    ax.set_title("RMSE vs reject threshold  (dotted = flow-odom only, no fixes)")
    ax.legend(fontsize=9); ax.grid(True, lw=0.4, alpha=0.5); ax.set_ylim(bottom=0)

    # Panel 2: acceptance rate vs reject
    ax = axes[1]
    for label, _, _ in DATASETS:
        d = results[label]
        ax.plot(REJECT_VALUES, d["accs"], "s--", color=d["color"], lw=1.4, ms=5,
                label=f"{label}")
    ax.axvline(45, color="gray", lw=0.9, ls="--", alpha=0.7)
    ax.set_ylabel("Fix acceptance rate (%)")
    ax.set_title("DSMAC fix acceptance rate vs reject threshold")
    ax.set_ylim(0, 105); ax.legend(fontsize=9); ax.grid(True, lw=0.4, alpha=0.5)

    # Panel 3: best reject vs cumulative flight distance (long dataset)
    ax = axes[2]
    ax.plot(cutoff_km_used, best_rejects, "o-", color="tomato", lw=2, ms=7,
            label="best reject (min RMSE) — long dataset")
    ax2 = ax.twinx()
    ax2.plot(cutoff_km_used, best_rmses, "^--", color="gray", lw=1.2, ms=5,
             alpha=0.75, label="RMSE at best reject")
    ax2.set_ylabel("Best-reject RMSE (m)", color="gray")
    ax.axvline(r["path_km"] * 0.5, color="steelblue", lw=0.8, ls=":", alpha=0.6)
    ax.text(r["path_km"] * 0.5 + 0.1, max(best_rejects) * 0.95,
            "½ flight", fontsize=8, color="steelblue")
    ax.set_xlabel("Cumulative flight distance (km)")
    ax.set_ylabel("Optimal reject threshold (m)", color="tomato")
    ax.set_title("Best reject threshold as a function of flight length  (long dataset)")
    lines1, lab1 = ax.get_legend_handles_labels()
    lines2, lab2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, lab1 + lab2, fontsize=9)
    ax.grid(True, lw=0.4, alpha=0.5)

    plt.tight_layout()
    plt.savefig(out, dpi=150)
    print(f"\nSaved: {out}")


if __name__ == "__main__":
    main()
