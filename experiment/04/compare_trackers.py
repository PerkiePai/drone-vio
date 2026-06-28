#!/usr/bin/env python3
"""Compare all six trackers on accuracy AND latency in a single run.

Runs each tracker with the standard best-config settings
  (--depth agl --stride 5 --attitude ahrs_compass)
then produces:
  - Terminal summary table (RMSE | fused RMSE | final error | inliers | latency | margin)
  - compare_trajectories.png   — all six trajectories + GT overlaid
  - compare_latency.png        — stacked bar: detect / track / solve per tracker

Optional DSMAC fusion: pass --reject N (e.g. 150) to fuse each tracker's
flow-odom trajectory with satellite-ortho DSMAC fixes.

Usage:
  # basic (no fusion)
  conda run -n drone python experiment/04/compare_trackers.py \\
      --dir _in/isaac-sim-20260624_2337

  # with DSMAC fusion
  conda run -n drone python experiment/04/compare_trackers.py \\
      --dir _in/isaac-sim-20260625 \\
      --depth agl --attitude ahrs_compass \\
      --skip_below 13 --reject 150

Output goes to experiment/04/<dataset-name>/compare_*.png
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# import from sibling script
EXP04_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, EXP04_DIR)
from flow_odometry import (
    run, load_dataset, compute_ahrs_attitude, compute_true_agl,
    TRACKERS, make_farneback, make_dis, make_sparse_raft,
    make_orb, detect_points, track_lk, track_dense, _orb_match,
    _raft_flow, LK_PARAMS, FEAT_PARAMS, FB_THRESH, FRAME_BUDGET_MS,
)

# DSMAC imports from experiment/02 (read-only, not modified)
EXP02_DIR = os.path.join(os.path.dirname(EXP04_DIR), "02")
sys.path.insert(0, EXP02_DIR)

COLORS = {
    "lk":          "#1f77b4",
    "fast_lk":     "#ff7f0e",
    "farneback":   "#2ca02c",
    "dis":         "#d62728",
    "orb":         "#9467bd",
    "sparse_raft": "#8c564b",
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


def _build_dsmac(d, K):
    """Load ortho + geo for DSMAC.  Returns (orthog, meta, helpers, lg)."""
    from compare_tracking import LG
    from dsmac_match import build_ortho, warp_north_up as _warp

    geo = list(csv.DictReader(open(os.path.join(d, "geo.csv"))))
    ortho, meta = build_ortho(geo, 19, 0.0016, os.path.join(d, "ortho_tiles"))
    orthog = cv2.cvtColor(ortho, cv2.COLOR_BGR2GRAY)
    n = 2 ** meta["z"]
    g = json.load(open(os.path.join(d, "georef.json")))
    lat0, lon0 = g["origin"]["latitude"], g["origin"]["longitude"]
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat0))
    GSD = 156543.03392 * math.cos(math.radians(lat0)) / n
    fx = K[0, 0]

    def enu_to_px(E, N):
        lat = lat0 + N / mlat
        lon = lon0 + E / mlon
        gx = (lon + 180) / 360 * n
        gy = (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n
        return (gx - meta["xa"]) * 256, (gy - meta["ya"]) * 256

    def px_to_enu(px, py):
        gx = meta["xa"] + px / 256
        gy = meta["ya"] + py / 256
        lon = gx / n * 360 - 180
        lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * gy / n))))
        return (lon - lon0) * mlon, (lat - lat0) * mlat

    lg = LG()
    return orthog, meta, enu_to_px, px_to_enu, _warp, fx, GSD, lg


def _fuse(est_xy, gt_xy, recs, agl_fuse, dsmac_ctx,
          reject, skip_below, fix_every=30, win=420, min_inliers=15, blend=0.8):
    """Apply DSMAC fixes to a flow-odom trajectory.

    est_xy  : (N,2) flow-odom positions (already skip_below-trimmed + strided)
    recs    : list of N records aligned with est_xy (from run())
    agl_fuse: (N,) AGL values aligned with est_xy
    Returns (fused_xy, fixes_list)
    """
    orthog, meta, enu_to_px, px_to_enu, warp_north_up, fx, GSD, lg = dsmac_ctx

    inc = np.vstack([[0, 0], np.diff(est_xy, axis=0)])
    pos = est_xy[0].copy()
    drift_since = 0.0
    fused = [pos.copy()]
    fixes = []

    for k in range(1, len(est_xy)):
        pos = pos + inc[k]
        drift_since += float(np.linalg.norm(inc[k]))

        if k % fix_every == 0 and drift_since >= skip_below:
            r = recs[k]
            im = cv2.imread(r["img"], cv2.IMREAD_GRAYSCALE)
            yaw = math.degrees(math.atan2(r["R_wb"][1, 0], r["R_wb"][0, 0]))
            h = float(agl_fuse[k])
            f = (h / fx) / GSD
            cx, cy = enu_to_px(pos[0], pos[1])
            x0 = max(0, int(cx - win))
            y0 = max(0, int(cy - win))
            patch = orthog[y0:y0 + 2*win, x0:x0 + 2*win]
            if patch.shape[0] < 50 or patch.shape[1] < 50:
                fused.append(pos.copy())
                continue
            q, cpt = warp_north_up(im, yaw, f)
            try:
                k0, k1, _ = lg.match(lg.extract(q), lg.extract(patch))
                if len(k0) < 8:
                    raise ValueError("too few matches")
                Hm, mask = cv2.findHomography(k0, k1, cv2.RANSAC, 5.0)
                if Hm is None or mask is None or int(mask.sum()) < min_inliers:
                    raise ValueError("homography failed")
                p = Hm @ np.array([cpt[0], cpt[1], 1.0]); p /= p[2]
                eE, eN = px_to_enu(x0 + p[0], y0 + p[1])
                d = math.hypot(eE - pos[0], eN - pos[1])
                acc = d <= reject
                if acc:
                    pos = np.array([pos[0] + blend*(eE - pos[0]),
                                    pos[1] + blend*(eN - pos[1])])
                    drift_since = 0.0
                fixes.append((k, eE, eN, acc, d, int(mask.sum())))
            except Exception:
                pass

        fused.append(pos.copy())

    return np.array(fused), fixes


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
                    help="frames for latency benchmark (default 200)")
    ap.add_argument("--skip_below", type=float, default=0.0,
                    help="skip initial frames where baro height < this (m)")
    ap.add_argument("--reject",   type=float, default=0.0,
                    help="DSMAC fix rejection threshold (m); 0 = no fusion")
    ap.add_argument("--fix_every", type=int, default=30,
                    help="attempt a DSMAC fix every N flow-odom steps (default 30)")
    ap.add_argument("--win",      type=int, default=420,
                    help="DSMAC ortho search half-window (px, default 420)")
    args = ap.parse_args()

    EXP_DIR  = EXP04_DIR
    OUT_DIR  = os.path.join(EXP_DIR, os.path.basename(args.dir))
    os.makedirs(OUT_DIR, exist_ok=True)

    do_fuse = args.reject > 0

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

    # DSMAC setup (only if fusion requested)
    dsmac_ctx = None
    if do_fuse:
        print("  loading DSMAC ortho…")
        dsmac_ctx = _build_dsmac(args.dir, K)
        print("  DSMAC ready")

    # skip_below offset (used to align agl for fusion)
    skip_n = 0
    if args.skip_below > 0.0:
        skip_n = next((i for i, r in enumerate(recs_full)
                       if r["h"] >= args.skip_below), 0)

    # ── Exp1: accuracy (+ optional fusion) ───────────────────────────────────
    skip_tag = f"  skip_below={args.skip_below:.0f} m" if args.skip_below > 0 else ""
    fuse_tag = f"  reject={args.reject:.0f} m" if do_fuse else ""
    print("\n" + "="*70)
    print(f"  Exp1: ACCURACY — {args.attitude} / {args.depth} / stride {args.stride}{skip_tag}{fuse_tag}")
    print("="*70)

    results = {}
    for tk in args.trackers:
        print(f"\n  [{tk}]")
        est, gt, n_used, recs = run(
            args.dir, args.scale, 0, 30,
            args.depth, args.stride,
            agl_arr=agl, attitude_R=att_R,
            tracker=tk, benchmark=False,
            skip_below=args.skip_below,
        )
        err = np.linalg.norm(est[:, :2] - gt[:, :2], axis=1)
        path_len = np.sum(np.linalg.norm(np.diff(gt[:, :2], axis=0), axis=1))
        tvec = np.array([(r["ts"] - recs[0]["ts"]) / 1e9 for r in recs])

        fused_xy = None
        fused_err = None
        fused_rmse = None
        nacc = 0
        n_fixes = 0
        if do_fuse:
            # AGL aligned with returned recs: original index = skip_n + k*stride
            agl_fuse = agl[skip_n::args.stride][:len(est)]
            fused_xy, fixes = _fuse(
                est[:, :2], gt[:, :2], recs, agl_fuse, dsmac_ctx,
                reject=args.reject, skip_below=args.skip_below,
                fix_every=args.fix_every, win=args.win,
            )
            fused_err = np.linalg.norm(fused_xy - gt[:, :2], axis=1)
            fused_rmse = float(np.sqrt((fused_err**2).mean()))
            nacc   = sum(1 for f in fixes if f[3])
            n_fixes = len(fixes)
            print(f"    flow-odom RMSE {np.sqrt((err**2).mean()):.1f} m  "
                  f"→ fused RMSE {fused_rmse:.1f} m  "
                  f"fixes {nacc}/{n_fixes}")
        else:
            print(f"    RMSE {np.sqrt((err**2).mean()):.1f} m  "
                  f"final {err[-1]:.1f} m  inliers {np.mean(n_used):.0f}")

        results[tk] = dict(
            est=est, gt=gt, err=err, n_used=n_used,
            recs=recs, tvec=tvec, path_len=path_len,
            rmse=float(np.sqrt((err**2).mean())),
            final=float(err[-1]),
            mean_pts=float(np.mean(n_used)),
            fused_xy=fused_xy, fused_err=fused_err,
            fused_rmse=fused_rmse, nacc=nacc, n_fixes=n_fixes,
        )

    # ── Exp2: latency ─────────────────────────────────────────────────────────
    print("\n" + "="*70)
    print(f"  Exp2: LATENCY — {args.latency_frames} frames after skip")
    print("="*70)

    recs_sub = recs_full[skip_n:skip_n + args.latency_frames]
    if att_R is not None:
        for i in range(len(recs_sub)):
            recs_sub[i]["R_wb"] = att_R[skip_n + i]
    agl_sub = agl[skip_n:skip_n + args.latency_frames] if agl is not None else None

    for tk in args.trackers:
        print(f"\n  [{tk}]", end=" ", flush=True)
        det, trk, slv = _latency_one_tracker(
            tk, recs_sub, args.scale, agl_sub, Kinv, R_CtoI)
        total = det + trk + slv
        results[tk]["lat_det"]   = det
        results[tk]["lat_trk"]   = trk
        results[tk]["lat_slv"]   = slv
        results[tk]["lat_total"] = total
        print(f"detect {det:.1f} ms  track {trk:.1f} ms  solve {slv:.1f} ms  "
              f"→ total {total:.1f} ms  margin {FRAME_BUDGET_MS-total:.1f} ms")

    # ── summary table ─────────────────────────────────────────────────────────
    print("\n" + "="*70)
    print(f"  SUMMARY — {args.attitude} / {args.depth} / stride {args.stride}{fuse_tag}")
    print("="*70)
    if do_fuse:
        hdr = (f"  {'Tracker':<14} {'FO-RMSE':>9} {'Fused':>9} {'Final':>8} "
               f"{'Fixes':>7} {'Det':>7} {'Trk':>7} {'Slv':>7} {'Total':>8} {'Margin':>8}")
    else:
        hdr = (f"  {'Tracker':<14} {'RMSE':>8} {'Final':>8} {'Inlrs':>7} "
               f"{'Det':>7} {'Trk':>7} {'Slv':>7} {'Total':>8} {'Margin':>8}")
    print(hdr)
    print("  " + "─" * (len(hdr)-2))
    for tk in args.trackers:
        r = results[tk]
        if do_fuse:
            fstr = f"{r['fused_rmse']:>8.1f}m" if r["fused_rmse"] is not None else "       N/A"
            fix_str = f"{r['nacc']}/{r['n_fixes']}"
            print(f"  {tk:<14} {r['rmse']:>8.1f}m {fstr} {r['final']:>7.1f}m "
                  f"{fix_str:>7} "
                  f"{r['lat_det']:>6.1f}ms {r['lat_trk']:>6.1f}ms {r['lat_slv']:>6.1f}ms "
                  f"{r['lat_total']:>7.1f}ms {FRAME_BUDGET_MS-r['lat_total']:>7.1f}ms")
        else:
            print(f"  {tk:<14} {r['rmse']:>7.1f}m {r['final']:>7.1f}m "
                  f"{r['mean_pts']:>6.0f} "
                  f"{r['lat_det']:>6.1f}ms {r['lat_trk']:>6.1f}ms {r['lat_slv']:>6.1f}ms "
                  f"{r['lat_total']:>7.1f}ms {FRAME_BUDGET_MS-r['lat_total']:>7.1f}ms")

    # ── trajectory plot ────────────────────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    ax = axes[0]
    gt_ref = results[args.trackers[0]]["gt"]
    ax.plot(gt_ref[:, 0], gt_ref[:, 1], "k-", lw=2, label="GT", zorder=10)
    ax.plot(gt_ref[0, 0], gt_ref[0, 1], "ko", ms=8, zorder=11)
    ax.plot(gt_ref[-1, 0], gt_ref[-1, 1], "ks", ms=8, zorder=11)
    for tk in args.trackers:
        est = results[tk]["est"]
        ax.plot(est[:, 0], est[:, 1], "-", color=COLORS[tk], lw=1.0,
                alpha=0.5, label=f"{tk} (flow)")
        if do_fuse and results[tk]["fused_xy"] is not None:
            fx_ = results[tk]["fused_xy"]
            ax.plot(fx_[:, 0], fx_[:, 1], "--", color=COLORS[tk], lw=1.4,
                    alpha=0.9, label=f"{tk} (fused)")
        ax.plot(est[-1, 0], est[-1, 1], "*", color=COLORS[tk], ms=8)
    ax.set_title("Top-down trajectories (ENU) — all trackers")
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.axis("equal"); ax.legend(fontsize=7); ax.grid(alpha=0.3)

    ax = axes[1]
    for tk in args.trackers:
        tvec = results[tk]["tvec"]
        err  = results[tk]["err"]
        ax.plot(tvec, err, "-", color=COLORS[tk], lw=1.0, alpha=0.5,
                label=f"{tk} RMSE={results[tk]['rmse']:.0f}m")
        if do_fuse and results[tk]["fused_err"] is not None:
            ax.plot(tvec, results[tk]["fused_err"], "--", color=COLORS[tk], lw=1.4,
                    alpha=0.9, label=f"{tk} fused={results[tk]['fused_rmse']:.0f}m")
    ax.set_title("Horizontal error vs GT")
    ax.set_xlabel("time (s)"); ax.set_ylabel("error (m)")
    ax.legend(fontsize=7); ax.grid(alpha=0.3)

    title = f"Tracker comparison — {args.attitude} / {args.depth} / stride {args.stride}"
    if do_fuse:
        title += f" / DSMAC reject={args.reject:.0f} m"
    fig.suptitle(title, fontsize=13)
    fig.tight_layout()
    traj_out = os.path.join(OUT_DIR, "compare_trajectories.png")
    fig.savefig(traj_out, dpi=110); plt.close(fig)
    print(f"\nsaved {traj_out}")

    # ── latency bar chart ──────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(10, 5))
    x     = np.arange(len(args.trackers))
    width = 0.5
    dets  = [results[tk]["lat_det"] for tk in args.trackers]
    trks  = [results[tk]["lat_trk"] for tk in args.trackers]
    slvs  = [results[tk]["lat_slv"] for tk in args.trackers]
    ax.bar(x, dets,               width=width, label="detect",      color="#4C72B0")
    ax.bar(x, trks, bottom=dets,  width=width, label="track/match", color="#DD8452")
    ax.bar(x, slvs,
           bottom=[d+t for d, t in zip(dets, trks)], width=width,
           label="solve", color="#55A868")
    ax.axhline(FRAME_BUDGET_MS, color="red", ls="--", lw=1.5,
               label=f"budget ({FRAME_BUDGET_MS:.0f} ms)")
    ax.set_xticks(x); ax.set_xticklabels(args.trackers, fontsize=10)
    ax.set_ylabel("mean latency (ms)")
    ax.set_title("Per-frame latency breakdown by sub-step")
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
