#!/usr/bin/env python3
"""FUSED flow-odom + DSMAC navigation (no GT position prior).

The deployed two-layer loop: flow-odom propagates position every step (smooth, but
drifts); periodically DSMAC attempts an absolute fix whose ortho search window is
centred on the *fused estimate itself* (flow-odom's position — NOT ground truth),
and a confident, temporally-consistent fix RESETS the accumulated drift. flow-odom
tells DSMAC where to look; DSMAC clamps flow-odom's drift.

Reuses the cached flow-odom LK trajectory (`tracker_trajs.npz`, stride-5 GT-att
frame-AGL) as per-step increments, so the matcher runs only at fix frames (fast).
The GT crutch REMOVED here is the position prior; attitude is still GT (matching the
cached LK baseline) — a fully-onboard run also swaps attitude->AHRS (+~1.3pp). GT is
used only for scoring. Needs `agl_cache.npz` + `tracker_trajs.npz` (from the RPE /
ablation runs) and downloads/caches the Esri ortho via dsmac_match.build_ortho.

Run in the `drone` (or `cv`) conda env. See frontend/CLAUDE.md → DSMAC / fusion.
"""
import argparse, csv, json, math, os, sys

import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "frontend", "flow-odom"))
sys.path.insert(0, os.path.join(ROOT, "frontend", "openvins-alike-lightglue"))
sys.path.insert(0, HERE)
import flow_odometry as fo
from compare_tracking import LG
from dsmac_match import build_ortho, warp_north_up


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.join(ROOT, "_in/isaac-sim-20260624_2337"))
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--win", type=int, default=420, help="ortho search half-window (px)")
    ap.add_argument("--min_inliers", type=int, default=15)
    ap.add_argument("--fix_every", type=int, default=30, help="DSMAC fix attempt cadence (fused steps)")
    ap.add_argument("--reject", type=float, default=45.0, help="reject a fix > this far from the prior (m)")
    ap.add_argument("--blend", type=float, default=0.8, help="fixed pull toward fix; ignored if --conf_blend")
    ap.add_argument("--conf_blend", action="store_true",
                    help="weight each fix by inlier count (and gate by prior agreement) instead of fixed --blend")
    ap.add_argument("--skip_below", type=float, default=0.0,
                    help="only apply a fix once flow-odom prior uncertainty exceeds this (m); 0 = always")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    D = args.dir
    out = args.out or os.path.join(D, "fused_flowodom_dsmac.png")

    geo = list(csv.DictReader(open(os.path.join(D, "geo.csv"))))
    ortho, meta = build_ortho(geo, 19, 0.0016, os.path.join(D, "ortho_tiles"))
    orthog = cv2.cvtColor(ortho, cv2.COLOR_BGR2GRAY)
    n = 2 ** meta["z"]
    K, R_CtoI, recs = fo.load_dataset(D)
    fx = K[0, 0]; N = len(recs)
    agl = np.load(os.path.join(D, "agl_cache.npz"))["agl"]
    g = json.load(open(os.path.join(D, "georef.json")))
    lat0, lon0 = g["origin"]["latitude"], g["origin"]["longitude"]
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat0))
    GSD = 156543.03392 * math.cos(math.radians(lat0)) / n

    def enu_to_px(E, Nn):
        lat, lon = lat0 + Nn / mlat, lon0 + E / mlon
        gx = (lon + 180) / 360 * n
        gy = (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n
        return (gx - meta["xa"]) * 256, (gy - meta["ya"]) * 256

    def px_to_enu(px, py):
        gx, gy = meta["xa"] + px / 256, meta["ya"] + py / 256
        lon = gx / n * 360 - 180
        lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * gy / n))))
        return (lon - lon0) * mlon, (lat - lat0) * mlat

    z = np.load(os.path.join(D, "tracker_trajs.npz"), allow_pickle=True)
    LK, GTtr, tvec = z["LK"], z["GT"], z["t_vec"]
    proc = list(range(0, N, args.stride))[:len(LK)]
    inc = np.vstack([[0, 0], np.diff(LK, axis=0)])

    lg = LG()

    def dsmac_fix(rec_i, prior):
        r = recs[rec_i]
        im = cv2.imread(r["img"], cv2.IMREAD_GRAYSCALE)
        yaw = math.degrees(math.atan2(r["R_wb"][1, 0], r["R_wb"][0, 0]))
        f = (agl[rec_i] / fx) / GSD
        cx, cy = enu_to_px(prior[0], prior[1])
        x0, y0 = max(0, int(cx - args.win)), max(0, int(cy - args.win))
        win = orthog[y0:y0 + 2 * args.win, x0:x0 + 2 * args.win]
        if win.shape[0] < 50 or win.shape[1] < 50:
            return None
        q, cpt = warp_north_up(im, yaw, f)
        try:
            k0, k1, _ = lg.match(lg.extract(q), lg.extract(win))
            if len(k0) < 8:
                return None
            Hm, mask = cv2.findHomography(k0, k1, cv2.RANSAC, 5.0)
            if Hm is None or mask is None or int(mask.sum()) < args.min_inliers:
                return None
            p = Hm @ np.array([cpt[0], cpt[1], 1.0]); p /= p[2]
            eE, eN = px_to_enu(x0 + p[0], y0 + p[1])
            return eE, eN, int(mask.sum())
        except Exception:
            return None

    # running prior-uncertainty proxy: distance drifted since last accepted fix
    pos = LK[0].copy()
    last_fix_pos = pos.copy()
    drift_since = 0.0
    fused = [pos.copy()]
    fixes = []
    for k in range(1, len(LK)):
        pos = pos + inc[k]
        drift_since += float(np.linalg.norm(inc[k]))
        if k % args.fix_every == 0 and drift_since >= args.skip_below:
            out_fix = dsmac_fix(proc[k], pos)
            if out_fix is not None:
                eE, eN, inl = out_fix
                d = math.hypot(eE - pos[0], eN - pos[1])
                acc = d <= args.reject
                if acc:
                    if args.conf_blend:
                        # inlier-confidence weight: ~0.45 at the gate, ->~0.9 for strong fixes
                        w = min(0.9, inl / (inl + 18.0))
                    else:
                        w = args.blend
                    pos = np.array([pos[0] + w * (eE - pos[0]), pos[1] + w * (eN - pos[1])])
                    last_fix_pos = pos.copy(); drift_since = 0.0
                fixes.append((k, eE, eN, acc, d, inl))
        fused.append(pos.copy())
    fused = np.array(fused)

    fused_err = np.linalg.norm(fused - GTtr, axis=1)
    fo_err = np.linalg.norm(LK - GTtr, axis=1)
    rmse = lambda e: float(np.sqrt(np.mean(e ** 2)))
    nacc = sum(1 for f in fixes if f[3])
    print(f"fused steps {len(fused)}  DSMAC attempts {len(fixes)} accepted {nacc} rejected {len(fixes)-nacc}")
    print(f"flow-odom only : RMSE {rmse(fo_err):.1f} m  final {fo_err[-1]:.1f} m  (drifts)")
    print(f"FUSED          : RMSE {rmse(fused_err):.1f} m  final {fused_err[-1]:.1f} m  (bounded)"
          f"  [{'conf-weighted' if args.conf_blend else f'blend {args.blend}'}, "
          f"fix/{args.fix_every}, skip<{args.skip_below}m]")

    fig, ax = plt.subplots(2, 1, figsize=(13, 9), sharex=True)
    a = ax[0]
    a.plot(tvec, GTtr[:, 0], "k-", lw=1, label="GT East")
    a.plot(tvec, LK[:, 0], "g-", lw=0.8, alpha=0.7, label="flow-odom East")
    a.plot(tvec, fused[:, 0], "b-", lw=1.2, label="FUSED East")
    for f in fixes:
        if f[3]:
            a.plot(tvec[f[0]], f[1], "r.", ms=7)
    a.plot([], [], "r.", label="accepted DSMAC fix")
    a.set_ylabel("East (m)")
    a.set_title("Position over time (East) — fused uses flow-odom prior, no GT")
    a.grid(alpha=0.3); a.legend(fontsize=8)
    a = ax[1]
    a.plot(tvec, fo_err, "g-", lw=1.2, label=f"flow-odom only (RMSE {rmse(fo_err):.1f} m, drifts)")
    a.plot(tvec, fused_err, "b-", lw=1.4, label=f"FUSED (RMSE {rmse(fused_err):.1f} m, bounded)")
    for f in fixes:
        if f[3]:
            a.axvline(tvec[f[0]], color="r", lw=0.4, alpha=0.25)
        else:
            a.plot(tvec[f[0]], args.reject, "rx", ms=6)
    a.set_ylabel("position error (m)"); a.set_xlabel("time (s)")
    a.set_title("Position error over time — DSMAC fixes (red) clamp flow-odom drift")
    a.grid(alpha=0.3); a.legend(fontsize=9)
    fig.suptitle("FUSED flow-odom + DSMAC (no GT prior) — Isaac nadir, Bangkok", fontsize=13)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
