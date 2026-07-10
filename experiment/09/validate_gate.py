#!/usr/bin/env python3
"""Calibrate & validate the canopy/repetitiveness gate against experiment/08's
164 already-matched samples (experiment/08/sweep_combined.csv) -- no new
SIFT+LightGlue calls. For each row, rebuilds the exact color ortho search
window (same dataset/frame/GT-position recipe as sweep_matchability.py),
computes green_dominance + repetitiveness, and reports how well each signal
predicts the already-known cleared_min_inliers==False outcome.

Run (drone env):
  conda run -n drone python experiment/09/validate_gate.py
"""
import csv, json, math, os, sys

import cv2
import numpy as np

HERE  = os.path.dirname(os.path.abspath(__file__))
EXP08 = os.path.join(os.path.dirname(HERE), "08")
sys.path.insert(0, EXP08)
sys.path.insert(0, HERE)
import flow_odometry as fo
import pipeline as pl

IN_DIR    = os.path.join(os.path.dirname(os.path.dirname(HERE)), "_in")
SWEEP_CSV = os.path.join(EXP08, "sweep_combined.csv")
WIN       = 420


def build_color_ortho(D):
    pl._ensure_geo_georef(D)
    geo = list(csv.DictReader(open(os.path.join(D, "geo.csv"))))
    lat_c = (min(float(r["lat_deg"]) for r in geo) + max(float(r["lat_deg"]) for r in geo)) / 2
    lon_c = (min(float(r["lon_deg"]) for r in geo) + max(float(r["lon_deg"]) for r in geo)) / 2
    zoom = pl._probe_zoom(lat_c, lon_c, 19)
    ortho, meta = pl.build_ortho(geo, zoom, 0.0016, os.path.join(D, "ortho_tiles"))
    g = json.load(open(os.path.join(D, "georef.json")))
    lat0, lon0 = g["origin"]["latitude"], g["origin"]["longitude"]
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat0))
    nN = 2 ** meta["z"]

    def enu_to_px(E, Nn):
        lat = lat0 + Nn / mlat
        lon = lon0 + E / mlon
        gx = (lon + 180) / 360 * nN
        gy = (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * nN
        return (gx - meta["xa"]) * 256, (gy - meta["ya"]) * 256

    return ortho, meta, enu_to_px


def eval_thresh(signal, thresh, failed, cleared):
    flagged = signal >= thresh
    tp = int(np.sum(flagged & failed))
    fp = int(np.sum(flagged & cleared))
    fn = int(np.sum(~flagged & failed))
    tn = int(np.sum(~flagged & cleared))
    precision = tp / max(tp + fp, 1)
    recall = tp / max(tp + fn, 1)
    return dict(thresh=round(float(thresh), 2), tp=tp, fp=fp, fn=fn, tn=tn,
                precision=round(precision, 3), recall=round(recall, 3),
                flagged_frac=round(float(flagged.mean()), 3))


def main():
    rows = list(csv.DictReader(open(SWEEP_CSV)))
    ds_list = []
    for r in rows:
        if r["dataset"] not in ds_list:
            ds_list.append(r["dataset"])

    results = []
    for ds in ds_list:
        D = os.path.join(IN_DIR, ds)
        print(f"=== {ds} ===")
        _, _, recs = fo.load_dataset(D)
        gt_by_frame = {r["frame"]: r["gt"][:2] for r in recs}
        ortho, meta, enu_to_px = build_color_ortho(D)

        for r in [r for r in rows if r["dataset"] == ds]:
            frame = int(r["frame"])
            if frame not in gt_by_frame:
                continue
            gt_xy = gt_by_frame[frame]
            cx, cy = enu_to_px(gt_xy[0], gt_xy[1])
            x0 = int(np.clip(cx - WIN, 0, meta["W"] - 2 * WIN))
            y0 = int(np.clip(cy - WIN, 0, meta["H"] - 2 * WIN))
            window_bgr = ortho[y0:y0 + 2 * WIN, x0:x0 + 2 * WIN]
            if window_bgr.shape[0] < 50 or window_bgr.shape[1] < 50:
                continue
            gd = pl.green_dominance(window_bgr)
            window_gray = cv2.cvtColor(window_bgr, cv2.COLOR_BGR2GRAY)
            rep = pl.repetitiveness(window_gray)
            results.append(dict(
                dataset=ds, frame=frame, green_dominance=gd, repetitiveness=rep,
                cleared=(r["cleared_min_inliers"] == "True"), inliers=int(r["inliers"])))

    out_csv = os.path.join(HERE, "gate_signals.csv")
    with open(out_csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["dataset", "frame", "green_dominance",
                                            "repetitiveness", "cleared", "inliers"])
        w.writeheader()
        w.writerows(results)
    print(f"\nsaved {out_csv} ({len(results)} rows)")

    gd = np.array([r["green_dominance"] for r in results])
    rep = np.array([r["repetitiveness"] for r in results])
    cleared = np.array([r["cleared"] for r in results])
    failed = ~cleared
    print(f"\n{cleared.sum()} cleared / {failed.sum()} failed  (n={len(results)})")

    print("\n--- green_dominance threshold sweep ---")
    for t in np.percentile(gd, [50, 60, 70, 75, 80, 85, 90, 95, 99]):
        print(eval_thresh(gd, t, failed, cleared))

    print("\n--- repetitiveness threshold sweep ---")
    for t in np.percentile(rep, [50, 60, 70, 75, 80, 85, 90, 95, 99]):
        print(eval_thresh(rep, t, failed, cleared))

    print("\n--- per-dataset false-positive check (dataset 1 is the regression risk: "
          "it has real successes to lose) ---")
    for ds in ds_list:
        idxs = [i for i, r in enumerate(results) if r["dataset"] == ds]
        n_cleared_here = int(cleared[idxs].sum())
        print(f"  {ds:<28} n={len(idxs):3d}  cleared={n_cleared_here}")


if __name__ == "__main__":
    main()
