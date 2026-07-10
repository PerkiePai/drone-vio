#!/usr/bin/env python3
"""Experiment 08, Experiment 1 — cross-flight / within-flight matchability sweep.

For each of the 6 new-format datasets, sample frames across the flight and run
the exact DSMAC query-building + SIFT+LightGlue matching + RANSAC-homography
steps that `pipeline.py:_dsmac_fix` uses (imported from the snapshot in this
folder, not reimplemented), but centred on GT position instead of the drifted
estimate (isolates matchability from prior drift-error, per Q4 in plan.md).
Records match/inlier counts, keypoint counts, ortho-texture score, raw-image
blur score, gyro magnitude, AGL, and Esri zoom per sample.

Dataset 1 (isaac-sim-20260630_152940) gets extra dense sampling over its last
third (frame 18000+) to resolve the within-flight fade noted in plan.md.

Run (drone env, per project rule):
  conda run -n drone python experiment/08/sweep_matchability.py
"""
import argparse, csv, math, os, sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import flow_odometry as fo
import pipeline as pl

DATASETS = [
    "isaac-sim-20260630_152940",
    "isaac-sim-20260704_205334",
    "isaac-sim-20260704_193743",
    "isaac-sim-20260705_230815",
    "isaac-sim-20260706_105804",
    "isaac-sim-20260705_220937",
]
DENSE_DATASET = "isaac-sim-20260630_152940"
DENSE_FRAME_START = 18000

FIELDS = [
    "dataset", "frame", "sample_idx", "agl_m", "zoom", "gyro_mag",
    "blur_score", "texture_score", "kp_query", "kp_window", "matches",
    "inliers", "cleared_min_inliers", "note",
]


def sample_indices(lo, N, n_base, dense_start=None, n_dense=0):
    base = np.linspace(lo, N - 1, n_base).round().astype(int)
    if dense_start is not None and dense_start < N - 1:
        dense = np.linspace(max(dense_start, lo), N - 1, n_dense).round().astype(int)
        base = np.concatenate([base, dense])
    return np.unique(base)


def process_dataset(D, args):
    ds = os.path.basename(D)
    print(f"\n=== {ds} ===")
    K, R_CtoI, recs = fo.load_dataset(D)
    N = len(recs)
    fx = K[0, 0]
    print(f"  {N} frames loaded")

    # AGL: real lidar interpolated onto camera frame numbers (same as run_pipeline)
    lidar_rows = list(csv.DictReader(open(os.path.join(D, "lidar.csv"))))
    lidar_fi = np.array([int(r["frame"]) for r in lidar_rows])
    lidar_agl = np.array([float(r["agl_m"]) for r in lidar_rows])
    order = np.argsort(lidar_fi)
    lidar_fi, lidar_agl = lidar_fi[order], lidar_agl[order]
    frame_idx = np.array([r["frame"] for r in recs])
    agl = np.interp(frame_idx, lidar_fi, lidar_agl)

    # attitude: AHRS + magnetometer compass (pipeline.py default: mag_gain=1.0)
    print("  computing AHRS+compass attitude ...")
    att_R = fo.compute_ahrs_attitude(D, recs, Kp=1.0, mag_gain=1.0)
    for i in range(N):
        recs[i]["R_wb"] = att_R[i]

    # gyro magnitude, interpolated onto each rec's timestamp (Q8 cross-reference)
    imu_rows = list(csv.DictReader(open(os.path.join(D, "imu.csv"))))
    imu_ts = np.array([int(r["ts_ns"]) for r in imu_rows]) / 1e9
    gyro = np.array([[float(r["wx"]), float(r["wy"]), float(r["wz"])] for r in imu_rows])
    gyro_mag = np.linalg.norm(gyro, axis=1)
    rec_ts = np.array([r["ts"] for r in recs]) / 1e9
    gyro_at_rec = np.interp(rec_ts, imu_ts, gyro_mag)

    # geo/ortho (reuse pipeline.py helpers verbatim; geo.csv/ortho_tiles are
    # already cached from the 6 prior full pipeline.py runs)
    pl._ensure_geo_georef(D)
    with open(os.path.join(D, "geo.csv")) as fh:
        geo = list(csv.DictReader(fh))
    lat_c = (min(float(r["lat_deg"]) for r in geo) + max(float(r["lat_deg"]) for r in geo)) / 2
    lon_c = (min(float(r["lon_deg"]) for r in geo) + max(float(r["lon_deg"]) for r in geo)) / 2
    zoom = pl._probe_zoom(lat_c, lon_c, 19)
    if zoom != 19:
        print(f"  Esri z19 not covered here — using z{zoom}")
    ortho, meta = pl.build_ortho(geo, zoom, 0.0016, os.path.join(D, "ortho_tiles"))
    orthog = cv2.cvtColor(ortho, cv2.COLOR_BGR2GRAY)
    nN = 2 ** meta["z"]
    import json
    with open(os.path.join(D, "georef.json")) as fh:
        g = json.load(fh)
    lat0, lon0 = g["origin"]["latitude"], g["origin"]["longitude"]
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat0))
    GSD = 156543.03392 * math.cos(math.radians(lat0)) / nN

    def enu_to_px(E, Nn):
        lat = lat0 + Nn / mlat
        lon = lon0 + E / mlon
        gx = (lon + 180) / 360 * nN
        gy = (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * nN
        return (gx - meta["xa"]) * 256, (gy - meta["ya"]) * 256

    lg = pl.make_sift_lg()

    # Exclude the pre-climb launch segment (AGL still near-ground): real
    # pipeline.py never attempts a DSMAC fix this early anyway (fix_every /
    # skip_below drift gating), and near-zero AGL degenerates warp_north_up's
    # scale factor. All 6 datasets climb past min_agl_m within ~6% of the flight.
    ok = np.nonzero(agl >= args.min_agl_m)[0]
    lo = int(ok[0]) if len(ok) else 0
    print(f"  flight sampling starts at frame {recs[lo]['frame']} "
          f"(AGL >= {args.min_agl_m} m)")

    dense_start = DENSE_FRAME_START if ds == DENSE_DATASET else None
    n_dense = args.n_dense if dense_start is not None else 0
    idx = sample_indices(lo, N, args.n_base, dense_start, n_dense)
    print(f"  sampling {len(idx)} frames"
          + (f" (dense from frame {dense_start})" if dense_start is not None else ""))

    rows = []
    save_at = {idx[0], idx[len(idx) // 2], idx[-1]}
    for i in idx:
        r = recs[int(i)]
        im = cv2.imread(r["img"], cv2.IMREAD_GRAYSCALE)
        yaw = math.degrees(math.atan2(r["R_wb"][1, 0], r["R_wb"][0, 0]))
        f = (agl[i] / fx) / GSD
        gt_xy = r["gt"][:2]
        cx, cy = enu_to_px(gt_xy[0], gt_xy[1])
        x0 = int(np.clip(cx - args.win, 0, meta["W"] - 2 * args.win))
        y0 = int(np.clip(cy - args.win, 0, meta["H"] - 2 * args.win))
        wimg = orthog[y0:y0 + 2 * args.win, x0:x0 + 2 * args.win]
        blur_score = float(cv2.Laplacian(im, cv2.CV_64F).var())

        row = dict(dataset=ds, frame=int(r["frame"]), sample_idx=int(i),
                   agl_m=float(agl[i]), zoom=zoom, gyro_mag=float(gyro_at_rec[i]),
                   blur_score=blur_score, texture_score=float("nan"),
                   kp_query=0, kp_window=0, matches=0, inliers=0,
                   cleared_min_inliers=False, note="")

        if wimg.shape[0] < 50 or wimg.shape[1] < 50:
            row["note"] = "window_too_small"
            rows.append(row)
            continue

        texture_score = float(cv2.Laplacian(wimg, cv2.CV_64F).var())
        row["texture_score"] = texture_score

        q, cpt = pl.warp_north_up(im, yaw, f)
        try:
            f0 = lg.extract(q)
            f1 = lg.extract(wimg)
            k0, k1 = lg.match(f0, f1)
            row["kp_query"] = int(f0["keypoints"].shape[1])
            row["kp_window"] = int(f1["keypoints"].shape[1])
            row["matches"] = int(len(k0))
            if len(k0) >= 8:
                Hm, mask = cv2.findHomography(k0, k1, cv2.RANSAC, 5.0)
                if Hm is not None and mask is not None:
                    row["inliers"] = int(mask.sum())
            row["cleared_min_inliers"] = row["inliers"] >= args.min_inliers
        except Exception as e:
            row["note"] = f"error:{e}"

        rows.append(row)

        if int(i) in save_at:
            cv2.imwrite(os.path.join(HERE, f"{ds}_f{int(r['frame']):06d}_query_warped.png"), q)
            cv2.imwrite(os.path.join(HERE, f"{ds}_f{int(r['frame']):06d}_ortho_window.png"), wimg)

    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--datasets", nargs="+", default=DATASETS)
    ap.add_argument("--n_base", type=int, default=25,
                    help="evenly spaced samples across the full flight (default 25)")
    ap.add_argument("--n_dense", type=int, default=15,
                    help="extra dense samples over dataset 1's last third (default 15)")
    ap.add_argument("--win", type=int, default=420,
                    help="DSMAC ortho search half-window px (default 420, matches pipeline.py)")
    ap.add_argument("--min_inliers", type=int, default=30,
                    help="min RANSAC inliers to count as a clearing fix (default 30)")
    ap.add_argument("--min_agl_m", type=float, default=10.0,
                    help="skip samples before AGL reaches this (excludes pre-climb "
                         "launch segment; default 10 m)")
    ap.add_argument("--in_dir", default=os.path.join(
        os.path.dirname(os.path.dirname(HERE)), "_in"))
    args = ap.parse_args()

    combined = []
    for ds in args.datasets:
        D = os.path.join(args.in_dir, ds)
        rows = process_dataset(D, args)
        out_csv = os.path.join(HERE, f"sweep_{ds}.csv")
        with open(out_csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=FIELDS)
            w.writeheader()
            w.writerows(rows)
        print(f"  saved {out_csv} ({len(rows)} rows)")
        combined.extend(rows)

    out_combined = os.path.join(HERE, "sweep_combined.csv")
    with open(out_combined, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(combined)
    print(f"\nsaved {out_combined} ({len(combined)} rows total)")


if __name__ == "__main__":
    main()
