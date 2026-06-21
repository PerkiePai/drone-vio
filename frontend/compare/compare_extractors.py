"""Compare feature EXTRACTORS (SuperPoint / SIFT / DISK / ALIKED) under a
common matcher (LightGlue). LightGlue ships weights trained per-extractor, so
this isolates the front-end: same images, same matcher, only the detector +
descriptor change.

For each extractor we report keypoints, matches, RANSAC inliers + ratio,
approximate rotation, and -- the point of the exercise -- extraction time vs
matching time separately.
"""
import os
import sys
import time
import csv
import math
import argparse

import numpy as np
import cv2
import torch

from lightglue import LightGlue, SuperPoint, DISK, ALIKED, SIFT
from lightglue.utils import rbd

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
IN_DIR = os.path.join(ROOT, "_in")
OUT_DIR = os.path.join(HERE, "_out")

W, H = 640, 480
MAX_KPTS = 1024
WARMUP, REPEAT = 3, 5

EXTRACTORS = {
    "SuperPoint": lambda d: SuperPoint(max_num_keypoints=MAX_KPTS).eval().to(d),
    "SIFT":       lambda d: SIFT(max_num_keypoints=MAX_KPTS).eval().to(d),
    "DISK":       lambda d: DISK(max_num_keypoints=MAX_KPTS).eval().to(d),
    "ALIKED":     lambda d: ALIKED(max_num_keypoints=MAX_KPTS).eval().to(d),
}
FEATURES = {"SuperPoint": "superpoint", "SIFT": "sift",
            "DISK": "disk", "ALIKED": "aliked"}


def load_rgb(path):
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None:
        sys.exit(f"Cannot read {path}")
    img = cv2.resize(img, (W, H))
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0


def timed(fn):
    for _ in range(WARMUP):
        fn()
    if torch.cuda.is_available():
        torch.cuda.synchronize()
    ts, res = [], None
    for _ in range(REPEAT):
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        t = time.perf_counter()
        res = fn()
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        ts.append((time.perf_counter() - t) * 1000.0)
    return res, float(np.mean(ts))


def geometry(mkpts0, mkpts1):
    n = len(mkpts0)
    out = {"inliers": 0, "inlier_ratio": 0.0, "rot_deg": float("nan")}
    if n < 8:
        return out
    _, mask = cv2.findFundamentalMat(mkpts0, mkpts1, cv2.FM_RANSAC, 1.0, 0.999)
    if mask is not None:
        inl = int(mask.sum())
        out["inliers"] = inl
        out["inlier_ratio"] = inl / n
    f = float(W)
    K = np.array([[f, 0, W / 2.0], [0, f, H / 2.0], [0, 0, 1.0]])
    E, _ = cv2.findEssentialMat(mkpts0, mkpts1, K, method=cv2.RANSAC,
                                prob=0.999, threshold=1.0)
    if E is not None and E.shape == (3, 3):
        _, R, _, _ = cv2.recoverPose(E, mkpts0, mkpts1, K)
        out["rot_deg"] = math.degrees(
            math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))
    return out


def run(extractor, matcher, im0, im1, device):
    def extract():
        return extractor.extract(im0, resize=None), \
               extractor.extract(im1, resize=None)
    (f0, f1), ext_ms = timed(extract)
    with torch.no_grad():
        m01, match_ms = timed(lambda: matcher({"image0": f0, "image1": f1}))
    f0r, f1r, m01r = [rbd(x) for x in [f0, f1, m01]]
    kpts0, kpts1 = f0r["keypoints"], f1r["keypoints"]
    matches = m01r["matches"]
    mkpts0 = kpts0[matches[..., 0]].cpu().numpy()
    mkpts1 = kpts1[matches[..., 1]].cpu().numpy()
    return mkpts0, mkpts1, len(kpts0), len(kpts1), ext_ms, match_ms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stem", default="bev-forest")
    ap.add_argument("--ref", type=int, default=0)
    ap.add_argument("--gaps", type=int, nargs="+", default=[1, 3, 6, 12])
    args = ap.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}   matcher: LightGlue   max_kpts: {MAX_KPTS}   "
          f"input: {W}x{H}\n")

    # Build each extractor + its matching LightGlue once.
    models = {}
    for name, build in EXTRACTORS.items():
        models[name] = (build(device),
                        LightGlue(features=FEATURES[name]).eval().to(device))

    ref_img = load_rgb(os.path.join(IN_DIR, f"{args.stem}_{args.ref:04d}s.jpg")).to(device)

    rows = []
    header = (f"{'pair':>10} {'extractor':>11} {'kpts':>9} {'match':>6} "
              f"{'inl':>5} {'inl%':>6} {'rot°':>6} {'extMs':>7} {'mchMs':>7}")
    print(header)
    print("-" * len(header))
    for gap in args.gaps:
        m = args.ref + gap
        p1 = os.path.join(IN_DIR, f"{args.stem}_{m:04d}s.jpg")
        if not os.path.exists(p1):
            print(f"  (skip gap {gap}: missing {os.path.basename(p1)})")
            continue
        img1 = load_rgb(p1).to(device)
        pair = f"{args.ref}->{m}"
        for name, (ext, mat) in models.items():
            mk0, mk1, nk0, nk1, ext_ms, mch_ms = run(ext, mat, ref_img, img1, device)
            geo = geometry(mk0, mk1)
            print(f"{pair:>10} {name:>11} {nk0:>4}:{nk1:<4} {len(mk0):>6} "
                  f"{geo['inliers']:>5} {geo['inlier_ratio']*100:>5.1f}% "
                  f"{geo['rot_deg']:>6.1f} {ext_ms:>6.1f} {mch_ms:>6.1f}")
            rows.append({
                "pair": pair, "extractor": name, "kpts0": nk0, "kpts1": nk1,
                "matches": len(mk0), "inliers": geo["inliers"],
                "inlier_ratio": round(geo["inlier_ratio"], 4),
                "rot_deg": round(geo["rot_deg"], 2),
                "extract_ms": round(ext_ms, 2), "match_ms": round(mch_ms, 2),
            })
        print()

    os.makedirs(OUT_DIR, exist_ok=True)
    csv_path = os.path.join(OUT_DIR, f"extractors_{args.stem}.csv")
    with open(csv_path, "w", newline="") as f:
        wtr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wtr.writeheader()
        wtr.writerows(rows)
    print(f"\nAll extractors paired with LightGlue (features-matched weights).")
    print(f"Saved: {csv_path}")


if __name__ == "__main__":
    main()
