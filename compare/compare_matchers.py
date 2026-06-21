"""Fair head-to-head comparison of SuperGlue vs LightGlue on the same frames.

Both matchers use the SuperPoint front-end. To make it apples-to-apples we
feed BOTH the identical pre-resized grayscale pixels and the same keypoint
budget (max 1024). For each frame pair we report:

  - matcher latency (warmed-up, averaged)   -> speed
  - total matches                            -> quantity
  - RANSAC inliers via fundamental matrix    -> quality (no intrinsics needed)
  - inlier ratio                             -> quality (best no-GT proxy)
  - recovered rotation angle (approx K)      -> downstream pose sanity / agreement

Pose uses an ASSUMED pinhole intrinsic (f = image width, principal point =
center) because this drone footage has no calibration; absolute pose is not
ground-truth, but the two models' recovered motion should agree if matching
is good.
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

PROJECT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IN_DIR = os.path.join(PROJECT, "_in")
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out")
SG_REPO = os.path.join(PROJECT, "SuperGluePretrainedNetwork")
sys.path.insert(0, SG_REPO)

from models.matching import Matching          # noqa: E402  (SuperGlue)
from models.utils import frame2tensor          # noqa: E402
from lightglue import LightGlue, SuperPoint    # noqa: E402

W, H = 640, 480
MAX_KPTS = 1024
WARMUP, REPEAT = 3, 5


def load_gray(path):
    g = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if g is None:
        sys.exit(f"Cannot read {path}")
    return cv2.resize(g, (W, H))


def timed(fn):
    """Warm up then average REPEAT runs; returns (result, mean_ms)."""
    for _ in range(WARMUP):
        fn()
    if torch.cuda.is_available():
        torch.cuda.synchronize()
    ts = []
    res = None
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
    """RANSAC inliers (fundamental) + approx rotation from essential matrix."""
    n = len(mkpts0)
    out = {"inliers": 0, "inlier_ratio": 0.0, "rot_deg": float("nan")}
    if n < 8:
        return out
    F, mask = cv2.findFundamentalMat(
        mkpts0, mkpts1, cv2.FM_RANSAC, 1.0, 0.999)
    if mask is not None:
        inl = int(mask.sum())
        out["inliers"] = inl
        out["inlier_ratio"] = inl / n
    f = float(W)
    K = np.array([[f, 0, W / 2.0], [0, f, H / 2.0], [0, 0, 1.0]])
    E, _ = cv2.findEssentialMat(
        mkpts0, mkpts1, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    if E is not None and E.shape == (3, 3):
        _, R, _, _ = cv2.recoverPose(E, mkpts0, mkpts1, K)
        out["rot_deg"] = math.degrees(
            math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))
    return out


def run_superglue(matching, g0, g1, device):
    inp0 = frame2tensor(g0.astype("float32"), device)
    inp1 = frame2tensor(g1.astype("float32"), device)
    with torch.no_grad():
        pred, ms = timed(lambda: matching({"image0": inp0, "image1": inp1}))
    pred = {k: v[0].detach().cpu().numpy() for k, v in pred.items()}
    kpts0, kpts1 = pred["keypoints0"], pred["keypoints1"]
    matches = pred["matches0"]
    valid = matches > -1
    mkpts0 = kpts0[valid]
    mkpts1 = kpts1[matches[valid]]
    return mkpts0, mkpts1, len(kpts0), len(kpts1), ms


def run_lightglue(extractor, matcher, g0, g1, device):
    def to_inp(g):
        t = torch.from_numpy(g).float() / 255.0
        return t[None].repeat(3, 1, 1).to(device)  # 3-channel, no resize
    f0 = extractor.extract(to_inp(g0), resize=None)
    f1 = extractor.extract(to_inp(g1), resize=None)
    with torch.no_grad():
        m01, ms = timed(lambda: matcher({"image0": f0, "image1": f1}))
    kpts0 = f0["keypoints"][0].detach().cpu().numpy()
    kpts1 = f1["keypoints"][0].detach().cpu().numpy()
    matches = m01["matches0"][0].detach().cpu().numpy()
    valid = matches > -1
    mkpts0 = kpts0[valid]
    mkpts1 = kpts1[matches[valid]]
    return mkpts0, mkpts1, len(kpts0), len(kpts1), ms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stem", default="bev-forest")
    ap.add_argument("--ref", type=int, default=0, help="reference second")
    ap.add_argument("--gaps", type=int, nargs="+", default=[1, 3, 6, 12])
    ap.add_argument("--weights", default="outdoor", choices=["indoor", "outdoor"])
    args = ap.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}   front-end: SuperPoint(max {MAX_KPTS})   "
          f"input: {W}x{H}\n")

    sg = Matching({
        "superpoint": {"nms_radius": 4, "keypoint_threshold": 0.005,
                       "max_keypoints": MAX_KPTS},
        "superglue": {"weights": args.weights, "sinkhorn_iterations": 20,
                      "match_threshold": 0.2},
    }).eval().to(device)
    lg_ext = SuperPoint(max_num_keypoints=MAX_KPTS).eval().to(device)
    lg_match = LightGlue(features="superpoint").eval().to(device)

    g_ref = load_gray(os.path.join(IN_DIR, f"{args.stem}_{args.ref:04d}s.jpg"))

    rows = []
    header = (f"{'pair':>10} {'model':>10} {'kpts':>9} {'match':>6} "
              f"{'inl':>5} {'inl%':>6} {'rot°':>6} {'ms':>8}")
    print(header)
    print("-" * len(header))
    for gap in args.gaps:
        m = args.ref + gap
        p1 = os.path.join(IN_DIR, f"{args.stem}_{m:04d}s.jpg")
        if not os.path.exists(p1):
            print(f"  (skip gap {gap}: missing {os.path.basename(p1)})")
            continue
        g1 = load_gray(p1)
        pair = f"{args.ref}->{m}"

        for name, fn in (("SuperGlue",
                          lambda: run_superglue(sg, g_ref, g1, device)),
                         ("LightGlue",
                          lambda: run_lightglue(lg_ext, lg_match, g_ref, g1, device))):
            mk0, mk1, nk0, nk1, ms = fn()
            geo = geometry(mk0, mk1)
            print(f"{pair:>10} {name:>10} {nk0:>4}:{nk1:<4} {len(mk0):>6} "
                  f"{geo['inliers']:>5} {geo['inlier_ratio']*100:>5.1f}% "
                  f"{geo['rot_deg']:>6.1f} {ms:>7.1f}")
            rows.append({
                "pair": pair, "model": name, "kpts0": nk0, "kpts1": nk1,
                "matches": len(mk0), "inliers": geo["inliers"],
                "inlier_ratio": round(geo["inlier_ratio"], 4),
                "rot_deg": round(geo["rot_deg"], 2), "match_ms": round(ms, 2),
            })
        print()

    os.makedirs(OUT_DIR, exist_ok=True)
    csv_path = os.path.join(OUT_DIR, "comparison.csv")
    with open(csv_path, "w", newline="") as f:
        wtr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wtr.writeheader()
        wtr.writerows(rows)
    print(f"Saved: {csv_path}")


if __name__ == "__main__":
    main()
