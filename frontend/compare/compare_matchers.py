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

HERE = os.path.dirname(os.path.abspath(__file__))
FRONTEND = os.path.dirname(HERE)            # holds SuperGluePretrainedNetwork
ROOT = os.path.dirname(FRONTEND)            # project root, holds _in
IN_DIR = os.path.join(ROOT, "_in")
OUT_DIR = os.path.join(HERE, "_out")
SG_REPO = os.path.join(FRONTEND, "SuperGluePretrainedNetwork")
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


def run_xfeat(xf, g0, g1, device, min_cossim=-1):
    """XFeat is detector+matcher in one; we time the FULL pipeline (not just
    the MNN step), since it has no SuperPoint front-end to share.
    min_cossim=-1 keeps all mutual matches; 0.82 is XFeat's own precision
    threshold that filters weak descriptor pairs."""
    def to_inp(g):
        return torch.from_numpy(g).float()[None, None].to(device) / 255.0
    i0, i1 = to_inp(g0), to_inp(g1)

    def pipeline():
        o0 = xf.detectAndCompute(i0, top_k=MAX_KPTS)[0]
        o1 = xf.detectAndCompute(i1, top_k=MAX_KPTS)[0]
        idx0, idx1 = xf.match(o0["descriptors"], o1["descriptors"],
                              min_cossim=min_cossim)
        return o0, o1, idx0, idx1

    with torch.no_grad():
        (o0, o1, idx0, idx1), ms = timed(pipeline)
    kpts0 = o0["keypoints"].cpu().numpy()
    kpts1 = o1["keypoints"].cpu().numpy()
    mkpts0 = kpts0[idx0.cpu().numpy()]
    mkpts1 = kpts1[idx1.cpu().numpy()]
    return mkpts0, mkpts1, len(kpts0), len(kpts1), ms


def run_xfeat_lighterglue(xf, g0, g1, device):
    """XFeat detector + LighterGlue (a distilled LightGlue trained for XFeat's
    64-d descriptors) -- the learned-matcher replacement for plain MNN. Timed
    as full pipeline, same as the other XFeat rows."""
    def to_inp(g):
        return torch.from_numpy(g).float()[None, None].to(device) / 255.0
    i0, i1 = to_inp(g0), to_inp(g1)

    def pipeline():
        o0 = xf.detectAndCompute(i0, top_k=MAX_KPTS)[0]
        o1 = xf.detectAndCompute(i1, top_k=MAX_KPTS)[0]
        o0["image_size"] = (W, H)
        o1["image_size"] = (W, H)
        mk0, mk1, _ = xf.match_lighterglue(o0, o1)
        return o0, o1, mk0, mk1

    with torch.no_grad():
        (o0, o1, mk0, mk1), ms = timed(pipeline)
    return mk0, mk1, len(o0["keypoints"]), len(o1["keypoints"]), ms


def run_xfeat_lighterglue_dyn(xf, g0, g1, device,
                              min_points=15, conf_hi=0.1, conf_lo=0.02):
    """Adaptive confidence for VIO survival: match at conf_hi for clean
    tracking, but if the match count drops below the factor-graph minimum,
    step down to conf_lo to recover enough points to stay alive through a bad
    clip. Returns the usual 5-tuple plus a note saying which threshold was used."""
    def to_inp(g):
        return torch.from_numpy(g).float()[None, None].to(device) / 255.0
    i0, i1 = to_inp(g0), to_inp(g1)
    used = {"conf": conf_hi, "stepped": False}

    def pipeline():
        o0 = xf.detectAndCompute(i0, top_k=MAX_KPTS)[0]
        o1 = xf.detectAndCompute(i1, top_k=MAX_KPTS)[0]
        o0["image_size"] = (W, H)
        o1["image_size"] = (W, H)
        mk0, mk1, _ = xf.match_lighterglue(o0, o1, min_conf=conf_hi)
        used["conf"], used["stepped"] = conf_hi, False
        if len(mk0) < min_points:
            mk0, mk1, _ = xf.match_lighterglue(o0, o1, min_conf=conf_lo)
            used["conf"], used["stepped"] = conf_lo, True
        return o0, o1, mk0, mk1

    with torch.no_grad():
        (o0, o1, mk0, mk1), ms = timed(pipeline)
    note = f"conf={used['conf']:.2f}" + ("  <-stepped" if used["stepped"] else "")
    return mk0, mk1, len(o0["keypoints"]), len(o1["keypoints"]), ms, note


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stem", default="bev-forest")
    ap.add_argument("--ref", type=int, default=0, help="reference second")
    ap.add_argument("--gaps", type=int, nargs="+", default=[1, 3, 6, 12])
    ap.add_argument("--weights", default="outdoor", choices=["indoor", "outdoor"])
    ap.add_argument("--detection_threshold", type=float, default=0.005,
                    help="SuperPoint keypoint score threshold, applied to BOTH "
                         "front-ends so the matcher is the only variable")
    ap.add_argument("--vio_min_points", type=int, default=15,
                    help="factor-graph minimum; XFeat+LGdyn steps conf down "
                         "(0.1->0.02) when matches fall below this")
    args = ap.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}   front-end: SuperPoint(max {MAX_KPTS}, "
          f"thr {args.detection_threshold})   input: {W}x{H}\n")

    sg = Matching({
        "superpoint": {"nms_radius": 4,
                       "keypoint_threshold": args.detection_threshold,
                       "max_keypoints": MAX_KPTS},
        "superglue": {"weights": args.weights, "sinkhorn_iterations": 20,
                      "match_threshold": 0.2},
    }).eval().to(device)
    lg_ext = SuperPoint(max_num_keypoints=MAX_KPTS,
                        detection_threshold=args.detection_threshold,
                        nms_radius=4).eval().to(device)
    lg_match = LightGlue(features="superpoint").eval().to(device)
    xf = torch.hub.load("verlab/accelerated_features", "XFeat",
                        pretrained=True, top_k=MAX_KPTS, trust_repo=True)

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
                          lambda: run_lightglue(lg_ext, lg_match, g_ref, g1, device)),
                         ("XFeat*",
                          lambda: run_xfeat(xf, g_ref, g1, device, min_cossim=-1)),
                         ("XFeat*.82",
                          lambda: run_xfeat(xf, g_ref, g1, device, min_cossim=0.82)),
                         ("XFeat+LG*",
                          lambda: run_xfeat_lighterglue(xf, g_ref, g1, device)),
                         ("XFeat+LGdyn",
                          lambda: run_xfeat_lighterglue_dyn(
                              xf, g_ref, g1, device,
                              min_points=args.vio_min_points))):
            res = fn()
            mk0, mk1, nk0, nk1, ms = res[:5]
            note = res[5] if len(res) > 5 else ""
            geo = geometry(mk0, mk1)
            print(f"{pair:>10} {name:>11} {nk0:>4}:{nk1:<4} {len(mk0):>6} "
                  f"{geo['inliers']:>5} {geo['inlier_ratio']*100:>5.1f}% "
                  f"{geo['rot_deg']:>6.1f} {ms:>7.1f}  {note}")
            rows.append({
                "pair": pair, "model": name, "kpts0": nk0, "kpts1": nk1,
                "matches": len(mk0), "inliers": geo["inliers"],
                "inlier_ratio": round(geo["inlier_ratio"], 4),
                "rot_deg": round(geo["rot_deg"], 2), "match_ms": round(ms, 2),
                "note": note,
            })
        print()

    os.makedirs(OUT_DIR, exist_ok=True)
    csv_path = os.path.join(OUT_DIR, f"comparison_{args.stem}.csv")
    with open(csv_path, "w", newline="") as f:
        wtr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wtr.writeheader()
        wtr.writerows(rows)
    print("\n* SuperGlue/LightGlue 'ms' is matcher-only (shared SuperPoint "
          "keypoints).\n  XFeat* 'ms' is the full detect+describe+match "
          "pipeline and uses its OWN detector,\n  so its kpts/matches are not "
          "drawn from the same points as the other two.")
    print(f"Saved: {csv_path}")


if __name__ == "__main__":
    main()
