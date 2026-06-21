"""Match two consecutive frames (second n and n+1) with SuperPoint+SuperGlue
and render a side-by-side visualization with color-coded match lines,
in the style of the SuperGlue paper (magicleap/SuperGluePretrainedNetwork).
"""
import os
import sys
import argparse

import numpy as np
import torch
import matplotlib.cm as cm

HERE = os.path.dirname(os.path.abspath(__file__))
FRONTEND = os.path.dirname(HERE)            # holds SuperGluePretrainedNetwork
ROOT = os.path.dirname(FRONTEND)            # project root, holds _in

# Make the cloned repo importable
sys.path.insert(0, os.path.join(FRONTEND, "SuperGluePretrainedNetwork"))

from models.matching import Matching          # noqa: E402
from models.utils import read_image, make_matching_plot  # noqa: E402

IN_DIR = os.path.join(ROOT, "_in")
OUT_DIR = os.path.join(HERE, "_out")


def frame_path(stem, sec):
    return os.path.join(IN_DIR, f"{stem}_{sec:04d}s.jpg")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stem", default="bev-forest",
                    help="frame filename stem (without _NNNNs.jpg)")
    ap.add_argument("--n", type=int, default=0, help="first second index n")
    ap.add_argument("--m", type=int, default=None,
                    help="second second index (default n+1)")
    ap.add_argument("--weights", default="outdoor",
                    choices=["indoor", "outdoor"])
    ap.add_argument("--resize", type=int, nargs="+", default=[640, 480],
                    help="resize, e.g. 640 480; use -1 to keep original")
    ap.add_argument("--max_keypoints", type=int, default=1024)
    ap.add_argument("--keypoint_threshold", type=float, default=0.005)
    ap.add_argument("--match_threshold", type=float, default=0.2)
    ap.add_argument("--img0", default=None,
                    help="explicit first image path (overrides --stem/--n)")
    ap.add_argument("--img1", default=None,
                    help="explicit second image path (overrides --stem/--m)")
    args = ap.parse_args()

    if args.img0 or args.img1:
        if not (args.img0 and args.img1):
            sys.exit("Provide both --img0 and --img1")
        p0 = args.img0 if os.path.isabs(args.img0) else os.path.join(IN_DIR, args.img0)
        p1 = args.img1 if os.path.isabs(args.img1) else os.path.join(IN_DIR, args.img1)
        tag = (os.path.splitext(os.path.basename(p0))[0] + "__" +
               os.path.splitext(os.path.basename(p1))[0])
    else:
        m = args.m if args.m is not None else args.n + 1
        p0 = frame_path(args.stem, args.n)
        p1 = frame_path(args.stem, m)
        tag = f"{args.stem}_{args.n:04d}_{m:04d}"
    for p in (p0, p1):
        if not os.path.exists(p):
            sys.exit(f"Missing frame: {p}")

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Device: {device}  weights: {args.weights}")

    config = {
        "superpoint": {
            "nms_radius": 4,
            "keypoint_threshold": args.keypoint_threshold,
            "max_keypoints": args.max_keypoints,
        },
        "superglue": {
            "weights": args.weights,
            "sinkhorn_iterations": 20,
            "match_threshold": args.match_threshold,
        },
    }
    matching = Matching(config).eval().to(device)

    image0, inp0, _ = read_image(p0, device, args.resize, 0, False)
    image1, inp1, _ = read_image(p1, device, args.resize, 0, False)

    import time
    if device == "cuda":
        torch.cuda.synchronize()
    t0 = time.perf_counter()
    with torch.no_grad():
        pred = matching({"image0": inp0, "image1": inp1})
    if device == "cuda":
        torch.cuda.synchronize()
    match_ms = (time.perf_counter() - t0) * 1000.0
    pred = {k: v[0].cpu().numpy() for k, v in pred.items()}

    kpts0, kpts1 = pred["keypoints0"], pred["keypoints1"]
    matches, conf = pred["matches0"], pred["matching_scores0"]

    valid = matches > -1
    mkpts0 = kpts0[valid]
    mkpts1 = kpts1[matches[valid]]
    mconf = conf[valid]

    # Color match lines by confidence (jet: red=low, green/cyan=high) — paper style
    color = cm.jet(mconf)

    text = [
        "SuperGlue",
        f"Keypoints: {len(kpts0)}:{len(kpts1)}",
        f"Matches: {len(mkpts0)}",
    ]
    small_text = [
        tag.replace("__", "  vs  "),
        f"weights: {args.weights}  match_thr: {args.match_threshold}",
    ]

    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, f"{tag}_matches.png")
    make_matching_plot(
        image0, image1, kpts0, kpts1, mkpts0, mkpts1, color, text, out,
        show_keypoints=True, fast_viz=False, small_text=small_text)

    print(f"Matches: {len(mkpts0)} / keypoints {len(kpts0)}:{len(kpts1)}")
    print(f"Compare time: {match_ms:.1f} ms ({device})")
    print(f"Saved: {out}")


if __name__ == "__main__":
    main()
