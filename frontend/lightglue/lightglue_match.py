"""Match two frames (consecutive seconds or explicit image paths) with
SuperPoint + LightGlue and render a paper-style side-by-side visualization
with confidence-colored match lines.

LightGlue (Lindenberger et al., ICCV 2023) is the faster, adaptive successor
to SuperGlue. Same front-end (SuperPoint keypoints/descriptors); the matcher
prunes points and exits early on easy pairs.
"""
import os
import sys
import time
import argparse

import torch
import matplotlib.cm as cm

from lightglue import LightGlue, SuperPoint
from lightglue.utils import load_image, rbd
from lightglue import viz2d

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))   # frontend/lightglue/ -> root
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
    ap.add_argument("--max_keypoints", type=int, default=1024)
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

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    extractor = SuperPoint(max_num_keypoints=args.max_keypoints).eval().to(device)
    matcher = LightGlue(features="superpoint").eval().to(device)

    image0 = load_image(p0).to(device)
    image1 = load_image(p1).to(device)

    feats0 = extractor.extract(image0)
    feats1 = extractor.extract(image1)

    if device.type == "cuda":
        torch.cuda.synchronize()
    t0 = time.perf_counter()
    matches01 = matcher({"image0": feats0, "image1": feats1})
    if device.type == "cuda":
        torch.cuda.synchronize()
    match_ms = (time.perf_counter() - t0) * 1000.0

    feats0, feats1, matches01 = [rbd(x) for x in [feats0, feats1, matches01]]
    kpts0, kpts1 = feats0["keypoints"], feats1["keypoints"]
    matches = matches01["matches"]
    mkpts0 = kpts0[matches[..., 0]]
    mkpts1 = kpts1[matches[..., 1]]
    scores = matches01["scores"].detach().cpu().numpy()

    # Color match lines by confidence (jet: red=low, green/cyan=high) — paper style.
    # viz2d.plot_matches needs a list of per-match tuples, not an (N,4) array.
    color = [tuple(c) for c in cm.jet(scores)]

    n_kpts0, n_kpts1, n_match = len(kpts0), len(kpts1), len(mkpts0)

    viz2d.plot_images([image0, image1])
    viz2d.plot_matches(mkpts0, mkpts1, color=color, lw=0.6)
    viz2d.add_text(0, "LightGlue", fs=18)
    viz2d.add_text(
        0, f"Keypoints: {n_kpts0}:{n_kpts1}\nMatches: {n_match}",
        pos=(0.01, 0.88), fs=12)

    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, f"{tag}_matches.png")
    viz2d.save_plot(out)

    print(f"Matches: {n_match} / keypoints {n_kpts0}:{n_kpts1}")
    print(f"Compare time: {match_ms:.1f} ms ({device.type})")
    print(f"Saved: {out}")


if __name__ == "__main__":
    main()
