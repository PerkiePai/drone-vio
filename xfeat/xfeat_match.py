"""Match two frames with XFeat (Potje et al., CVPR 2024) and render a
paper-style side-by-side visualization with confidence-colored match lines.

Unlike SuperGlue/LightGlue, XFeat is NOT SuperPoint-based: it is a single
lightweight CNN that detects keypoints and 64-d descriptors, matched here with
mutual nearest-neighbour (its native sparse pipeline). So it is compared as a
*whole pipeline*, not as a drop-in matcher on shared keypoints.

We reuse SuperGlue's make_matching_plot so all three experiments look alike.
"""
import os
import sys
import time
import argparse

import numpy as np
import cv2
import torch
import matplotlib.cm as cm

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)
IN_DIR = os.path.join(PROJECT, "_in")
OUT_DIR = os.path.join(HERE, "_out")
sys.path.insert(0, os.path.join(PROJECT, "SuperGluePretrainedNetwork"))
from models.utils import make_matching_plot     # noqa: E402  (shared viz)

W, H = 640, 480


def frame_path(stem, sec):
    return os.path.join(IN_DIR, f"{stem}_{sec:04d}s.jpg")


def load_gray(path):
    g = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if g is None:
        sys.exit(f"Cannot read {path}")
    return cv2.resize(g, (W, H))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stem", default="bev-forest")
    ap.add_argument("--n", type=int, default=0)
    ap.add_argument("--m", type=int, default=None)
    ap.add_argument("--top_k", type=int, default=1024,
                    help="max XFeat keypoints per image")
    ap.add_argument("--min_cossim", type=float, default=-1,
                    help="cosine-sim threshold for MNN matches (-1 = keep all)")
    ap.add_argument("--img0", default=None)
    ap.add_argument("--img1", default=None)
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
    xf = torch.hub.load("verlab/accelerated_features", "XFeat",
                        pretrained=True, top_k=args.top_k, trust_repo=True)

    g0, g1 = load_gray(p0), load_gray(p1)
    img0 = g0.astype("float32")
    img1 = g1.astype("float32")

    def to_inp(g):
        return torch.from_numpy(g).float()[None, None].to(device) / 255.0

    t0 = time.perf_counter()
    out0 = xf.detectAndCompute(to_inp(g0), top_k=args.top_k)[0]
    out1 = xf.detectAndCompute(to_inp(g1), top_k=args.top_k)[0]
    idx0, idx1 = xf.match(out0["descriptors"], out1["descriptors"],
                          min_cossim=args.min_cossim)
    if device.type == "cuda":
        torch.cuda.synchronize()
    pipe_ms = (time.perf_counter() - t0) * 1000.0

    kpts0 = out0["keypoints"].cpu().numpy()
    kpts1 = out1["keypoints"].cpu().numpy()
    mkpts0 = kpts0[idx0.cpu().numpy()]
    mkpts1 = kpts1[idx1.cpu().numpy()]
    # Per-match confidence = cosine similarity of matched descriptors.
    d0 = out0["descriptors"][idx0]
    d1 = out1["descriptors"][idx1]
    conf = (d0 * d1).sum(-1).clamp(0, 1).cpu().numpy()
    color = cm.jet(conf)

    text = ["XFeat", f"Keypoints: {len(kpts0)}:{len(kpts1)}",
            f"Matches: {len(mkpts0)}"]
    small_text = [tag.replace("__", "  vs  "),
                  f"top_k: {args.top_k}  min_cossim: {args.min_cossim}"]

    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, f"{tag}_matches.png")
    make_matching_plot(
        img0, img1, kpts0, kpts1, mkpts0, mkpts1, color, text, out,
        show_keypoints=True, fast_viz=False, small_text=small_text)

    print(f"Matches: {len(mkpts0)} / keypoints {len(kpts0)}:{len(kpts1)}")
    print(f"Pipeline time (detect+match): {pipe_ms:.1f} ms ({device.type})")
    print(f"Saved: {out}")


if __name__ == "__main__":
    main()
