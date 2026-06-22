#!/usr/bin/env python3
"""Front-end tracking comparison for VIO on MARS-LVIG AMvalley (nadir forest).

Compares three front-ends on the SAME frames:
  - KLT         : FAST corners + pyramidal Lucas-Kanade optical flow (OpenVINS default)
  - ALIKED+LG   : learned detector + LightGlue learned matcher
  - XFeat+LGdyn : XFeat detector + LighterGlue with adaptive-confidence fallback
                  (matches at conf=0.1; steps to 0.02 only if count < vio_min_points)

Two views, both VIO-relevant:
  1. Pairwise vs frame-gap g in {1,2,3,5,10} -> matches, RANSAC-fundamental inliers,
     inlier ratio, latency.
  2. Track survival over consecutive frames -> features seeded on frame 0, how many
     survive to frame k.

No camera intrinsics needed: quality proxy is the fundamental-matrix inlier ratio.
Run in the `cv` conda env (torch + lightglue).  Frames from extract_frames.py.
"""
import os, sys, time, csv, argparse
import numpy as np, cv2, torch

HERE = os.path.dirname(os.path.abspath(__file__))
FRAMES = os.path.join(HERE, "_frames")
OUT = os.path.join(HERE, "_out")
os.makedirs(OUT, exist_ok=True)

from lightglue import LightGlue, ALIKED
from lightglue.utils import rbd

MAX_KPTS = 1024
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"


def list_frames():
    fs = sorted(f for f in os.listdir(FRAMES) if f.endswith(".png"))
    return [os.path.join(FRAMES, f) for f in fs]


def load(path, scale):
    bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    if scale != 1.0:
        bgr = cv2.resize(bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return bgr, gray


def geo_inliers(p0, p1):
    """RANSAC fundamental-matrix inliers (no intrinsics needed)."""
    p0 = np.ascontiguousarray(np.asarray(p0, dtype=np.float64).reshape(-1, 2))
    p1 = np.ascontiguousarray(np.asarray(p1, dtype=np.float64).reshape(-1, 2))
    # drop any NaN/Inf that can crash OpenCV
    ok = np.isfinite(p0).all(axis=1) & np.isfinite(p1).all(axis=1)
    p0, p1 = p0[ok], p1[ok]
    if len(p0) < 8:
        return 0
    try:
        F, mask = cv2.findFundamentalMat(p0, p1, cv2.FM_RANSAC, 1.0, 0.999)
        return int(mask.sum()) if mask is not None else 0
    except cv2.error:
        return 0


# ---------------- KLT (OpenVINS-style) ----------------
def fast_corners(gray, n=MAX_KPTS, thr=20):
    fast = cv2.FastFeatureDetector_create(threshold=thr)
    kps = fast.detect(gray, None)
    kps = sorted(kps, key=lambda k: -k.response)[:n]
    return np.array([k.pt for k in kps], np.float32).reshape(-1, 1, 2)


LK = dict(winSize=(21, 21), maxLevel=4,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
FB_THRESH = 1.0  # forward-backward consistency (px), as OpenVINS does


def klt_track(g0, g1, p0):
    """Bidirectional KLT: track fwd then back, keep only FB-consistent + in-bounds."""
    if len(p0) == 0:
        return p0[:0], np.zeros(0, bool)
    p1, st1, _ = cv2.calcOpticalFlowPyrLK(g0, g1, p0, None, **LK)
    p0b, st2, _ = cv2.calcOpticalFlowPyrLK(g1, g0, p1, None, **LK)
    fb = np.linalg.norm((p0 - p0b).reshape(-1, 2), axis=1)
    h, w = g1.shape
    x, y = p1.reshape(-1, 2)[:, 0], p1.reshape(-1, 2)[:, 1]
    good = (st1.reshape(-1).astype(bool) & st2.reshape(-1).astype(bool) &
            (fb < FB_THRESH) & (x >= 0) & (x < w) & (y >= 0) & (y < h))
    return p1, good


def klt_pair(g0, g1):
    p0 = fast_corners(g0)
    if len(p0) == 0:
        return np.empty((0, 2)), np.empty((0, 2))
    p1, good = klt_track(g0, g1, p0)
    return p0.reshape(-1, 2)[good], p1.reshape(-1, 2)[good]


# ---------------- ALIKED + LightGlue ----------------
def t_img(gray):
    return torch.from_numpy(gray)[None, None].float().to(DEVICE) / 255.0


class LG:
    def __init__(self):
        self.ext = ALIKED(max_num_keypoints=MAX_KPTS).eval().to(DEVICE)
        self.mat = LightGlue(features="aliked").eval().to(DEVICE)

    @torch.no_grad()
    def extract(self, gray):
        return self.ext.extract(t_img(gray), resize=None)

    @torch.no_grad()
    def match(self, f0, f1):
        m = self.mat({"image0": f0, "image1": f1})
        f0r, f1r, mr = rbd(f0), rbd(f1), rbd(m)
        idx = mr["matches"]
        k0 = f0r["keypoints"][idx[:, 0]].cpu().numpy()
        k1 = f1r["keypoints"][idx[:, 1]].cpu().numpy()
        return k0, k1, idx.cpu().numpy()


# ---------------- XFeat + LighterGlue (adaptive confidence) ----------------
class XFDyn:
    def __init__(self, h, w, min_points=15, conf_hi=0.1, conf_lo=0.02):
        self.xf = torch.hub.load("verlab/accelerated_features", "XFeat",
                                 pretrained=True, top_k=MAX_KPTS, trust_repo=True)
        self.img_size = (w, h)  # (W, H) as LighterGlue expects
        self.min_points = min_points
        self.conf_hi = conf_hi
        self.conf_lo = conf_lo

    @torch.no_grad()
    def extract(self, gray):
        t = torch.from_numpy(gray)[None, None].float().to(DEVICE) / 255.0
        o = self.xf.detectAndCompute(t, top_k=MAX_KPTS)[0]
        o["image_size"] = self.img_size
        return o

    @torch.no_grad()
    def match(self, f0, f1):
        """Returns (mk0, mk1, idx) where idx is (M,2) match indices into f0/f1 keypoints."""
        mk0, mk1, _ = self.xf.match_lighterglue(f0, f1, min_conf=self.conf_hi)
        if len(mk0) < self.min_points:
            mk0, mk1, _ = self.xf.match_lighterglue(f0, f1, min_conf=self.conf_lo)
        if isinstance(mk0, torch.Tensor):
            mk0, mk1 = mk0.cpu().numpy(), mk1.cpu().numpy()
        # Reconstruct row indices: mk* are exact subsets of f*["keypoints"]
        kpts0 = f0["keypoints"].cpu().numpy()
        kpts1 = f1["keypoints"].cpu().numpy()
        if len(mk0) > 0:
            i0 = np.abs(mk0[:, None] - kpts0[None]).max(axis=2).argmin(axis=1)
            i1 = np.abs(mk1[:, None] - kpts1[None]).max(axis=2).argmin(axis=1)
            idx = np.stack([i0, i1], axis=1)
        else:
            idx = np.zeros((0, 2), dtype=int)
        return mk0, mk1, idx


# ---------------- experiments ----------------
def pairwise(frames, scale, gaps, lg, xfd, n_pairs):
    grays = {}
    def G(i):
        if i not in grays: grays[i] = load(frames[i], scale)[1]
        return grays[i]
    feats = {}
    def F(i):
        if i not in feats: feats[i] = lg.extract(G(i))
        return feats[i]
    xfeats = {}
    def XF(i):
        if i not in xfeats: xfeats[i] = xfd.extract(G(i))
        return xfeats[i]

    rows = []
    for g in gaps:
        idxs = list(range(0, min(n_pairs * g, len(frames) - g), g))[:n_pairs]
        agg = {"KLT": [], "ALIKED+LightGlue": [], "XFeat+LGdyn": []}
        for i in idxs:
            # KLT
            t = time.time(); k0, k1 = klt_pair(G(i), G(i + g)); klt_ms = (time.time() - t) * 1e3
            agg["KLT"].append((len(k0), geo_inliers(k0, k1), klt_ms))
            # ALIKED+LightGlue (matcher-only; extraction cached)
            f0, f1 = F(i), F(i + g)
            torch.cuda.synchronize() if DEVICE == "cuda" else None
            t = time.time(); m0, m1, _ = lg.match(f0, f1)
            torch.cuda.synchronize() if DEVICE == "cuda" else None
            lg_ms = (time.time() - t) * 1e3
            agg["ALIKED+LightGlue"].append((len(m0), geo_inliers(m0, m1), lg_ms))
            # XFeat+LGdyn (matcher-only; extraction cached)
            xf0, xf1 = XF(i), XF(i + g)
            torch.cuda.synchronize() if DEVICE == "cuda" else None
            t = time.time(); xm0, xm1, _ = xfd.match(xf0, xf1)
            torch.cuda.synchronize() if DEVICE == "cuda" else None
            xf_ms = (time.time() - t) * 1e3
            agg["XFeat+LGdyn"].append((len(xm0), geo_inliers(xm0, xm1), xf_ms))
        for name, vals in agg.items():
            v = np.array(vals, float)
            m_match, m_inl, m_ms = v[:, 0].mean(), v[:, 1].mean(), v[:, 2].mean()
            ratio = (v[:, 1] / np.maximum(v[:, 0], 1)).mean()
            rows.append(dict(method=name, gap=g, pairs=len(idxs),
                             matches=round(m_match, 1), min_matches=int(v[:, 0].min()),
                             inliers=round(m_inl, 1),
                             inlier_ratio=round(ratio, 3), latency_ms=round(m_ms, 1)))
    return rows


def survival(frames, scale, lg, xfd, T):
    """Seed on frame 0, count survivors to frame k for all three methods."""
    grays = [load(frames[i], scale)[1] for i in range(T + 1)]

    # KLT (bidirectional FB-checked, as OpenVINS)
    p0 = fast_corners(grays[0]); n0 = len(p0); cur = p0; klt = [n0]
    for t in range(1, T + 1):
        p1, good = klt_track(grays[t - 1], grays[t], cur)
        cur = p1[good].reshape(-1, 1, 2)
        klt.append(len(cur))

    # ALIKED+LightGlue: chain consecutive matches from frame 0
    feats = [lg.extract(g) for g in grays]
    track = {j: j for j in range(rbd(feats[0])["keypoints"].shape[0])}
    lg_surv = [len(track)]
    for t in range(1, T + 1):
        _, _, idx = lg.match(feats[t - 1], feats[t])
        nxt = {}
        for a, b in idx:
            if a in track:
                nxt[b] = track[a]
        track = nxt
        lg_surv.append(len(track))

    # XFeat+LGdyn: chain consecutive matches from frame 0
    xfeats = [xfd.extract(g) for g in grays]
    xf_track = {j: j for j in range(len(xfeats[0]["keypoints"]))}
    xf_surv = [len(xf_track)]
    for t in range(1, T + 1):
        _, _, idx = xfd.match(xfeats[t - 1], xfeats[t])
        nxt = {}
        for a, b in idx:
            if int(a) in xf_track:
                nxt[int(b)] = xf_track[int(a)]
        xf_track = nxt
        xf_surv.append(len(xf_track))

    return n0, klt, lg_surv, xf_surv


def draw_matches(g0, g1, p0, p1, color, title):
    h, w = g0.shape
    canvas = cv2.cvtColor(np.hstack([g0, g1]), cv2.COLOR_GRAY2BGR)
    step = max(1, len(p0) // 200)  # don't draw all
    for a, b in zip(p0[::step], p1[::step]):
        pa = (int(a[0]), int(a[1])); pb = (int(b[0]) + w, int(b[1]))
        cv2.line(canvas, pa, pb, color, 1, cv2.LINE_AA)
        cv2.circle(canvas, pa, 2, color, -1); cv2.circle(canvas, pb, 2, color, -1)
    cv2.putText(canvas, title, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
    return canvas


def make_viz(frames, scale, lg, xfd, rows, klt, lgs, xfs):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    # --- match panels for a representative pair at gap 1 and gap 10 ---
    for gap in (1, 10):
        g0 = load(frames[40], scale)[1]; g1 = load(frames[40 + gap], scale)[1]
        k0, k1 = klt_pair(g0, g1)
        f0, f1 = lg.extract(g0), lg.extract(g1); m0, m1, _ = lg.match(f0, f1)
        xf0, xf1 = xfd.extract(g0), xfd.extract(g1); xm0, xm1, _ = xfd.match(xf0, xf1)
        a = draw_matches(g0, g1, k0, k1, (0, 200, 0), f"KLT  gap={gap}  n={len(k0)}")
        b = draw_matches(g0, g1, m0, m1, (0, 165, 255), f"ALIKED+LightGlue  gap={gap}  n={len(m0)}")
        c = draw_matches(g0, g1, xm0, xm1, (80, 80, 255), f"XFeat+LGdyn  gap={gap}  n={len(xm0)}")
        cv2.imwrite(os.path.join(OUT, f"matches_gap{gap}.png"), np.vstack([a, b, c]))
    # --- survival curve ---
    plt.figure(figsize=(6, 4))
    plt.plot(range(len(klt)), 100 * np.array(klt) / klt[0], "-o", ms=3, label="KLT")
    plt.plot(range(len(lgs)), 100 * np.array(lgs) / lgs[0], "-o", ms=3, label="ALIKED+LightGlue")
    plt.plot(range(len(xfs)), 100 * np.array(xfs) / xfs[0], "-^", ms=3, label="XFeat+LGdyn")
    plt.xlabel("frames since seed"); plt.ylabel("tracks surviving (%)")
    plt.title("Track survival (seeded on frame 0)"); plt.grid(alpha=0.3); plt.legend()
    plt.tight_layout(); plt.savefig(os.path.join(OUT, "survival.png"), dpi=120); plt.close()
    # --- matches + inlier ratio vs gap ---
    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    for name, mk in (("KLT", "o"), ("ALIKED+LightGlue", "s"), ("XFeat+LGdyn", "^")):
        r = [x for x in rows if x["method"] == name]
        gx = [x["gap"] for x in r]
        ax[0].plot(gx, [x["matches"] for x in r], mk + "-", label=name)
        ax[1].plot(gx, [x["inlier_ratio"] for x in r], mk + "-", label=name)
    ax[0].set_xlabel("frame gap"); ax[0].set_ylabel("mean matches"); ax[0].grid(alpha=0.3); ax[0].legend()
    ax[1].set_xlabel("frame gap"); ax[1].set_ylabel("RANSAC inlier ratio"); ax[1].grid(alpha=0.3); ax[1].legend()
    plt.tight_layout(); plt.savefig(os.path.join(OUT, "pairwise.png"), dpi=120); plt.close()
    print(f"viz -> {OUT}/matches_gap1.png, matches_gap10.png, survival.png, pairwise.png")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scale", type=float, default=0.5, help="resize factor (0.5 -> 1224x1024)")
    ap.add_argument("--gaps", type=int, nargs="+", default=[1, 2, 3, 5, 10])
    ap.add_argument("--pairs", type=int, default=30, help="frame pairs per gap")
    ap.add_argument("--surv_T", type=int, default=30, help="survival horizon (frames)")
    ap.add_argument("--vio_min_points", type=int, default=15,
                    help="XFeat+LGdyn steps conf 0.1->0.02 when match count drops below this")
    ap.add_argument("--viz", action="store_true", help="also save match panels + plots")
    ap.add_argument("--frames", default=None, help="frames dir (default ./_frames)")
    ap.add_argument("--out", default=None, help="output dir (default ./_out)")
    args = ap.parse_args()

    global FRAMES, OUT
    if args.frames: FRAMES = args.frames
    if args.out: OUT = args.out
    os.makedirs(OUT, exist_ok=True)

    frames = list_frames()
    if len(frames) < 12:
        sys.exit(f"need frames in {FRAMES} (run extract_frames.py); found {len(frames)}")
    h, w = load(frames[0], args.scale)[1].shape
    print(f"Device: {DEVICE}  frames: {len(frames)}  proc-res: {w}x{h}  max_kpts: {MAX_KPTS}\n")

    lg = LG()
    xfd = XFDyn(h, w, min_points=args.vio_min_points)
    # warmup both matchers
    g0 = load(frames[0], args.scale)[1]; g1 = load(frames[1], args.scale)[1]
    _ = lg.match(lg.extract(g0), lg.extract(g1))
    _ = xfd.match(xfd.extract(g0), xfd.extract(g1))

    print("=== pairwise vs frame-gap (mean over pairs) ===")
    print(f"{'method':>18} {'gap':>4} {'matches':>8} {'min':>5} {'inliers':>8} {'ratio':>6} {'ms':>7}")
    rows = pairwise(frames, args.scale, args.gaps, lg, xfd, args.pairs)
    for r in rows:
        print(f"{r['method']:>18} {r['gap']:>4} {r['matches']:>8} {r['min_matches']:>5} "
              f"{r['inliers']:>8} {r['inlier_ratio']:>6} {r['latency_ms']:>7}")
    with open(os.path.join(OUT, "pairwise.csv"), "w", newline="") as f:
        cw = csv.DictWriter(f, fieldnames=list(rows[0].keys())); cw.writeheader(); cw.writerows(rows)

    print("\n=== track survival (seeded on frame 0) ===")
    n0, klt, lgs, xfs = survival(frames, args.scale, lg, xfd, args.surv_T)
    print(f"KLT seeds={klt[0]}   ALIKED+LG seeds={lgs[0]}   XFeat+LGdyn seeds={xfs[0]}")
    print(f"{'frame':>6} {'KLT':>8} {'KLT%':>6} {'ALIKED+LG':>10} {'ALG%':>6} {'XF+LGdyn':>10} {'XF%':>6}")
    for k in [1, 2, 3, 5, 10, 15, 20, 25, 30]:
        if k <= args.surv_T:
            print(f"{k:>6} {klt[k]:>8} {100*klt[k]/max(klt[0],1):>5.0f}% "
                  f"{lgs[k]:>10} {100*lgs[k]/max(lgs[0],1):>5.0f}% "
                  f"{xfs[k]:>10} {100*xfs[k]/max(xfs[0],1):>5.0f}%")
    with open(os.path.join(OUT, "survival.csv"), "w", newline="") as f:
        cw = csv.writer(f); cw.writerow(["frame", "KLT", "ALIKED_LightGlue", "XFeat_LGdyn"])
        for k in range(len(klt)): cw.writerow([k, klt[k], lgs[k], xfs[k]])
    print(f"\nCSVs -> {OUT}/pairwise.csv, survival.csv")

    if args.viz:
        make_viz(frames, args.scale, lg, xfd, rows, klt, lgs, xfs)


if __name__ == "__main__":
    main()
