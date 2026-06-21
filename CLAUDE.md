# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Purpose

Feature-matching experiments for drone visual-inertial odometry (VIO). The work
compares two learned matchers — **SuperGlue** and **LightGlue** — on top-down
(nadir) aerial drone forest footage, measuring match quality and latency to
decide what to use in a VIO front-end.

## Environment

All Python runs in the conda env **`car-detection`** (torch 2.5.1+cu121 with
CUDA, OpenCV, and the pip-installed `lightglue` package). The base env has no
torch. Always invoke through the env:

```
conda run -n car-detection python <script> <args>
```

Platform is Windows; the shell is PowerShell. When passing native-exe args,
mind PowerShell quoting (see the tool guidance). `2>$null` is handy to drop the
harmless `torch.load` FutureWarnings the SuperGlue weights emit.

## Commands

```bash
# 1. Extract one frame per second from every video in _in/ back into _in/
conda run -n car-detection python frontend/superglue/capture_frames.py

# 2. SuperGlue match — by second index, or by explicit image paths
conda run -n car-detection python frontend/superglue/superglue_match.py --n 0 --m 6
conda run -n car-detection python frontend/superglue/superglue_match.py --img0 a.jpg --img1 b.jpg

# 3. LightGlue match — identical CLI to the SuperGlue script
conda run -n car-detection python frontend/lightglue/lightglue_match.py --n 0 --m 1

# 4. XFeat match — same CLI plus --top_k / --min_cossim
conda run -n car-detection python frontend/xfeat/xfeat_match.py --n 0 --m 1

# 5. Comparison harness across frame gaps -> table + compare/_out/comparison_<stem>.csv
conda run -n car-detection python frontend/compare/compare_matchers.py --gaps 1 3 6 12
```

## Layout

All matcher code lives under `frontend/`; `_in/` (shared data) stays at the
project root. Scripts resolve `_in` via `ROOT` (two levels up from a
`frontend/<sub>/` script) and the SuperGlue repo via `FRONTEND` (one level up).

- `_in/` — shared inputs at the project root: source `*.mp4` plus extracted
  frames. **Gitignored.**
- `frontend/superglue/` — `capture_frames.py`, `superglue_match.py`, `_out/`.
- `frontend/lightglue/` — `lightglue_match.py`, `_out/`. NOTE: this dir holds
  *our* script; the LightGlue *model* is the pip-installed `lightglue` package,
  not vendored here.
- `frontend/xfeat/` — `xfeat_match.py`, `_out/`. XFeat is loaded via
  `torch.hub.load("verlab/accelerated_features", ...)` (cached); no clone here.
- `frontend/compare/` — `compare_matchers.py` and `_out/comparison_<stem>.csv`.
- `frontend/SuperGluePretrainedNetwork/` — upstream magicleap clone.
  **Gitignored**, but must exist on disk: the SuperGlue scripts and the compare
  harness `sys.path.insert` it and import `models.matching` / `models.utils`.
- All `_out/` dirs and `_in/` are gitignored; only the scripts are tracked.

## Architecture

**Shared front-end.** Both matchers sit on top of the same SuperPoint keypoint
detector/descriptor. SuperGlue uses the magicleap repo's `Matching` wrapper
(SuperPoint + SuperGlue in one module). LightGlue uses the pip package's
`SuperPoint` extractor feeding a separate `LightGlue` matcher. SuperGlue's
pretrained weights are bound to SuperPoint descriptors — you cannot swap in
SIFT/ORB.

**Frame-naming contract.** `capture_frames.py` writes `<stem>_NNNNs.jpg` (zero-
padded second index). The match scripts' `--stem/--n/--m` interface rebuilds
those paths via `frame_path()`. This naming is the coupling between extraction
and matching; keep it consistent.

**XFeat is a different paradigm.** Unlike the other two, XFeat is NOT
SuperPoint-based: one lightweight CNN does detection + 64-d description, matched
by mutual nearest-neighbour. It therefore cannot share the SuperPoint front-end
and is compared as a *whole pipeline*. In `compare_matchers.py` its `ms` column
is the full detect+match time (labeled `XFeat*`), whereas SuperGlue/LightGlue
`ms` is matcher-only on shared keypoints — do not read them as like-for-like.
On this repetitive-canopy footage XFeat's MNN matcher (run with `min_cossim=-1`,
keeping all mutual matches) yields many low-confidence matches and a much lower
RANSAC inlier ratio than the learned matchers; raising `--min_cossim` trades
matches for precision. The harness runs six rows: SuperGlue, LightGlue, XFeat*
(MNN, all matches), XFeat*.82 (MNN, 0.82 cossim filter), XFeat+LG* (XFeat
detector + LighterGlue, a learned matcher for XFeat's 64-d descriptors), and
XFeat+LGdyn (adaptive). The LighterGlue variant roughly 2-5x's XFeat's inlier
ratio and is best on the hardest/fastest footage — confirming XFeat's weakness
was the MNN matcher, not its descriptors. Results are written per-stem to
`compare/_out/comparison_<stem>.csv`.

**Adaptive confidence for VIO survival (XFeat+LGdyn).** Static LighterGlue uses
`min_conf=0.1`, which is great for precision but can starve at extreme baseline
(e.g. bev-forest 0->12 dropped to 1 match -> dead track). The XFeat+LGdyn row
matches at 0.1, and only if the count falls below `--vio_min_points` (default
15, the factor-graph minimum) does it re-match at 0.02 to recover enough points
to keep the VIO alive. On clean footage it never triggers (identical to static,
no cost); when it steps the printed row is annotated `conf=0.02  <-stepped` and
its `ms` reflects the second match call. It is the only matcher here that never
catastrophically failed.

**Mirrored match scripts.** `superglue_match.py`, `lightglue_match.py`, and
`xfeat_match.py` deliberately share the same CLI (`--stem/--n/--m`, `--img0/--img1`,
`--max_keypoints`), the same confidence→`cm.jet` line coloring (red=low,
green/cyan=high, as in the SuperGlue paper figure), and the same side-by-side
output. When changing one, mirror the other. Gotcha: LightGlue's
`viz2d.plot_matches` needs a *list of per-match color tuples*, not an `(N,4)`
array — passing the array raises an opaque "RGBA sequence should have length 3
or 4".

**Fairness harness (`compare_matchers.py`).** To isolate the *matcher*, it loads
each frame once with OpenCV, resizes to a fixed 640×480 grayscale, and feeds the
*identical pixels* to both models (bypassing each repo's own image loader; for
LightGlue it calls `extractor.extract(img, resize=None)` to suppress internal
resizing). Latency is warmed-up then averaged. Known remaining unfairness: the
SuperPoint *detection thresholds* differ (SuperGlue 0.005 → ~600 kpts vs
LightGlue's lower default → ~1024), so LightGlue starts from more candidates.

**Geometry without calibration.** This footage has no camera intrinsics, so:
inlier ratio is computed from a RANSAC **fundamental** matrix (needs no `K`),
and pose recovery uses an **assumed** pinhole `K` (focal = image width,
principal point = center). Recovered rotation is therefore approximate and not
ground truth — useful as a sanity/agreement check between the two models, not as
an accuracy metric.
