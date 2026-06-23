# CLAUDE.md — frontend/ (matcher code)

Architecture and gotchas for the feature-matching experiments. All scripts live
under `frontend/`; the shared data dir `_in/` is at the project root. Scripts
resolve `_in` via `ROOT` (two levels up from a `frontend/<sub>/` script) and the
vendored SuperGlue repo via `FRONTEND` (one level up). See the root `CLAUDE.md`
for the conda env and run commands.

## Layout

- `frontend/superglue/` — `capture_frames.py`, `superglue_match.py`, `_out/`.
- `frontend/lightglue/` — `lightglue_match.py`, `_out/`. NOTE: this dir holds
  *our* script; the LightGlue *model* is the pip-installed `lightglue` package,
  not vendored here.
- `frontend/xfeat/` — `xfeat_match.py`, `_out/`. XFeat is loaded via
  `torch.hub.load("verlab/accelerated_features", ...)` (cached); no clone here.
- `frontend/compare/` — `compare_matchers.py` (matcher comparison,
  `_out/comparison_<stem>.csv`) and `compare_extractors.py` (extractor
  comparison, `_out/extractors_<stem>.csv`).
- `frontend/openvins-alike-lightglue/` — `extract_frames.py` (pull real
  MARS-LVIG frames from a bag via the `openvins:noetic` container) and
  `compare_tracking.py` (OpenVINS-KLT vs ALIKED+LightGlue). `_frames/`, `_out/`.
- `frontend/SuperGluePretrainedNetwork/` — upstream magicleap clone.
  **Gitignored**, but must exist on disk: the SuperGlue scripts and the compare
  harnesses `sys.path.insert` it and import `models.matching` / `models.utils`.
- All `_out/` dirs are gitignored; only the scripts are tracked.

## Architecture

**Shared front-end.** SuperGlue and LightGlue both sit on the same SuperPoint
keypoint detector/descriptor. SuperGlue uses the magicleap repo's `Matching`
wrapper (SuperPoint + SuperGlue in one module). LightGlue uses the pip package's
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

**Extractor harness (`compare_extractors.py`).** Fixes the *matcher* (LightGlue)
and swaps the *front-end* — SuperPoint, SIFT, DISK, ALIKED — each paired with
its features-matched LightGlue weights (`LightGlue(features=...)`). Feeds all the
same 640×480 RGB pixels and times extraction vs matching separately (extractor
speed is the point). First run needs internet to download the DISK/ALIKED/SIFT
extractor weights and their `*_lightglue` weights into `~/.cache/torch/hub/`
(SuperPoint is the only one cached by default); afterwards it runs fully offline.
Findings: ALIKED gives the best inlier ratios on degraded/natural footage at
moderate cost (~88 ms), SuperPoint is the fastest extractor (~52 ms) but the
quality floor, DISK peaks on clean short-baseline but is slowest (~150 ms), and
ALIKED's detector can collapse on a low-texture frame (bev-forest 0→12: 94 kpts
→ 0 matches) — a tail risk for VIO.

**Three-way tracking comparison (`openvins-alike-lightglue/`).** `compare_tracking.py`
compares KLT, ALIKED+LightGlue, and **XFeat+LGdyn** on real frames (AMvalley aerial
AND TUM-VI handheld via `--frames`), on VIO-relevant metrics: matches + RANSAC-
fundamental inliers **vs frame gap** (with worst-case `min`, the survival floor) and
**track survival** (features seeded on frame 0, chained through consecutive matches).
`XFDyn` class: XFeat `detectAndCompute` (cached per frame, image_size set for
LighterGlue) + adaptive `match_lighterglue` (conf=0.1; steps to 0.02 only when count
drops below `--vio_min_points`, default 15). Keypoint indices for survival tracking
are reconstructed from matched coordinates via argmin against the keypoint array.
All three `ms` columns measure the VIO incremental per-frame cost: given the previous
frame is already cached, what does it cost to process a new frame? KLT times
`klt_track` only (old-frame corners cached via `P(i)`); ALIKED+LG and XFeat+LGdyn
time `extract(new_frame) + match` (old-frame features cached via `F(i)` / `XF(i)`).

**Geometry without calibration.** This footage has no camera intrinsics, so:
inlier ratio is computed from a RANSAC **fundamental** matrix (needs no `K`),
and pose recovery uses an **assumed** pinhole `K` (focal = image width,
principal point = center). Recovered rotation is therefore approximate and not
ground truth — useful as a sanity/agreement check between models, not as an
accuracy metric.
