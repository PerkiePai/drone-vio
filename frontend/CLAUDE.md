# CLAUDE.md — frontend/ (matcher code)

Feature-matching experiments + flow-odom. All scripts live under `frontend/`; the
shared data dir `_in/` is at the project root (resolved via `ROOT`, two levels up from
a `frontend/<sub>/` script; vendored SuperGlue via `FRONTEND`, one level up). See the
root `CLAUDE.md` for the conda env (`cv`) and run commands.

## Layout

- `superglue/` — `capture_frames.py`, `superglue_match.py`.
- `lightglue/` — `lightglue_match.py` (the model is the pip `lightglue` package, not vendored).
- `xfeat/` — `xfeat_match.py` (XFeat via `torch.hub.load("verlab/accelerated_features")`, cached).
- `compare/` — `compare_matchers.py`, `compare_extractors.py` (→ `_out/*_<stem>.csv`).
- `openvins-alike-lightglue/` — `extract_frames.py` (pull MARS-LVIG frames from a bag via
  the `openvins:noetic` container) + `compare_tracking.py` (KLT vs ALIKED+LightGlue vs XFeat+LGdyn).
- `flow-odom/` — `flow_odometry.py`, altitude-scaled optical-flow odometry (metric-VIO
  alternative for the nadir cam). Plots estimate vs GT into the dataset dir.
- `SuperGluePretrainedNetwork/` — upstream magicleap clone. **Gitignored** but must exist
  on disk (scripts `sys.path.insert` it and import `models.matching` / `models.utils`).
- All `_out/`/`_frames/` are gitignored; only scripts are tracked.

## Matcher architecture

**Shared front-end.** SuperGlue and LightGlue both sit on **SuperPoint** keypoints —
SuperGlue via magicleap's `Matching` wrapper, LightGlue via the pip `SuperPoint`
extractor + separate `LightGlue` matcher. SuperGlue's weights are bound to SuperPoint;
you cannot swap in SIFT/ORB.

**Frame-naming contract.** `capture_frames.py` writes `<stem>_NNNNs.jpg` (zero-padded
second index); the match scripts' `--stem/--n/--m` rebuild those paths via `frame_path()`.
Keep this naming consistent — it's the coupling between extraction and matching.

**XFeat is a different paradigm** — one lightweight CNN does detection + 64-d description,
matched by mutual nearest-neighbour, so it can't share the SuperPoint front-end and is
compared as a *whole pipeline* (its `ms` is full detect+match, not matcher-only). On
repetitive canopy, plain MNN (`min_cossim=-1`) yields many low-confidence matches and a
low RANSAC inlier ratio; raising `--min_cossim` trades matches for precision.
`compare_matchers.py` runs six rows: SuperGlue, LightGlue, XFeat* (MNN all),
XFeat*.82 (0.82 cossim), XFeat+LG* (XFeat detector + **LighterGlue**), XFeat+LGdyn
(adaptive). LighterGlue ~2–5×'s XFeat's inlier ratio — the weakness was the MNN matcher,
not the descriptors.

**Adaptive confidence (XFeat+LGdyn).** Matches at `min_conf=0.1`; only if the count
falls below `--vio_min_points` (default 15, the factor-graph minimum) does it re-match at
0.02 to keep the VIO alive. Never triggers on clean footage (zero cost); when it steps,
the row is annotated `conf=0.02 <-stepped`. The only matcher here that never
catastrophically failed.

**Mirrored match scripts.** `superglue_match.py`, `lightglue_match.py`, `xfeat_match.py`
share the same CLI, confidence→`cm.jet` line coloring, and side-by-side output — mirror
changes across all three. Gotcha: LightGlue's `viz2d.plot_matches` needs a *list of
per-match color tuples*, not an `(N,4)` array.

**Fairness harnesses.** `compare_matchers.py` isolates the matcher: loads each frame once,
resizes to fixed 640×480 gray, feeds identical pixels to both, warmed-up averaged latency
(remaining unfairness: SuperPoint detection thresholds differ → LightGlue starts from more
candidates). `compare_extractors.py` fixes the matcher (LightGlue) and swaps the front-end
(SuperPoint/SIFT/DISK/ALIKED, each with its features-matched LightGlue weights); first run
needs internet for weights. Findings: ALIKED best inlier ratios on degraded footage
(~88 ms), SuperPoint fastest (~52 ms) but quality floor, DISK best clean/short-baseline
but slowest (~150 ms), ALIKED's detector can collapse on low-texture frames (tail risk).

**Three-way tracking (`openvins-alike-lightglue/compare_tracking.py`).** KLT vs
ALIKED+LightGlue vs XFeat+LGdyn on real frames (AMvalley aerial + TUM-VI via `--frames`),
on VIO metrics: matches + RANSAC-fundamental inliers vs frame gap (with worst-case `min`)
and track survival (seeded on frame 0, chained). All `ms` columns measure the **incremental
per-frame cost** (previous frame already cached): KLT times `klt_track`, the learned
methods time `extract(new) + match`.

**Geometry without calibration.** No intrinsics, so inlier ratio uses a RANSAC
**fundamental** matrix (no `K`), and pose recovery uses an **assumed** pinhole `K`
(focal = width, pp = center) — rotation is approximate, a sanity check not an accuracy metric.

## Flow-odom (`flow-odom/flow_odometry.py`)

A PX4Flow-style metric-VIO alternative for the nadir cam, since monocular MSCKF scale is
unobservable in near-constant-velocity cruise. Per frame pair: LK track → normalize with
pinhole `K` → **de-rotate** with relative attitude → least-squares the camera translation
from `Z·du = -tx + x·tz` (per-point ground depth `Z` from height + attitude ray–ground
intersection) → rotate to ENU and integrate. **Scale comes from height, not the
accelerometer**, so it stays bounded.
Run: `conda run -n cv python frontend/flow-odom/flow_odometry.py [--depth baro|agl]`
→ writes `flow_vs_gt_<depth>.png` (top-down vs GT, horiz error, altitude, depth-diagnosis).

**Frame convention:** GT `poses.csv` quats are **FLU-in-ENU** but the extrinsic is **FRD**
→ `R_body_cam = R_CtoI @ diag([1,-1,-1])`. Sanity: the optical axis must point DOWN; if `Z`
comes out negative and all points filter out, the convention is wrong.

**Depth source is the limiter — PROVEN (2026-06-23):**
- `baro` (default): scales flow by baro height-above-takeoff → correct shape/heading but
  **scale ~0.28 (~4× small)**, because Cesium terrain falls ~6× deeper than baro a.g.t.
  Final error 66 m.
- `agl`: true AGL = `camera_altitude − terrain_elevation` (terrain reconstructed by
  triangulating tracked features with GT poses, `compute_true_agl`) → **scale 1.14, final
  error 22.8 m = 2.2% of the 1 km path.** Proof that a correct metric depth fixes scale.
  (Uses GT poses as a "DEM" → upper-bound PoC; deploy by replacing `compute_true_agl` with a
  rangefinder/DEM lookup indexed by VIO lat/lon.)

**Accuracy levers (2026-06-25, on `_in/isaac-sim-20260624_2337`, 1773 m / 450 s):**
- `--stride N`: process every Nth frame (1-frame flow at 30 fps is ~0.3 px noise). **`--stride 5`
  → 4.6 m = 0.26%, scale 0.98.** Rule of thumb: stride ≈ fps/6.
- `--no_fb` toggles a forward-backward LK consistency check (reject reprojection > 1 px;
  on by default).
- `compute_ahrs_attitude(...)` + `run(attitude_R=…)`: run with **NO ground truth** via a
  Mahony AHRS (gyro+accel, GT-init heading). gyro+accel-only **1.89%** (yaw drifts, no
  compass); + a compass stand-in → **~1.4–1.5%**. So flow-odom + AGL is **deployable with no
  GT at ~1.5%** over 1.8 km; residual is AHRS tilt quality.

Headline plot: `flow_vs_gt_agl_s5.png`. See `backend/CLAUDE.md` → Isaac Sim.
