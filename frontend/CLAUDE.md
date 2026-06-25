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
- `geoloc/` — `dsmac_match.py`, DSMAC-style vision-only **absolute** geo-localization
  (nadir frame ↔ satellite ortho). The drift-free *global* layer complementing flow-odom's
  drifting *local* odometry. Downloads/caches an Esri ortho into `_in/<dataset>/ortho_tiles`.
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

**Ablations (2026-06-25, `_in/isaac-sim-20260624_2337`, scale 0.5, stride 5, LK; scripts
were one-off in scratchpad, plots/`*.npz` saved in the dataset dir). Five findings:**

1. **Self-triangulated AGL is CIRCULAR (don't deploy it).** Replacing the GT-pose "DEM"
   in `compute_true_agl` with flow-odom's *own* estimated poses (Case B) cannot recover
   metric scale — it only reaches self-consistency at whatever scale it's seeded near.
   Cold-start (from baro) locks at scale ~0.73 (self-AGL median 50 m vs true 86 m);
   warm-start *at* the correct answer (0.30%) **drifts away** (scale 1.17→1.22→1.28,
   0.30%→1.19%). Bounded (0.9–1.5%, not km) but never metric. Same unobservability that
   sinks OpenVINS — you need an **external** metric reference. Plot `flow_caseB_selfagl_s5.png`.

2. **Front-end tracker barely matters; on RPE, ALIKED+LightGlue ≳ LK ≫ XFeat+LGdyn.**
   Swapping LK for learned matchers (geometry/AGL/attitude fixed) keeps final ATE in a
   0.2–0.5% band — depth (AGL) is the limiter, not correspondences. But **endpoint ATE is
   misleading**: XFeat had the best *final* (3.9 m) yet **worst** KITTI-RPE drift (4.62%
   avg) — its win was endpoint luck (scale 0.95 shrink + noisy steps cancelling). RPE
   ranking (honest): ALIKED+LightGlue **2.20%** < LK **2.42%** ≪ XFeat **4.62%**. LK is
   best accuracy-per-compute (≈90% of ALIKED at 2–4× the speed). Learned matchers only pay
   off where LK *breaks* (wide gaps / low texture), which this clean 12.5 Hz sim doesn't
   stress. Use RPE, not final ATE, to compare dead-reckoners. Plot `flow_track_RPE_compare.png`,
   trajectories cached in `tracker_trajs.npz`.

3. **A diverged OpenVINS run's ATTITUDE is also corrupted — can't rescue flow-odom.**
   Hypothesis "OV position blows up but orientation stays observable, so feed OV attitude
   into flow-odom" is **refuted**: over the OV window, OV attitude error vs GT is **51.6°
   mean / 114° max** (stepwise jumps = real divergence, not a frame bug; both ItoG/GtoI
   conventions tested) → 47% trajectory error. The Mahony **AHRS is far better (2.3° → 2.0%)**
   and remains the right no-GT attitude source. MSCKF scale collapse contaminates yaw too.

4. **AGL DEM keyed by VIO position has a FEEDBACK LOOP — prefer a rangefinder.** Looking up
   the terrain DEM at the *drifting estimated* xy (Case A, the realistic DEM deployment)
   instead of the true position costs ~12× with GT attitude (0.42%→**5.18%**) over this
   steep terrain (AGL 1–145 m): drift→wrong DEM→wrong scale→more drift. Bounded but real.
   A **downward rangefinder measures AGL directly with zero position dependence → no loop.**

5. **Depth is the dominant limiter and it COMPOUNDS with attitude error.** Clean 2×2
   (attitude {GT, AHRS} × depth {frame-AGL≈rangefinder, DEM@predicted}):

   | | frame-AGL (≈rangefinder) | DEM@predicted |
   |---|---|---|
   | GT-att   | 0.42% | 5.18% |
   | **AHRS** | **1.75%** | **12.76%** (all-deployable, no GT) |

   Isolated cost vs 0.42% baseline: Mahony AHRS **+1.33 pp**, DEM@predicted **+4.77 pp**
   (depth ~3.6× the bigger limiter). Crucially the two **compound super-additively**: AHRS+DEM
   = **12.76%**, far above the ~6.5% you'd get if independent — attitude drift feeds the DEM
   feedback loop. **Takeaway: a rangefinder (not a VIO-indexed DEM) is the key hardware
   choice — it takes the no-GT result 12.76%→1.75% (~7×).** Mahony AHRS is the cheap, solved
   part; keep it. Plot `hybrid_caseA_exp1.png`.

**Deployable recipe (no GPS, no GT):** flow-odom (LK) + Mahony AHRS attitude + **metric AGL
from a downward rangefinder** → ~1.75% over 1.8 km. Avoid: self-triangulated AGL (circular),
DEM-by-VIO-position (feedback), OV attitude from a diverged run (corrupted).

## DSMAC vision-only geo-localization (`geoloc/dsmac_match.py`)

The **drift-free, scale-free** complement to flow-odom. Instead of integrating motion (which
needs metric AGL and drifts), it matches each nadir frame against a **pre-stored satellite
ortho** and reads off **absolute** position — the monocular-scale problem is *bypassed*, not
solved. (TERCOM/DSMAC lineage; this is the "global fix" in the SPRIN-D odometry+map-matching
particle-filter pattern. Closest paper: *Altitude-Adaptive Vision-Only Geo-Localization for
UAVs*, arXiv 2602.23872.)

Per frame: de-rotate by `-yaw`, scale by `AGL/fx/GSD` to the ortho's m/px, ALIKED+LightGlue
match, RANSAC-homography the frame centre onto the ortho, pixel→ENU, score vs `geo.csv`.
Reference map = Esri World Imagery tiles (no key) stitched over the geo.csv footprint, cached
in `_in/<dataset>/ortho_tiles`.

**Result (2026-06-25, `_in/isaac-sim-20260624_2337`, 60 frames vs a 3072² ortho @ 0.29 m/px,
Bangkok):** match rate **55%** (33/60), abs-position error **median 13.8 m** (mean 11.2, p90
14.7, max 21.1), **100% of matches < 30 m**. Failures are over low-texture/repetitive ground
(parks, uniform rooftops) — mirrors the paper's ~50% real-flight R@1; successes are decisive
(100–160 inliers). Plot `dsmac_geoloc.png`.

**vs the flow-odom baseline — they fix opposite weaknesses, so FUSE them:**

| | flow-odom (local) | DSMAC (global) |
|---|---|---|
| Output | relative metric odometry | absolute position |
| Accuracy | **1.75%** (≈8–30 m over 1.8 km, best-case 0.42%) | **~14 m**, flat |
| Drift | accumulates (crosses 100 m only ~6 km out) | **none** — re-anchors every fix |
| Needs | metric AGL (rangefinder) | a reference satellite map |
| Coverage | every frame | only textured ground (~55%) |
| Scale problem | the core challenge | **bypassed** |

Flow-odom is locally tighter but drifts; DSMAC is coarser but never drifts and needs no
rangefinder. **Fused** (flow-odom propagates between fixes, DSMAC resets drift where texture
allows) = smooth + metric + bounded, **no GPS / no lidar / no rangefinder**. This is the full
two-layer stack, both halves validated on the same flight.

**Prototype caveats:** uses GT for the per-frame search-window centre + heading (validates the
matching *core*; deployed, those come from flow-odom dead-reckoning + AHRS) and `agl_cache.npz`
for warp scale (rough baro/flatness suffices — RANSAC tolerates scale error far better than
flow-odom integration). Domain gap is mild here because Cesium renders *from* satellite tiles;
real camera vs Esri is harder (the paper's 77%→50% drop). `geoloc/` + `ortho_tiles/` cache are
under gitignored `_in/`; only the script is tracked.
