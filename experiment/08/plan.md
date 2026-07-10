# Experiment 08 — Why DSMAC Fires on One Flight of Six (and Fades on That One)

**Date:** 2026-07-08
**Source:** `/grill-with-docs` session on the results of 6 `pipeline.py` runs (see Prior Results below)
**Conda env:** the 6 prior runs used `cv` (documented deviation, see note below); **all new work in
this experiment — the Experiment 1 sweep script and any future reruns — uses `drone`**, per standing
project rule (always `conda run -n drone`, never `cv`, regardless of any measured equivalence)
**Scripts:** `pipeline.py`, `flow_odometry.py` (snapshots in this folder; both already carry three
engineering fixes made earlier this session — see Prerequisites)

---

## Context

Six new-format recordings (real magnetometer, real lidar AGL, no per-frame GT attitude) were run
through `pipeline.py` end to end. Three infrastructure bugs were found and fixed along the way
(all already applied to the `pipeline.py`/`flow_odometry.py` snapshots in this folder):

1. **Magnetometer AHRS fusion** (`flow_odometry.py: compute_ahrs_attitude`) — the new recorder
   format drops per-frame GT attitude in favour of a real simulated magnetometer (`mx,my,mz` in
   `imu.csv`). The old compass path faked a compass from GT yaw, which no longer exists per-frame.
   Replaced with a genuine Mahony two-vector (gravity + magnetic-north) correction, world-mag
   vector calibrated once from GT frame-0 attitude + the mag reading at that instant. Verified by
   instrumenting the predicted-vs-measured mag residual across a full flight: mean 1.6°, max 11°,
   never diverging — the filter tracks the calibrated reference the whole flight.
2. **Lidar AGL interpolation** (`pipeline.py: run_pipeline`) — real per-frame AGL now exists in
   `lidar.csv`, but it's sampled at a different tick stride than `frames.csv` (e.g. every 5th sim
   tick vs every 10th), so frame numbers never exactly coincide. The original lookup required an
   exact match, silently failed every time, and fell back to baro (~5× scale error per ADR-0002's
   common-mode blindness — this alone explained the first broken run's 0/0 DSMAC fixes). Fixed by
   interpolating lidar AGL onto camera frame numbers instead of requiring an exact hit.
3. **Esri zoom auto-fallback** (`pipeline.py: _probe_zoom`, `build_ortho` call site) — `pipeline.py`
   hardcoded Esri zoom 19. For `isaac-sim-20260705_220937`'s flight region, Esri has **zero z19
   coverage** — confirmed by md5: 4,949 of 4,950 cached tiles were the byte-identical "Map data not
   yet available" placeholder. DSMAC was matching real drone footage against a blank grey tile.
   Fixed with a coverage probe that steps zoom down from 19 until a tile exceeds 4 KB (real Esri
   tiles here run 6.5–10 KB; the placeholder is a fixed ~2.5 KB regardless of location).

Also reconstructed `geo.csv`/`georef.json` for datasets that no longer have them logged directly
(`pipeline.py: _ensure_geo_georef`), deriving per-frame lat/lon from `takeoff.json`'s
`georef_origin` + `position_enu` and `poses.csv`'s takeoff-anchored ENU — verified against
`poses.csv`'s own displacement magnitude at the last logged frame (matched to within the expected
ENU→lat/lon conversion error).

### New confound: wind-induced camera shake

These 6 recordings also added simulated wind, which shakes the nadir camera — a confound not
present in earlier-format datasets and not accounted for in the original plan. Checked at the IMU
level before deciding how to handle it: gyro magnitude (mean/std) and gyro/accel frame-to-frame
jitter are **nearly identical across all 6 flights** (gyro jitter 0.0099–0.0103, accel jitter
0.130–0.145) — dataset 1 (the DSMAC success) isn't the calmest of the six; `isaac-sim-20260705_230815`
has the *lowest* gyro jitter of all 6 and is a total DSMAC failure. So raw IMU shake looks like a
roughly **uniform tax across all six**, not the discriminator between success and failure — but IMU
angular velocity is an indirect proxy (what actually matters is blur *in the image*, which depends
on rotation during the exposure window, not the raw gyro signal). Added a direct raw-image blur
metric to Experiment 1 below to test this properly rather than resting on the IMU proxy.

### Deviation from established convention: conda env

Experiments 01–07 always ran in `drone`, never `cv`, and a project memory claimed `cv` "crashes on
RTX 5090 kernels" for LightGlue/DSMAC specifically. Checked before deciding what to do with the 6
already-completed runs: `cv` and `drone` report identical `torch 2.11.0+cu128`, CUDA 12.8, `sm(12,0)`
— same stack, and no crash was observed in any of the 6 runs (dataset 1 got real, physically-sane
DSMAC fixes). Decision on the **already-collected data**: keep the 6 runs as-is, do not re-run —
doubling ~2 hours of GPU time for what should be a footnote isn't justified.

That equivalence finding does **not** extend forward, however: the user has since set a standing
project rule — always `conda run -n drone`, never `cv`, in this repo, independent of any measured
technical equivalence (see `[[feedback-drone-vio-use-drone-env]]` in memory). So the 6 prior runs
are a one-off, explicitly documented deviation, not a precedent — **all new work in this experiment
(the Experiment 1 sweep script, and any future pipeline.py runs) uses `drone`.** The stale memory
note about `cv` crashing has been corrected separately (it no longer crashes), but that correction
is now moot for env *choice* going forward — the rule is `drone`, full stop.

---

## Datasets and prior results

All six already run once (repo-root `pipeline.py`, `cv` env, AHRS+compass attitude, real lidar AGL,
SIFT+LightGlue DSMAC, `--min_inliers 30` default, `--fix_every 30 --skip_below 13 --reject 150`,
`--stride 5 --scale 0.5`; no `--autotune`).

| # | Dataset | Path | Fused RMSE | Final error | DSMAC fixes att/acc | AGL median | Esri zoom used |
|---|---------|------|-----------:|-------------:|:--------------------:|-----------:|:--------------:|
| 1 | `isaac-sim-20260630_152940` | 12 523 m | **370.3 m (2.96%)** | 658.9 m (5.26%) | **92 / 89 (97%)** | 87 m | z18 (fallback) |
| 2 | `isaac-sim-20260704_205334` | 11 161 m | 499.6 m (4.48%) | 556.7 m (4.99%) | 0 / 0 | 344 m | z18 (fallback) |
| 3 | `isaac-sim-20260704_193743` | 11 643 m | 533.9 m (4.59%) | 881.7 m (7.57%) | 0 / 0 | 67 m | z19 (native) |
| 4 | `isaac-sim-20260705_230815` | 10 125 m | 697.2 m (6.89%) | 1 468.8 m (14.51%) | 0 / 0 | 70 m | z18 (fallback) |
| 5 | `isaac-sim-20260706_105804` | 9 354 m | 736.5 m (7.87%) | 1 198.5 m (12.81%) | 0 / 0 | 85 m | z19 (native) |
| 6 | `isaac-sim-20260705_220937` | 10 033 m | 821.2 m (8.19%) | 1 037.8 m (10.34%) | 0 / 0 | 81 m | z18 (fallback) |

Dataset 6 was hand-diagnosed already (see prior session diagnostics): with a **GT position prior**
(best case, isolating pure matchability from drift), SIFT finds 1,015 / 1,622 keypoints on the
query/window pair (healthy), but LightGlue confidently matches only 16 of them (~1.6% match rate,
RANSAC inliers maxed at 5) — genuine cross-domain matching difficulty (Isaac-Sim-rendered nadir vs
real Esri imagery over this specific rural/farmland terrain), not a bug. Esri zoom (z18 vs z19,
column above) does **not** predict success — both zooms appear on both sides of the success/failure
split.

Dataset 1's own log shows a **within-flight** pattern worth explaining too: fixes accumulate cleanly
through frame ~18,000 (attempts == accepted, error 50–150 m) then stop being accepted in the last
~8,000 frames (89 accepted / 92 attempted from frame 21,000 on, drift climbing to 5,115 m by frame
27,000) — the same success/failure question, just localized within one flight instead of across six.

---

## Prior open questions this experiment resolves

Grilling session established (this doc *is* the grill list):

- **Q1 (scope):** understand why dataset 1 succeeded and the other 5 didn't — and why dataset 1
  itself fades at the end — *before* attempting any DSMAC robustness engineering. Guessing at a fix
  without knowing the failure mode risks the same trap ADR-0001/ADR-0002 already caught this repo in
  once (tuning a knob toward whichever metric looked best without a stated mechanism).
- **Q2 (rigor):** quantitative, not just eyeballed — sample match/inlier counts across all 6 flights,
  not just dataset 6.
- **Q3 (terrain proxy):** both an objective texture metric (Laplacian variance of the ortho patch)
  *and* a manual visual terrain label per flight (urban/suburban/farmland/forest/mixed) — cross-check
  whether they agree.
- **Q4 (prior):** GT position as the DSMAC search-window prior throughout, not the drifted estimate —
  isolates matchability from "flow-odom had already wandered off."
- **Q5 (write-up location):** superseded by discovering the `experiment/XX` convention — findings go
  in this experiment's `result.md`, not `frontend/CLAUDE.md`.
- **Q6 (env deviation):** keep the 6 existing `cv`-env runs, documented above; no re-run.
- **Q7 (tail-degradation scope):** in scope — same sweep, same sampling, just also read off dataset
  1's tail frames.
- **Q8 (wind shake):** added a raw-image blur metric to the sweep. IMU-level shake stats are already
  near-uniform across all 6 flights (see New confound note above), so the working expectation is
  shake is a roughly constant tax rather than the success/failure discriminator — but a direct
  blur-on-the-actual-image measurement is the real test, not the IMU proxy.

---

## Experiment 1: Cross-flight and within-flight matchability sweep

**Goal:** explain the 1-success/5-failure split, and dataset 1's own late-flight fade, with numbers
instead of a single eyeballed case.

**Method:** for each of the 6 datasets, sample ~25 frames evenly spaced by frame index across the
full flight (dataset 1 gets denser sampling in its last third — frames 18,000–28,903 — to resolve
the fade). For each sampled frame:

1. Build the DSMAC query patch exactly as `_dsmac_fix` does (`warp_north_up` at the frame's real
   AHRS+compass yaw and lidar AGL), but with **GT position** as the search-window centre (Q4).
2. Run SIFT extraction + LightGlue matching + RANSAC homography, exactly as `_dsmac_fix` does.
3. Record: SIFT keypoint counts (query, window), raw LightGlue match count, RANSAC inlier count,
   whether it would have cleared `min_inliers=30`, AGL at that frame, Esri zoom in use.
4. Record the ortho window's Laplacian-variance texture score (objective terrain-texture proxy).
5. Record the **raw query image's** Laplacian-variance blur score, computed on the unwarped source
   frame before `warp_north_up` (isolates motion blur from the warp's own resampling), plus the
   gyro magnitude at that frame's timestamp for cross-reference against the IMU proxy above.

Separately, for each dataset: one manual visual terrain label (urban/suburban/farmland/forest/mixed)
from inspecting a few representative query/ortho-window image pairs (same method used to diagnose
dataset 6 previously — save `query_warped.png` / `ortho_window.png`, view them).

**Script:** a standalone sweep script in this folder (`sweep_matchability.py`, to be written before
running), importing `flow_odometry.load_dataset`/`compute_ahrs_attitude` and
`pipeline.build_ortho`/`warp_north_up`/`make_sift_lg`/`_probe_zoom` directly rather than duplicating
their logic (avoids the drift between the diagnostic script and the real pipeline that would
undermine the "exactly as `_dsmac_fix` does" requirement).

**Metrics to record (CSV per dataset + one combined table):**

| Metric | Purpose |
|---|---|
| Match count, inlier count (per sample) | Direct matchability signal, comparable to the 30-inlier floor |
| Keypoint counts (query, window) | Rules out "not enough keypoints detected" as a confound |
| Texture score (Laplacian var of ortho window) | Objective terrain-texture proxy |
| Manual terrain label (per dataset) | Captures repetitive-vs-distinctive, which raw texture can't |
| Blur score (Laplacian var of raw query image) | Direct motion-blur measurement (wind-shake confound, Q8) |
| Gyro magnitude at sample | Cross-reference against the already-checked IMU-level proxy |
| AGL at sample | Rules out altitude/GSD-mismatch as a confound |
| Esri zoom in use | Already suspected ruled out (mixed on both sides) — confirm with numbers |
| Frame position in flight (for dataset 1 only) | Resolves the tail-fade question |

**Summary table of targets:**

| Question | Signal that would answer it |
|---|---|
| Is terrain type (not texture variance) the driver? | Manual label correlates with match/inlier rate; texture score does not (or only weakly) |
| Does dataset 1's fade correlate with a terrain-type change late in the flight? | Manual label / texture score shifts in the last third of dataset 1's samples, coinciding with the fix-acceptance drop-off |
| Does altitude drive it instead? | AGL correlates with match rate independent of terrain label |
| Does Esri zoom drive it? | Expected no — confirm z18/z19 datasets don't separate cleanly on match rate |
| Does wind-shake blur drive it (Q8)? | Blur score correlates with match/inlier rate across samples; IMU-level check already suggests shake is near-uniform across flights (a constant tax, not a discriminator) — the image-level blur score is the real test |

**Deliverable:** `experiment/08/result.md` — dataset table, the combined sweep table, key findings,
the two terrain-label vs. texture-score cross-checks, and a conclusion statement on what (if
anything) makes DSMAC-viable terrain predictable in advance. This result feeds a follow-up
`/writing-plans` improvement plan (not part of this experiment) — e.g., whether to invest in a
better matcher, a terrain-viability pre-check before committing to DSMAC on a given flight, or
neither.
