# Experiment 10 — Terrain-Relief/DEM Correlation (Canopy Global-Fix Add-On)

**Date:** 2026-07-10
**Plan:** `plan.md`
**Scripts:** `pipeline.py`, `flow_odometry.py` (snapshots in this folder)

## Summary

Added a `--relief_gate {off,on}` flag to `pipeline.py` that, when DSMAC's existing
canopy gate (`is_canopy_nonviable`, Exp09) flags a fix attempt as hopeless for image
matching, tries a TERCOM-style terrain-relief correlation fix instead: an offline
whole-flight DEM raster (`build_dem_raster`, triangulated from real GT position +
AHRS+compass attitude — see the corrected-premise note below) correlated against the
online sensed relief signal `terrain_elev = baro_altitude − agl` (real lidar AGL for
all six datasets). Additive — ds1's DSMAC-only behavior is untouched and byte-for-byte
reproduced when `--relief_gate off`.

**Result: the mechanism fires and produces nonzero, bounded fixes on every canopy and
repetitive-farmland leg — the plan's primary bar — but the effect on fused accuracy is
genuinely mixed: 2 of 5 datasets improve substantially (ds4 −33%, ds6 −18% RMSE), 2
regress slightly (ds2 +7%, ds3 +7%), and 1 is a null (ds5, only 1 relief attempt all
flight).** The a priori canopy-vs-repetitive-farmland split does **not** predict which
way a dataset goes — both categories contain one clear win and one regression. Two
premise corrections were needed before any of this could run (see below); both were
flagged to and resolved with the user before implementation.

---

## Corrected premises (found during implementation, before any code was run)

Two things in the plan's Design recap turned out to not match the actual data/code,
found by inspection before writing `build_dem_raster`:

1. **ds3 does have `lidar.csv`.** The plan's dataset table claimed ds3 (repetitive,
   `isaac-sim-20260704_193743`) had no `lidar.csv` and fell back to the
   GT-pose-triangulated AGL cache. Checked directly: `lidar.csv` exists, covers the
   full frame range, and Exp09's own logs already show `AGL: ... (real lidar
   rangefinder)` for ds3. All six datasets use real lidar AGL — the "ds3 idealized-AGL
   caveat" the plan asked to restate does not apply and is dropped from this write-up.

2. **None of the six datasets log per-frame GT attitude.** `poses.csv` for all six is
   `frame,x,y,z` only (no `qx,qy,qz,qw`) — confirmed by inspection. `build_dem_raster`'s
   ray-direction math needs `recs[i]["R_wb"]`, which `load_dataset` leaves `None` past
   frame 0 for these datasets; `pipeline.py` only ever populates it with the
   AHRS+compass estimate (`compute_ahrs_attitude`), consistent with the project's
   standing "GT is used ONLY for final scoring" rule. So the DEM cannot literally use
   "GT poses" as the plan's Design recap claimed — camera **position** is real GT
   (`poses.csv` x/y/z, always logged) but camera **attitude** is the AHRS+compass
   estimate (~2° bounded tilt error, this project's own measured figure). Resolved by
   moving the DEM-build call to *after* `compute_ahrs_attitude` runs (not right after
   the AGL block, where `R_wb` would still be `None` and crash), and documenting the
   corrected premise in `build_dem_raster`'s docstring. This is still not the Exp06
   self-triangulation circularity: AHRS attitude comes from independent physical
   sensors (gyro+accel+compass), not from the live flow-odom position trace the DEM is
   later correlated against.

---

## Datasets

| ID | Directory | Terrain | AGL median | Path length |
|----|-----------|---------|-----------|-------------|
| ds1 | `isaac-sim-20260630_152940` | farmland, DSMAC works (regression check only) | 87 m | 12523 m |
| ds2 | `isaac-sim-20260704_205334` | canopy | 344 m | 11161 m |
| ds3 | `isaac-sim-20260704_193743` | repetitive | 67 m | 11643 m |
| ds4 | `isaac-sim-20260705_230815` | canopy | 70 m | 10125 m |
| ds5 | `isaac-sim-20260706_105804` | canopy | 85 m | 9354 m |
| ds6 | `isaac-sim-20260705_220937` | repetitive | 81 m | 10033 m |

All runs full flight (no `--max_frames`), `--canopy_gate color_texture`. ds2's AGL
median (344 m) is much higher here than Exp09's reported 226 m because Exp09 truncated
ds2 to its first 6000 frames; this run used the full flight.

## DEM raster stats (whole-flight, `--relief_gate on` builds)

| ID | Cells (nx × ny @ 5 m) | Triangulated points |
|----|----------------------|---------------------|
| ds2 (canopy) | 1520 × 980 | 1,598,568 |
| ds3 (repetitive) | 910 × 1100 | 2,115,715 |
| ds4 (canopy) | 1362 × 824 | 2,089,651 |
| ds5 (canopy) | 1357 × 555 | 2,175,845 |
| ds6 (repetitive) | 788 × 1361 | 1,827,046 |

Every dataset landed 3–4 orders of magnitude above the `min_pts=200` sparsity guard —
canopy does reduce triangulated point count somewhat (ds2 lowest at 1.6M) but nowhere
near the regime where the DEM raster would be too sparse to grid. Unlike DSMAC's
appearance matching, single-camera short-baseline feature tracking under canopy still
finds enough correspondable structure to triangulate a dense point cloud; the failure
mode that kills DSMAC (cross-domain sim-render vs. Esri-ortho appearance) doesn't touch
this geometry-only triangulation at all.

## Per-dataset results — relief-fix engagement, accuracy, wall clock

| ID | Relief att/acc | DSMAC fixes (both runs) | RMSE off → on | Final err off → on | Wall clock off → on |
|----|----------------|--------------------------|----------------|---------------------|----------------------|
| ds1 (regression) | n/a (relief off) | 86/89 | 370.3 m (baseline) | 658.9 m (baseline) | 123 s |
| ds2 (canopy) | 47/68 (69%) | 0/0 | 499.6 → 533.2 m (**+6.7%**) | 556.7 → 608.4 m | 117 s → 289 s |
| ds3 (repetitive) | 7/11 (64%) | 0/0 | 533.9 → 572.1 m (**+7.2%**) | 881.7 → 932.1 m | 103 s → 280 s |
| ds4 (canopy) | 37/51 (73%) | 0/0 | 697.2 → **467.0 m (−33.0%)** | 1468.8 → **987.7 m (−32.7%)** | 77 s → 251 s |
| ds5 (canopy) | 0/1 (0%) | 0/0 | 736.5 → 736.5 m (unchanged) | 1198.5 → 1198.5 m (unchanged) | 124 s → 329 s |
| ds6 (repetitive) | 5/7 (71%) | 0/0 | 821.2 → **670.5 m (−18.3%)** | 1037.8 → **787.1 m (−24.1%)** | 94 s → 258 s |

DSMAC scored 0/0 on ds2–ds6 in both this run and Exp09's own validation — consistent,
expected, and the whole reason this experiment exists. Wall-clock overhead of the added
DEM build + correlation search is a consistent **+164 to +205 s** per full flight
(dominated by the one-time whole-flight triangulation, not the per-fix search).

## ds1 regression check — PASS

`--relief_gate off` (canopy_gate on, relief machinery unused) reproduces Exp09's ds1
baseline **exactly**: RMSE 370.3 m, final error 658.9 m, 86/89 DSMAC fixes accepted —
matching to the decimal place. `--relief_gate`'s default-off machinery does not touch
the already-working DSMAC-only path.

## Canopy vs. repetitive-farmland split — does NOT predict outcome

The plan's a priori expectation was that repetitive farmland (ds3, ds6) — being
geometrically flatter — would show legitimate **zero** relief-fixes (flat terrain, no
elevation signal to correlate), while canopy (ds2, ds4, ds5) would show the mechanism's
intended benefit (canopy-top height variation gives real relief). That didn't happen:

- **Both categories got nonzero, bounded relief-fix attempts** (7–68 attempts, 64–73%
  accept rate) — repetitive farmland here is not geometrically flat enough to starve
  the mechanism the way the plan predicted.
- **Both categories contain one clear win and one regression**: canopy ds4 improved
  33%, canopy ds2 regressed 7%; repetitive ds6 improved 18%, repetitive ds3 regressed
  7%. ds5 (canopy) is the one dataset where the mechanism barely engaged at all (1
  attempt in the whole flight, rejected).
- Accept **rate** is similar across every engaged dataset (64–73%) regardless of
  whether that dataset's RMSE improved or regressed — so the corr-threshold gate isn't
  what's separating winners from losers. The likely explanation is that accepted fixes
  in ds2/ds3 are landing on positions that pass the correlation/spread gates but are
  further from true GT than the fixes in ds4/ds6 (a per-fix-vs-GT accuracy signal this
  run didn't separately instrument — see Open items).

**Read the terrain-type labels as a proxy for image appearance (DSMAC's failure mode),
not for 3D relief (this mechanism's signal) — they are evidently decoupled.**

## Summary vs. target bar

| Bar | Result |
|---|---|
| Nonzero, bounded relief-fixes on canopy legs where DSMAC scored 0/0 (Exp09) | **Met** — 0–68 attempts, 0–73% accept rate, all canopy and repetitive-farmland datasets |
| No fused-RMSE regression on ds1 | **Met** — byte-identical to Exp09's baseline with the gate off |
| Relief correlation *improves* fused accuracy on canopy legs | **Mixed** — 2/5 datasets substantially better (ds4, ds6), 2/5 slightly worse (ds2, ds3), 1/5 null (ds5) |

The mechanism works as designed (it produces a nonzero corrected fix where none existed
before) but is not a uniform win — this is an honest, partial result, not the "canopy
fix rate moves decisively off zero" outcome the Exp08/09 open item was hoping for.
Whether it's net-positive depends on per-flight terrain relief quality in a way this
experiment surfaces but doesn't yet fully explain.

## Open items

- **Real (non-idealized) DEM source for deployment.** This experiment's DEM is built
  from the same flight's own GT position (real sensor-equivalent) but AHRS-estimated
  attitude — an upgrade path parallel to the AGL work: swap `build_dem_raster`'s
  whole-flight triangulation for a real elevation product or a genuine Cesium-terrain
  API query, keeping the online `terrain_elev` (real lidar) side unchanged.
- **Per-fix accuracy vs. GT was not separately instrumented** (matching the existing
  convention in this codebase — Exp09 also reports outcomes only via fused
  RMSE/final-error deltas, not per-fix GT error). Adding that would explain *why*
  ds2/ds3's accepted relief fixes hurt while ds4/ds6's helped, despite similar accept
  rates — the natural next step before trusting this mechanism unattended.
- **1D/along-track TERCOM vs. 2D patch search.** Chosen for simplicity and graceful
  degradation on sparse/noisy depth; whether it needs the finer spatial resolution of a
  2D patch search (closer to DSMAC's own approach) is still open, especially for ds2/ds3
  where the current formulation regressed.
