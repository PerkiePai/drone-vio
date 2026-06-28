# Experiment 04 — Results: LK vs Alternative Trackers

**Date:** 2026-06-28  
**Dataset:** `_in/isaac-sim-20260624_2337` (7.5 min, 1 773 m, ~86 m mean AGL)  
**Config:** `--depth agl --stride 5 --attitude ahrs_compass`

---

## Bug fixes applied during run

Two bugs in the committed scripts were fixed before the experiment could complete:

1. **`compare_trackers.py` line 296** — `SyntaxError`: positional `width` argument
   appeared after keyword `bottom=` in `ax.bar()`. Fixed to `width=width`.
2. **`flow_odometry.py` `_orb_match`** — `cv2.findFundamentalMat` raises
   `cv2.error` on degenerate near-planar nadir points (OpenCV 4.13 regression).
   Wrapped in `try/except cv2.error` to fall back to unfiltered matches.

---

## Accuracy results (Exp1)

| Tracker      | RMSE (m) | Final err (m) | Mean inliers | vs LK baseline |
|--------------|----------|---------------|--------------|----------------|
| lk           | 21.4     | 25.7          | 576          | —              |
| fast_lk      | 19.7     | 24.5          | 579          | −8%            |
| farneback    | 21.0     | 25.5          | 578          | −2%            |
| **dis**      | **18.7** | **20.5**      | 577          | **−13%**       |
| orb          | 23.4     | 37.8          | 366          | +9%            |
| sparse_raft  | 284.7    | 7.6           | 0            | FAIL           |

---

## Latency results (Exp2) — first 200 frames, budget 80 ms

| Tracker     | Detect | Track  | Solve | Total  | Margin | Real-time? |
|-------------|--------|--------|-------|--------|--------|------------|
| lk          | 0.9 ms | 1.0 ms | 2.4 ms | 4.2 ms | 75.8 ms | ✓ |
| fast_lk     | 0.9 ms | 0.9 ms | 2.4 ms | 4.2 ms | 75.8 ms | ✓ |
| farneback   | 1.4 ms | 30.2 ms | 2.4 ms | 34.1 ms | 45.9 ms | ✓ |
| dis         | 0.9 ms | 7.0 ms | 2.4 ms | 10.3 ms | 69.7 ms | ✓ |
| orb         | 2.5 ms | 0.7 ms | 2.0 ms | 5.2 ms | 74.8 ms | ✓ |
| sparse_raft | 1.0 ms | 46.0 ms | 0.0 ms | 47.0 ms | 33.0 ms | ✓ (but FAIL on accuracy) |

---

## Key findings

### Primary question: does any method beat LK RMSE within the 80 ms budget?

**Yes. DIS and FAST+LK both beat LK while staying well within budget.**

| Method  | RMSE | Latency | Verdict |
|---------|------|---------|---------|
| **DIS** | **18.7 m** (−13%) | 10.3 ms | **Best overall — new recommended baseline** |
| FAST+LK | 19.7 m (−8%) | 4.2 ms | Same speed as LK, better accuracy |
| Farneback | 21.0 m (−2%) | 34.1 ms | Negligible gain, 8× slower |
| ORB | 23.4 m (+9%) | 5.2 ms | Fewer inliers (366 vs 576), worse accuracy |

### DIS wins on accuracy
DIS dense flow (medium preset) reduces RMSE from 21.4 m to 18.7 m and final
error from 25.7 m to 20.5 m. Latency is 10.3 ms vs 4.2 ms for LK — still
69.7 ms inside the 80 ms budget. The dense flow produces the same number of
inliers as LK (577 vs 576) but with smaller residuals.

### FAST+LK: free accuracy gain
Swapping Shi-Tomasi for FAST corner detection gives 19.7 m RMSE (−8%) at
identical 4.2 ms latency. FAST finds slightly different corner distributions
that happen to track better on nadir texture at this altitude.

### ORB is worse
ORB yields only 366 inliers (vs ~578 for other methods) because BFMatcher
crossCheck + F-matrix RANSAC is conservative on nadir imagery where many
matches are ambiguous (repetitive texture). This leads to worse RMSE (23.4 m)
despite fast matching (0.7 ms).

### RAFT-Small: domain mismatch — complete failure
RAFT-Small (torchvision, Sintel pre-train) produces 0 usable inliers across
the entire 7.5-min flight. Diagnostic on a single stride-5 frame pair shows:

- Forward flow magnitude: **38 px mean** (large displacement, high altitude)
- Forward-backward error: **mean 72 px, median 28 px** (vs 1 px threshold)
- Inliers at 1 px: 2/600 · at 2 px: 16/600 · at 3 px: 30/600

The model is fundamentally out-of-distribution. RAFT was trained on synthetic
and natural-scene video with small motions (~1–10 px). Nadir aerial at 86 m
AGL with 5-frame stride produces 38 px mean displacement with repetitive
ground texture — the model hallucinates inconsistent flow fields.

**Note:** RAFT final error is listed as 7.6 m only because the trajectory
never moved (zero velocity integrated), so it trivially ends near the origin
of the coordinate system. The 284.7 m RMSE reflects pure accumulation of
position errors from drift without any flow-based correction.

---

## Output files

- `experiment/04/isaac-sim-20260624_2337/compare_trajectories.png` — top-down
  ENU trajectories for all 6 trackers overlaid on GT, plus error-over-time curves
- `experiment/04/isaac-sim-20260624_2337/compare_latency.png` — stacked bar
  chart of detect/track/solve latency per tracker with budget line

---

## Recommendation

**Switch from `lk` to `dis` as the new default tracker.**  
- RMSE: 18.7 m vs 21.4 m (−13%)  
- Latency: 10.3 ms vs 4.2 ms (still 70 ms inside budget)  
- Drop-in replacement — identical odometry math, same inlier count

Alternatively, `fast_lk` gives −8% RMSE for free (same latency as `lk`).

---

## Open items carried to Exp05

- **Compass noise sensitivity** (`mag_noise_deg > 0`): how much does real
  magnetometer noise degrade AHRS+compass accuracy? (deferred from Exp03)
- **Short-flight tilt gap**: roll/pitch scale error (1.31×) requires AGL
  rangefinder or multi-cam structural fix
- **RAFT on nadir**: would require a nadir-specific fine-tune or a model
  trained on large-displacement aerial data (e.g. GMFlow-large)
