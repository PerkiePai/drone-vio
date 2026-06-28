# Test Plan — Flow-odom + DSMAC Evaluation

## Datasets

| ID | Directory | Duration | GT range | Alt (mean) | Notes |
|----|-----------|----------|----------|------------|-------|
| **Short** | `_in/isaac-sim-20260624_2337` | 7.5 min | 1 773 m | 43.5 m | Main ablation dataset; all prior results here |
| **Long** | `_in/isaac-sim-20260625` | 76.9 min | **22.5 km path** (max radius 9.5 km) | 48.9 m | Long-mission; both cams frame-matched; attitude compare exists |

Run all experiments on **both datasets** unless noted. The long dataset is the critical one for validating fusion — flow-odom drift will cross DSMAC's ~13 m ceiling well before 9.5 km, so fused should clearly win there.

---

## Pre-experiment: `--reject` Gate Calibration

**Goal:** find a safe `--reject` threshold for the long dataset before running the main experiments.
The default 45 m works on the short flight (flow-odom drift stays < 45 m throughout).
On the long flight, flow-odom at 1.75% drift accumulates ~385 m over 22 km — a correct DSMAC fix
will be rejected once drift exceeds 45 m, making fusion fail silently.

**Run on short dataset first** (known ground truth to validate the gate logic):

| Run | `--reject` | Expected behaviour |
|-----|------------|-------------------|
| R1 | 15 m (tight) | fewer accepted fixes; may miss good fixes early in flight |
| R2 | 45 m (default) | 45/45 accepted on prior run — use as sanity check |
| R3 | 150 m (loose) | accepts more fixes; risk of outlier corruption |

```bash
conda run -n cv python frontend/geoloc/fuse_flowodom_dsmac.py \
    --dir _in/isaac-sim-20260624_2337 --reject 15
conda run -n cv python frontend/geoloc/fuse_flowodom_dsmac.py \
    --dir _in/isaac-sim-20260624_2337 --reject 45
conda run -n cv python frontend/geoloc/fuse_flowodom_dsmac.py \
    --dir _in/isaac-sim-20260624_2337 --reject 150
```

**Then on long dataset:**

| Run | `--reject` | Expected behaviour |
|-----|------------|-------------------|
| R4 | 45 m | many good fixes rejected once drift > 45 m → fusion breaks after ~2–3 km |
| R5 | 150 m | more fixes accepted as drift grows; some outliers possible |
| R6 | 400 m | very loose; nearly all fixes accepted but outlier risk is high |

```bash
conda run -n cv python frontend/geoloc/fuse_flowodom_dsmac.py \
    --dir _in/isaac-sim-20260625 --reject 45
conda run -n cv python frontend/geoloc/fuse_flowodom_dsmac.py \
    --dir _in/isaac-sim-20260625 --reject 150
conda run -n cv python frontend/geoloc/fuse_flowodom_dsmac.py \
    --dir _in/isaac-sim-20260625 --reject 400
```

**Metrics to record:**

| Metric | How |
|--------|-----|
| Accepted / attempted fixes | printed by script |
| Rejected count | printed by script |
| RMSE (m) | fused vs GT |
| First rejection frame | at what km does the gate start rejecting good fixes (R4) |

**Plots:**
- Accepted (green) vs rejected (red) fix dots on the top-down map for each threshold
- Error over distance for each threshold on the same axes — shows where tight gate hurts

**Decision:** pick the `--reject` value used for all Exp 2 runs based on this sweep.
Recommended starting point: **150 m for the long dataset**, 45 m for the short.

**⚠ Prerequisite:** `tracker_trajs.npz` and `agl_cache.npz` must exist in `_in/isaac-sim-20260625/`
before any fuse run on the long dataset. Generate them by running flow_odometry on the long dataset first.

---

## Experiment 1: Flow-odom — Rangefinder vs Baro

**Goal:** quantify the AGL source impact on accuracy and measure per-frame latency for real-time feasibility.

| Run | `--depth` | `--stride` | Attitude |
|-----|-----------|------------|----------|
| A | `agl`  (rangefinder proxy) | 5 | GT |
| B | `baro` (no rangefinder)    | 5 | GT |
| C | `agl`  | 5 | AHRS (Mahony, no GT) |
| D | `baro` | 5 | AHRS |

Run C (short): `conda run -n cv python frontend/flow-odom/flow_odometry.py --dir _in/isaac-sim-20260624_2337 --depth agl --stride 5`
Run C (long):  `conda run -n cv python frontend/flow-odom/flow_odometry.py --dir _in/isaac-sim-20260625 --depth agl --stride 5`

**Metrics to record:**

| Metric | How |
|--------|-----|
| RMSE (m) | vs `geo.csv` GT |
| Final error (m) | last-frame position error |
| Scale factor | median `estimated_step / true_step` |
| Per-frame latency (ms) | time LK track + de-rotate + LS solve, averaged over all frames |
| Real-time margin | `latency_ms` vs `stride × frame_period_ms` (stride 5 × 80 ms = 400 ms budget) |

**Plots:**
- Top-down trajectory (estimated vs GT)
- Horizontal error over time/distance
- Depth diagnosis (AGL vs baro vs true height)

**What to look for:** runs A/C should match the ablation table from CLAUDE.md (0.42% GT-att, 1.75% AHRS). Run D is the worst-case no-hardware baseline (~12.76% expected). Latency should be well under budget for flow-odom.

---

## Experiment 2: Fused Flow-odom + DSMAC — Rangefinder vs Baro

**Goal:** same 2×2 matrix as Experiment 1, but with DSMAC fixes layered on top. Also stress-test the real-time budget since DSMAC is heavy.

| Run | Flow-odom depth | DSMAC warp AGL | Attitude | Fusion flags |
|-----|-----------------|----------------|----------|--------------|
| E | `agl`  | true (from `agl_cache.npz`) | GT   | default |
| F | `baro` | baro           | GT   | default |
| G | `agl`  | true           | AHRS | default |
| H | `baro` | baro           | AHRS | default |

Run E (short): `conda run -n cv python frontend/geoloc/fuse_flowodom_dsmac.py --dir _in/isaac-sim-20260624_2337`
Run E (long):  `conda run -n cv python frontend/geoloc/fuse_flowodom_dsmac.py --dir _in/isaac-sim-20260625`

**Metrics to record:**

| Metric | How |
|--------|-----|
| RMSE (m) | full trajectory vs GT |
| Final error (m) | last-frame |
| DSMAC fix rate | `accepted / attempted` |
| DSMAC rejected count | fixes outside `--reject` gate |
| DSMAC per-fix latency (ms) | ALIKED detect + LightGlue match + RANSAC, per fix frame |
| Flow-odom per-frame latency (ms) | same as Exp 1 |
| Total pipeline latency (ms) | worst-case frame: flow-odom + DSMAC running |

**Plots:**
- Top-down trajectory (estimated vs GT) — same format as Exp 1 for direct comparison
- Error over distance — show sawtooth (each DSMAC fix yanks error down)
- DSMAC fix scatter: accepted (green) vs rejected (red) on the top-down map

**What to look for:**
- **Short (1.8 km):** fused may not beat flow-odom-only (10.3 m vs 8.3 m from prior run) — expected, flow-odom is already tighter than DSMAC's ~13 m fixes.
- **Long (22.5 km path):** fused should clearly win. At 1.75% drift, flow-odom alone accumulates ~394 m by 22.5 km; fused stays bounded at ~13 m. This is the key payoff experiment.

---

## Experiment 3: Real-time Stream Feasibility

**Goal:** determine if the pipeline can run in real-time at 12.5 Hz (80 ms/frame) or 30 Hz (33 ms/frame).

**Tests:**
1. Simulate real-time by feeding frames one-by-one with `time.sleep(1/fps)` and measuring wall-clock lag.
2. Measure DSMAC fix latency separately: is it < `fix_every × frame_period`? (default `fix_every=30` → 30 × 80 ms = 2.4 s budget).
3. Profile where time goes: LK track vs ALIKED detect vs LightGlue match vs RANSAC vs tile lookup.

**Expected finding:** flow-odom alone is real-time (LK < 10 ms/frame). DSMAC per fix is likely 200–800 ms — it must run in a **background thread** and post fixes asynchronously; flow-odom dead-reckons in the main loop. This is the deployed architecture.

---

## What Else to Measure / What You May Have Missed

### 1. Attitude as a separate variable (important)
The 2×2 table in Experiment 1 covers this — but don't skip runs C and D. From the ablation, attitude error alone costs +1.33 pp and compounds with AGL error super-additively (AHRS+baro = 12.76% vs 6.5% expected if independent). This compounding is the key insight for hardware selection.

### 2. DSMAC warp AGL source vs flow-odom AGL source (separate)
DSMAC uses AGL only to set warp scale; RANSAC tolerates errors here. But test baro vs true AGL for DSMAC independently (hold flow-odom AGL fixed) to quantify how much warp error actually costs on match rate and position accuracy.

### 3. Long-flight crossover (now directly testable with `isaac-sim-20260625`)
The theoretical crossover where fused beats flow-odom is ~6 km. The 9.5 km dataset goes well past it.
- Plot error vs distance for both datasets on the same axes — the crossover point should be visible
- `isaac-sim-20260625` already has `attitude_compare.png` and half/second-half splits recorded; check for any existing cached trajectories before re-running from scratch

### 4. Rejection gate sensitivity (`--reject`)
Default is 45 m. Test 15 m (tight) vs 45 m (default) vs 90 m (loose). Tight gate → fewer fixes accepted but no bad ones. Loose gate → more fixes but outliers can corrupt the trajectory. Find the sweet spot for this terrain.

### 5. `--skip_below` and `--conf_blend` flags
These are already in `fuse_flowodom_dsmac.py` and designed to make fused win even on short flights:
- `--skip_below 13` — only apply a DSMAC fix once flow-odom drift exceeds DSMAC's accuracy (13 m), so weak fixes don't corrupt a better trajectory
- `--conf_blend` — weight fix pull by inlier count so low-confidence fixes barely nudge position

Test with these enabled on Runs E and G to see if fused RMSE beats flow-odom-only on the short 1.8 km flight.

### 6. Error vs distance curve (not just final RMSE)
Final RMSE is misleading for dead-reckoners (XFeat had the best final ATE but worst RPE from ablation finding 2). Plot error growth rate:
- Flow-odom: should grow linearly with distance
- Fused: should be bounded (sawtooth pattern, not growing)
This is the most important plot for showing the value of fusion on long flights.

### 7. DSMAC match rate breakdown by terrain type
55% overall match rate. Which frame segments fail? Overlay the match/fail pattern on the top-down map — expect failures over parks, uniform rooftops, open water. This shows where DSMAC is reliable and where flow-odom must carry the load.

### 8. Scale factor tracking over time
For flow-odom: log the per-step scale factor (estimated / true step distance) at each frame. Baro should show ~4× underscale vs true AGL from ablation. This is a good diagnostic when rangefinder is unavailable.

---

## Summary Table (expected results to confirm)

### Short dataset — `isaac-sim-20260624_2337` (1.8 km)

| Run | Setup | Expected RMSE | Expected latency |
|-----|-------|---------------|-----------------|
| A | flow-odom, agl, GT-att  | ~7–9 m (0.4%)  | < 20 ms/frame |
| B | flow-odom, baro, GT-att | ~66 m (3.7%)   | < 20 ms/frame |
| C | flow-odom, agl, AHRS    | ~31 m (1.75%)  | < 20 ms/frame |
| D | flow-odom, baro, AHRS   | ~226 m (12.76%)| < 20 ms/frame |
| E | fused, agl, GT-att      | ~10 m bounded  | DSMAC: ~200–800 ms/fix |
| F | fused, baro, GT-att     | >10 m bounded  | same |
| G | fused, agl, AHRS        | ~10–15 m bounded | same |
| H | fused, baro, AHRS       | bounded, high drift between fixes | same |

### Long dataset — `isaac-sim-20260625` (9.5 km / ~22 km total path)

| Run | Setup | Expected RMSE | Key expectation |
|-----|-------|---------------|-----------------|
| A-L | flow-odom, agl, GT-att  | ~90 m (0.4%)   | grows linearly with distance |
| C-L | flow-odom, agl, AHRS    | ~394 m (1.75%) | drifts badly past 6 km |
| D-L | flow-odom, baro, AHRS   | > 1 km         | worst-case baseline |
| E-L | fused, agl, GT-att      | **~13 m bounded** | fused clearly beats flow-odom here |
| G-L | fused, agl, AHRS        | **~13–20 m bounded** | the deployable target |

Key conclusions to validate:
- **Rangefinder takes no-GT result from ~12.76% → 1.75% (~7×)** — visible on short dataset
- **Fused beats flow-odom only beyond ~6 km** — clearly visible on long dataset (22.5 km path, crossover in the plot)
- **DSMAC bounds error permanently at ~13 m** — confirmed by flat error curve on long dataset (flow-odom alone reaches ~394 m)
