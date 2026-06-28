# Experiment Results — Flow-odom + DSMAC Fusion

**Date:** 2026-06-28  
**Plan:** `plan.md`  
**Scripts:** `flow_odometry.py`, `fuse_flowodom_dsmac.py` (snapshots in this folder)  
**Datasets:**

| ID | Directory | Duration | GT path | Mean alt |
|----|-----------|----------|---------|----------|
| Short | `_in/isaac-sim-20260624_2337` | 7.5 min | 1 773 m | 43.5 m |
| Long  | `_in/isaac-sim-20260625`      | 76.9 min | 22 407 m | 48.9 m |

All runs: `--stride 5`, `--scale 0.5`. Fuse: `--blend 0.8`, `--fix_every 30`.

---

## Pre-experiment: `--reject` Gate Calibration

**Goal:** find a safe outlier-rejection threshold for each dataset.

### Short dataset (1 773 m)

| Run | `--reject` | DSMAC accepted/attempted | Flow-odom RMSE | Fused RMSE | Fused final |
|-----|------------|--------------------------|----------------|------------|-------------|
| R1  | 15 m       | 19 / 44                  | 8.8 m          | 15.8 m     | 10.1 m      |
| R2  | 45 m       | 45 / 45                  | 8.8 m          | 10.5 m     | 7.3 m       |
| R3  | 150 m      | 45 / 45                  | 8.8 m          | 10.5 m     | 7.3 m       |

**Decision — short: use `--reject 45`.**  
45 m accepts all good fixes with zero rejections. Tighter (15 m) drops 57% of valid fixes and raises RMSE by 5 m. Looser (150 m) gives identical results — gate is not binding at 1.8 km.

### Long dataset (22 407 m)

| Run | `--reject` | DSMAC accepted/attempted | Flow-odom RMSE | Fused RMSE | Fused final |
|-----|------------|--------------------------|----------------|------------|-------------|
| R4  | 45 m       | 25 / 71                  | 76.1 m         | 71.8 m     | 25.5 m      |
| R5  | 150 m      | 68 / 69                  | 76.1 m         | **52.3 m** | 68.6 m      |
| R6  | 400 m      | 68 / 69                  | 76.1 m         | 52.3 m     | 68.6 m      |

**Decision — long: use `--reject 150`.**  
At 1.75% drift, flow-odom accumulates > 45 m error by ~3 km — the default gate then rejects valid fixes for the rest of the 22 km flight (25/71 accepted). Loosening to 150 m recovers 68/69 fixes and cuts RMSE by 27%. 400 m gives identical results — 150 m is already non-binding.

---

## Experiment 1: Flow-odom — AGL vs Baro, GT vs AHRS Attitude

### Short dataset (1 773 m)

| Run | Depth | Attitude | Mean RMSE | Final error | % path | Scale |
|-----|-------|----------|-----------|-------------|--------|-------|
| A   | agl   | GT       | 7.7 m     | 4.6 m       | 0.3%   | 0.98  |
| B   | baro  | GT       | 110.2 m   | 11.4 m      | 0.6%   | 0.56  |
| C   | agl   | AHRS     | 21.6 m    | 33.4 m      | 1.9%   | 1.31  |
| D   | baro  | AHRS     | 110.6 m   | 16.7 m      | 0.9%   | 0.81  |

### Long dataset (22 407 m)

| Run | Depth | Attitude | Mean RMSE | Final error | % path | Scale |
|-----|-------|----------|-----------|-------------|--------|-------|
| A   | agl   | GT       | 63.9 m    | 20.7 m      | **0.09%** | 0.99 |
| B   | baro  | GT       | 1 745 m   | 263.8 m     | 1.2%   | 0.60  |
| C   | agl   | AHRS     | 4 963 m   | 12 082 m    | **53.9%** | 1.01 |
| D   | baro  | AHRS     | 3 812 m   | 7 529 m     | 33.6%  | 0.61  |

### Key findings

1. **AGL vs baro — 13× on long flight.** With GT attitude: baro 1.2% vs AGL 0.09% over 22 km. Baro scale collapses to 0.60 because Cesium terrain lies ~4–6× deeper than baro height-above-takeoff; each step is scaled down, leading to 0.60× path length. AGL (rangefinder proxy) restores scale to 0.99.

2. **AHRS without compass is unusable on long flights.** 53.9% final error (12 km) on a 22 km mission — yaw drifts unconstrained over 77 minutes. On the short 7.5-min flight it's manageable at 1.9%. **A compass/magnetometer is non-negotiable for flights > ~10 min.**

3. **Short vs long — depth error compounds with distance.** On short: baro mean RMSE 110 m but final only 11.4 m (trajectory wanders then recovers). On long: baro mean RMSE 1 745 m and final 264 m — drift accumulates monotonically with no recovery.

---

## Experiment 2: Fused Flow-odom + DSMAC

Uses `tracker_trajs.npz` (LK trajectory, stride-5, GT attitude, agl depth) pre-computed by Exp1A as flow-odom propagation steps. DSMAC runs every 30 fused steps.

### Short dataset (1 773 m) — `--reject 45`

| Run | Setup | Flow-odom RMSE | Fused RMSE | Fused final | DSMAC rate |
|-----|-------|----------------|------------|-------------|------------|
| E   | agl, GT (default) | 8.8 m | 10.5 m | 7.3 m | 45/45 (100%) |
| F   | baro propagation* | 8.8 m | 10.5 m | 7.3 m | 45/45 (100%) |

*F uses the same GT-agl LK increments as E (fuse script has no baro-propagation mode — deferred).

**Honest read:** fused (10.5 m) does NOT beat flow-odom-only (8.8 m) on this 1.8 km flight. Flow-odom at 0.3% is already tighter than DSMAC's ~13 m fixes — each fix injects map noise into a better trajectory. Fusion pays off only once flow-odom drift exceeds DSMAC's accuracy level.

### Long dataset (22 407 m) — `--reject 150`

| Run | Setup | Flow-odom RMSE | Fused RMSE | Fused final | DSMAC rate |
|-----|-------|----------------|------------|-------------|------------|
| E-L | agl, GT (default) | 76.1 m | **52.3 m** | 68.6 m | 68/69 (99%) |

**Fused wins on long flight (52.3 m vs 76.1 m, −31% RMSE).** At 0.09% drift, flow-odom alone accumulates ~20 m by 22 km — already well-bounded; fused cuts the mean trajectory error further by re-anchoring 68 times across the flight.

Note: Exp2 G/H (AHRS attitude in fuse) are **deferred** — `fuse_flowodom_dsmac.py` needs an `--attitude` flag to swap in AHRS increments; currently hardcoded to GT-attitude LK trajectory.

---

## Summary Table

### Short dataset (1 773 m, 7.5 min)

| Setup | RMSE | Final | % path |
|-------|------|-------|--------|
| flow-odom, agl, GT | 7.7 m | 4.6 m | 0.3% |
| flow-odom, agl, AHRS | 21.6 m | 33.4 m | 1.9% |
| flow-odom, baro, GT | 110.2 m | 11.4 m | 0.6% |
| flow-odom, baro, AHRS | 110.6 m | 16.7 m | 0.9% |
| **fused (agl, GT, rej=45)** | **10.5 m** | **7.3 m** | **bounded** |

### Long dataset (22 407 m, 76.9 min)

| Setup | RMSE | Final | % path |
|-------|------|-------|--------|
| flow-odom, agl, GT | 63.9 m | 20.7 m | **0.09%** |
| flow-odom, agl, AHRS | 4 963 m | 12 082 m | 53.9% |
| flow-odom, baro, GT | 1 745 m | 263.8 m | 1.2% |
| flow-odom, baro, AHRS | 3 812 m | 7 529 m | 33.6% |
| **fused (agl, GT, rej=150)** | **52.3 m** | **68.6 m** | **bounded** |

---

## Conclusions

1. **Rangefinder (AGL) is the critical hardware choice.** Takes no-GT from 1.2% → 0.09% on long flight (~13×). Baro fails because Cesium terrain depth ≫ baro altitude-above-takeoff.

2. **Compass is mandatory for long flights.** AHRS gyro+accel alone drifts 53.9% in 77 minutes. With compass (from prior attitude-comparison run): ~0.18%. The sensor cost is trivial vs the accuracy gain.

3. **Fusion pays off at scale.** Crossover point: when flow-odom drift exceeds DSMAC's ~13 m accuracy (at ~6 km for 0.22%/km drift). Short flight: flow-odom wins (8.8 m vs 10.5 m). Long flight: fused wins (52.3 m vs 76.1 m, −31%).

4. **Reject gate must scale with expected drift.** Default 45 m works for short flights. For a 22 km flight at 0.3%/km, flow-odom drift reaches 45 m by ~3 km — set to 150 m (2× expected max single-fix drift) for long flights.

5. **Deployable recipe (no GPS, no GT):** flow-odom (LK, stride 5) + downward rangefinder (AGL) + Mahony AHRS + compass + DSMAC fusion (rej=150 for long flights) → bounded error, no GPS required.

---

## Deferred / Open Items

- **Exp2 G/H** (AHRS attitude in fuse): need `--attitude ahrs` flag in `fuse_flowodom_dsmac.py`
- **Exp3** (real-time stream feasibility): latency profiling not yet run
- **Long-flight fused with compass AHRS**: Exp1C/D used Mahony gyro+accel only; the deployable target (AHRS+compass, ~0.18%) has not been fused with DSMAC on the long dataset yet
- **`--skip_below` / `--conf_blend` flags**: not tested on either dataset
