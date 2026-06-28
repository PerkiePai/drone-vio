# Test Plan — AHRS+Compass Attitude in Fused Pipeline

## Context

Experiment 01 established:
- AGL (rangefinder) is critical: 0.09% drift vs 1.2% baro on 22 km flight
- AHRS **without compass** is unusable beyond ~10 min: 53.9% final error from unconstrained yaw drift
- Fusion beats flow-odom-only at scale: −31% RMSE on long flight when attitude is correct

All Exp01 "AHRS" runs used **gyro + accelerometer only** (Mahony, no magnetometer).
The claimed deployable result (AHRS+compass ~0.18%) has **never been tested in the fused pipeline**.

The `fuse_flowodom_dsmac.py` script is also currently hardcoded to GT-attitude LK increments —
it has no `--attitude` flag to swap in AHRS increments. That must be added before G/H runs are possible.

---

## Datasets

Same as Exp01.

| ID | Directory | Duration | GT path | Mean alt |
|----|-----------|----------|---------|----------|
| Short | `_in/isaac-sim-20260624_2337` | 7.5 min | 1 773 m | 43.5 m |
| Long  | `_in/isaac-sim-20260625`      | 76.9 min | 22 407 m | 48.9 m |

All runs: `--stride 5`, `--scale 0.5`.

---

## Prerequisites (implementation before running)

### 1. Add `--attitude` flag to `fuse_flowodom_dsmac.py`

The fuse script must accept `--attitude [gt|ahrs]`. When `ahrs`, it loads
the AHRS-attitude LK trajectory (`tracker_trajs_ahrs.npz`) instead of the
GT-attitude one (`tracker_trajs.npz`).

This requires `flow_odometry.py` to save a separate trajectory file when run
with `--attitude ahrs`. Check whether Exp01 already cached `tracker_trajs_ahrs.npz`
in `_in/isaac-sim-20260625/` — if so, only the fuse script needs updating.

### 2. Add compass to Mahony AHRS in `flow_odometry.py`

The existing Mahony implementation uses gyro + accel only. Add magnetometer
fusion using the simulated mag data from Isaac Sim (if available in `imu.csv`)
or a synthetic north vector from `geo.csv` heading. The PI correction term
for yaw should mirror the existing roll/pitch correction using the mag vector.

If Isaac Sim does not output magnetometer data, synthesize it from GT yaw as
a "perfect compass" baseline first — this isolates the compass benefit without
needing real mag noise.

---

## Experiment 1: AHRS+Compass vs AHRS-only vs GT (flow-odom)

**Goal:** confirm that adding compass to Mahony closes the gap from 53.9% → ~0.18%,
and establish the compass baseline for fusion.

| Run | Attitude | Depth | Expected long RMSE |
|-----|----------|-------|--------------------|
| A   | GT       | agl   | ~64 m (0.09%) — Exp01 reference |
| C   | AHRS (no compass) | agl | ~4 963 m (53.9%) — Exp01 reference |
| C2  | AHRS + compass    | agl | target ~0.18% (~40 m on 22 km) |

Run C2 (short):
```bash
conda run -n cv python frontend/flow-odom/flow_odometry.py \
    --dir _in/isaac-sim-20260624_2337 --depth agl --stride 5 --attitude ahrs_compass
```

Run C2 (long):
```bash
conda run -n cv python frontend/flow-odom/flow_odometry.py \
    --dir _in/isaac-sim-20260625 --depth agl --stride 5 --attitude ahrs_compass
```

**Metrics:**

| Metric | How |
|--------|-----|
| RMSE (m) | vs `geo.csv` GT |
| Final error (m) | last-frame |
| Yaw drift (°) | log estimated yaw vs GT yaw over time |
| Scale factor | median estimated/true step |

**Key plot:** yaw error over time for AHRS-only vs AHRS+compass vs GT — should show
compass holds yaw bounded while AHRS-only diverges monotonically.

---

## Experiment 2: Fused Pipeline with AHRS+Compass (Deferred G/H from Exp01)

**Goal:** benchmark the full deployable recipe end-to-end on both datasets.

| Run | Attitude | Depth | Reject | Dataset |
|-----|----------|-------|--------|---------|
| G   | AHRS+compass | agl | 45  | Short |
| G-L | AHRS+compass | agl | 150 | Long  |
| H   | AHRS+compass | baro | 45 | Short |
| H-L | AHRS+compass | baro | 150 | Long |

Run G (short):
```bash
conda run -n cv python frontend/geoloc/fuse_flowodom_dsmac.py \
    --dir _in/isaac-sim-20260624_2337 --attitude ahrs_compass --reject 45
```

Run G-L (long):
```bash
conda run -n cv python frontend/geoloc/fuse_flowodom_dsmac.py \
    --dir _in/isaac-sim-20260625 --attitude ahrs_compass --reject 150
```

**Metrics:** same as Exp01 Exp2.

**What to look for:**

- **G vs E (GT attitude):** gap should be small if compass holds yaw bounded.
  If RMSE(G) ≈ RMSE(E), compass AHRS is a viable GT substitute — deployable confirmed.
- **G-L vs E-L:** the critical number. E-L was 52.3 m. G-L should be close.
  If G-L > 2× E-L, compass is insufficient and a different yaw source is needed.
- **H / H-L (baro):** expected to be much worse than G/G-L; confirms rangefinder remains the
  primary hardware dependency even with good attitude.

---

## Experiment 3: `--skip_below` and `--conf_blend` on Short Flight

**Goal:** test whether the two deferred flags can make fused beat flow-odom-only on the
short 1.8 km flight (Exp01 showed fused lost: 10.5 m vs 8.8 m).

Hypothesis: DSMAC fixes at ~13 m accuracy corrupt a trajectory already tighter than 13 m.
`--skip_below 13` defers fixes until flow-odom drift exceeds DSMAC accuracy.

| Run | Flags | Expected |
|-----|-------|----------|
| E   | default (Exp01 reference) | 10.5 m |
| E2  | `--skip_below 13` | should be ≤ 8.8 m (no bad fixes applied) |
| E3  | `--conf_blend` | fix pull weighted by inlier count |
| E4  | `--skip_below 13 --conf_blend` | combined |

Run E2:
```bash
conda run -n cv python frontend/geoloc/fuse_flowodom_dsmac.py \
    --dir _in/isaac-sim-20260624_2337 --reject 45 --skip_below 13
```

**Crossover check:** also run E2-L (long dataset) to confirm `--skip_below` does not
hurt the long flight where fixes are genuinely needed.

---

## Summary Table (targets)

### Short dataset (1 773 m)

| Run | Setup | Exp01 RMSE | Target RMSE |
|-----|-------|------------|-------------|
| A   | flow-odom, agl, GT | 7.7 m | reference |
| C   | flow-odom, agl, AHRS-only | 21.6 m | reference |
| C2  | flow-odom, agl, AHRS+compass | — | ~8–10 m |
| E   | fused, GT, rej=45 | 10.5 m | reference |
| E2  | fused, GT, rej=45, skip_below=13 | — | ≤ 8.8 m |
| G   | fused, AHRS+compass, rej=45 | — | ~10–13 m |

### Long dataset (22 407 m)

| Run | Setup | Exp01 RMSE | Target RMSE |
|-----|-------|------------|-------------|
| A-L | flow-odom, agl, GT | 63.9 m | reference |
| C-L | flow-odom, agl, AHRS-only | 4 963 m | reference |
| C2-L | flow-odom, agl, AHRS+compass | — | ~40 m |
| E-L | fused, GT, rej=150 | 52.3 m | reference |
| G-L | fused, AHRS+compass, rej=150 | — | **~55–70 m** |

G-L is the primary deliverable: fused + deployable sensors, no GT, no GPS.

---

## Open Items Carried from Exp01

- Exp3 (real-time stream feasibility / latency profiling) — still not run; defer to Exp03
- DSMAC match rate breakdown by terrain type — still not done
- `--skip_below` / `--conf_blend` addressed here in Exp3 above
