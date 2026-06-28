# Test Plan — DSMAC Extractor Comparison on Long Dataset

**Conda env:** `drone`  
**Scripts to copy into this folder:** `dsmac_match.py`, `fuse_flowodom_dsmac.py`, `flow_odometry.py`

---

## Context

Exp01–04 established:
- DSMAC with ALIKED+LightGlue: 55% match rate, 13.8 m median abs-error (short dataset, 60-frame sample, Exp03)
- Best deployable trajectory: G-L = 35.7 m RMSE, 57.0 m final over 22 km (AHRS+compass, rej=150, skip=13, Exp02)
- Long dataset (`isaac-sim-20260625`) has no per-fix accuracy benchmark — only trajectory-level results exist

DSMAC has only ever used ALIKED+LightGlue. The `lightglue` pip package ships four extractor variants
with trained weights: **SuperPoint**, **SIFT**, **DISK**, **ALIKED**. This experiment benchmarks all
four on the long dataset to:
1. Establish a per-fix accuracy baseline for the long flight
2. Find whether a different extractor improves match rate or position accuracy
3. Confirm the real-time fix budget (< 2.4 s) holds for all extractors on the RTX 5090

XFeat excluded: no native LightGlue weights for XFeat descriptors; worst RPE (4.62%) in Exp04 tracker comparison.

---

## Dataset

**Long dataset only:** `_in/isaac-sim-20260625` (76.9 min, 22 407 m, ~49 m mean alt, 67 863 frames).

The short dataset already has ALIKED+LightGlue benchmarked in Exp03. The long dataset covers wider
geographic area and is the primary deployment target.

---

## Extractors

| ID | Extractor | Matcher | Notes |
|----|-----------|---------|-------|
| **aliked** | ALIKED | LightGlue | **Baseline** — current DSMAC default |
| superpoint | SuperPoint | LightGlue | Fastest; MegaDepth-trained weights cover aerial scenes |
| disk | DISK | LightGlue | Best on clean baselines; may suit consistent 49 m cruise alt |
| sift | SIFT | LightGlue | Domain-agnostic classical descriptor; most robust to sim→satellite gap |

---

## Experiment 1: Per-fix Accuracy and Latency (200-frame sample)

Sample **200 frames evenly spaced** across the long flight (every ~340 frames) and run each extractor
against the cached ortho tiles. Covers the full geographic range without running all 67 863 frames.

For each extractor × frame: tile fetch (cached) → detect → LightGlue match → RANSAC homography
→ compare fix to `geo.csv` → abs-position error.

**Metrics per extractor:**

| Metric | How |
|--------|-----|
| Match rate (%) | frames with valid RANSAC homography / 200 |
| Median abs-position error (m) | over matched frames |
| P90 abs-position error (m) | tail accuracy |
| Max abs-position error (m) | worst fix |
| Extract latency mean / P95 (ms) | timed over 200 frames |
| Match latency mean / P95 (ms) | matcher only |
| Total fix latency mean / P95 (ms) | extract + match + RANSAC |

**Baseline targets (Exp03 short dataset, ALIKED+LG):**

| Metric | Short-dataset value | Goal for long |
|--------|--------------------|-|
| Match rate | 55% | establish long-dataset baseline; beat if possible |
| Median abs error | 13.8 m | ≤ 13.8 m |
| Total fix latency mean | 44.1 ms | within 2 400 ms budget |

---

## Experiment 2: Trajectory-level Accuracy (Best Extractor vs ALIKED Baseline)

Run the full fused pipeline on the complete long dataset with:
1. ALIKED+LG (re-run to confirm G-L baseline with current scripts)
2. The extractor with the best match rate × median-error product from Exp1

Settings: `--attitude ahrs_compass --reject 150 --skip_below 13 --blend 0.8 --fix_every 30`

**Metrics:**

| Metric | G-L baseline (Exp02) | Target |
|--------|---------------------|--------|
| Flow-odom RMSE | 80.2 m | — (unchanged) |
| Fused RMSE | 35.7 m | ≤ 35.7 m |
| Fused final | 57.0 m | ≤ 57.0 m |
| DSMAC rate | 68/68 (100%) | ≥ 68/68 |

If ALIKED remains best from Exp1, skip Exp2 and record as confirmed long-dataset baseline.

---

## Script Changes Required

`dsmac_match.py` currently hardcodes ALIKED. Add:
- `--extractor {aliked,superpoint,disk,sift}` flag (default: `aliked`)
- `--n_sample N` flag to run on N evenly-spaced frames instead of all frames
- `--benchmark` mode already exists (from Exp03) — reuse

`fuse_flowodom_dsmac.py`: add `--extractor` flag to pass through to the DSMAC sub-call.

---

## Commands

```bash
# Exp 1 — per-fix comparison, 200-frame sample, run once per extractor
conda run -n drone python experiment/05/dsmac_match.py \
    --dir _in/isaac-sim-20260625 --extractor aliked \
    --n_sample 200 --benchmark

conda run -n drone python experiment/05/dsmac_match.py \
    --dir _in/isaac-sim-20260625 --extractor superpoint \
    --n_sample 200 --benchmark

conda run -n drone python experiment/05/dsmac_match.py \
    --dir _in/isaac-sim-20260625 --extractor disk \
    --n_sample 200 --benchmark

conda run -n drone python experiment/05/dsmac_match.py \
    --dir _in/isaac-sim-20260625 --extractor sift \
    --n_sample 200 --benchmark

# Exp 2 — full trajectory with best extractor (replace aliked if a better one found)
conda run -n drone python experiment/05/fuse_flowodom_dsmac.py \
    --dir _in/isaac-sim-20260625 --attitude ahrs_compass \
    --reject 150 --skip_below 13 --extractor aliked
```

---

## Expected Results

| Extractor | Match rate (est.) | Median error (est.) | Latency (est.) |
|-----------|-------------------|---------------------|----------------|
| ALIKED (baseline) | ~50–60% | ~13–15 m | ~44 ms |
| SuperPoint | ~45–55% | ~14–17 m | ~25–35 ms |
| DISK | ~55–65% | ~12–15 m | ~60–90 ms |
| SIFT | ~45–55% | ~13–17 m | ~50–70 ms |

DISK is the most likely to improve median error given the consistent cruise altitude and broader
texture variety of the long flight. SuperPoint is the most likely to save latency. ALIKED may
remain the best overall given its track record on degraded/nadir footage.

---

## Open Items Carried from Exp04

- **Compass noise sensitivity:** `mag_noise_deg > 0` — quantify degradation from real magnetometer noise
- **Short-flight tilt gap:** AHRS roll/pitch scale error (1.31) — structural fix requires rangefinder or multi-cam
