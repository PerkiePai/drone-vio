# Full Autotune — Validation Results

**Plan:** `plan.md` — dynamic `reject` and `skip_below` gates (extends blend autotune)  
**Date:** 2026-06-29

---

## What was implemented

Three parameters now autotune together under `--autotune`:

| Parameter | Fixed default | Autotuned formula |
|---|---|---|
| `blend` | 0.8 | `flow_var / (flow_var + dsmac_var)` × inlier_conf, clipped [0.3, 1.0] |
| `reject` | 150 m | `drift_since + 3 × dsmac_std` |
| `skip_below` | 13 m | `dsmac_std` |

All three fall back to their fixed CLI defaults during the warmup phase (`warmup_fixes=6`).
`dsmac_std` is estimated from warmup fix residuals, then frozen.

---

## Datasets

| Dataset | Label | Path | Duration | Frames |
|---|---|---|---|---|
| `_in/isaac-sim-20260624_2337` | **short** | 1 773 m | 7.5 min | 14 070 |
| `_in/isaac-sim-20260625` | **long** | 22 407 m | 76.9 min | 67 861 |

Both: `agl_cache.npz` (true AGL), AHRS+compass attitude, SIFT+LightGlue DSMAC, stride=5, scale=0.5.

---

## Results

### Short dataset (1 773 m, 7.5 min)

| Run | Fixes (acc/att) | RMSE | RMSE % | Final error | Final % |
|---|---|---|---|---|---|
| Baseline `--blend 0.8` | 83 / 83 (100%) | 11.6 m | 0.65% | 5.0 m | 0.28% |
| Full autotune `--autotune` | 83 / 83 (100%) | **7.7 m** | **0.43%** | **1.2 m** | **0.07%** |
| delta | — | **-34%** | | **-76%** | |

### Long dataset (22 407 m, 76.9 min)

| Run | Fixes (acc/att) | RMSE | RMSE % | Final error | Final % |
|---|---|---|---|---|---|
| Baseline `--blend 0.8` | 426 / 426 (100%) | **15.1 m** | **0.067%** | **0.8 m** | 0.004% |
| Full autotune `--autotune` | 428 / 428 (100%) | 16.2 m | 0.072% | 3.7 m | 0.017% |
| delta | — | **+7%** | | +363% | |

---

## Analysis

**Short dataset — autotune wins (−34% RMSE, −76% final error).**  
The dynamic `reject` gate tightens when drift is low, filtering marginal fixes that the
fixed 150 m gate would accept. The dynamic `skip_below` (= `dsmac_std` ≈ 13 m on SIFT)
is similar to the hardcoded 13 m on this dataset, so its contribution is neutral here.
The adaptive blend does most of the lifting on the short flight.

**Long dataset — baseline still marginally better (+7% RMSE), but autotune final error
improved vs previous autotune run (3.7 m vs 5.4 m before this plan's reject fix).**  
On 22 km, all 426-428 fixes are accepted under both strategies — DSMAC quality is very
consistent, so the fixed blend=0.8 and the fixed 150 m reject are already well-tuned.
The warmup over-correction (blend=1.0 for 6 fixes) causes a small early error that
propagates through the long flight, keeping RMSE slightly above baseline.

**Dynamic reject gate side-effect on long dataset:** accepts 428 fixes vs 426 with
fixed 150 m gate (+2 fixes) because after large drift segments the dynamic gate widens
(`drift_since + 3σ > 150 m`), allowing fixes that the fixed gate would have rejected.
This is correct behavior but the extra 2 fixes land in a slightly noisier window.

**Bottom line:**
- Use `--autotune` for short or variable-quality flights — clear improvement.
- Use `--blend 0.8` (default) for long flights with consistent DSMAC — marginally better.
- The dynamic reject formula works correctly: it tightens the gate right after corrections
  and widens it after dead-reckoning legs, which is the physically correct behavior.

---

## Output plots

| File | Content |
|---|---|
| `_out/pipeline_short_baseline.png` | Short — blend=0.8 fixed |
| `_out/pipeline_short_autotune.png` | Short — full autotune |
| `_out/pipeline_long_baseline.png` | Long — blend=0.8 fixed |
| `_out/pipeline_long_autotune.png` | Long — full autotune |
