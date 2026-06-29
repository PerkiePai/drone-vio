# Blend Autotune — Validation Results

Date: 2026-06-29  
Runs: 4 (2 datasets × baseline vs autotune)

---

## Datasets

| Dataset | Label | Path | Duration | Frames |
|---|---|---|---|---|
| `_in/isaac-sim-20260624_2337` | **short** | 1 773 m | 7.5 min | 14 070 |
| `_in/isaac-sim-20260625` | **long** | 22 407 m | 76.9 min | 67 861 |

Both use `agl_cache.npz` (true AGL), AHRS+compass attitude, SIFT+LightGlue DSMAC, stride=5, scale=0.5.

---

## Results

### Short dataset (isaac-sim-20260624\_2337 — 1 773 m)

| Run | DSMAC fixes (acc/att) | RMSE | RMSE % path | Final error | Final % |
|---|---|---|---|---|---|
| Baseline `--blend 0.8` | 83 / 83 (100%) | **11.6 m** | 0.65% | 5.0 m | 0.28% |
| Autotune `--autotune` | 83 / 83 (100%) | **7.7 m** | 0.43% | 1.2 m | 0.07% |
| **delta autotune vs baseline** | — | **-34%** | | **-76%** | |

### Long dataset (isaac-sim-20260625 — 22 407 m)

| Run | DSMAC fixes (acc/att) | RMSE | RMSE % path | Final error | Final % |
|---|---|---|---|---|---|
| Baseline `--blend 0.8` | 426 / 426 (100%) | **15.1 m** | 0.067% | 0.8 m | 0.004% |
| Autotune `--autotune` | 427 / 427 (100%) | **16.2 m** | 0.072% | 5.4 m | 0.024% |
| **delta autotune vs baseline** | — | **+7%** | | **+575%** | |

---

## Analysis

**Short dataset**: autotune wins clearly — RMSE drops 34%, final error drops 76%.
The Kalman-style blend adapts aggressively during the first 6 warmup fixes (blend=1.0 snap),
then moderates once `dsmac_std` is frozen. The inlier-confidence scale further down-weights
uncertain fixes. On a short flight with variable altitude (1-145 m AGL), this adaptive
trust outperforms a fixed blend.

**Long dataset**: baseline wins marginally — autotune RMSE is 7% higher and final error
worse. Why:
- The long dataset has 426-427 fixes all accepted (100% rate), meaning DSMAC is highly
  consistent over 22 km. A fixed blend=0.8 is already near-optimal when fixes are clean.
- The warmup phase (first 6 fixes, blend=1.0) over-corrects early; on a 22 km flight,
  that early error propagates enough to slightly raise RMSE.
- `dsmac_std` is frozen from warmup residuals that may not represent the full flight's
  noise distribution.

**Conclusion**: autotune is better suited for shorter flights or variable-quality DSMAC
(e.g. mixed terrain, altitude sweeps). For long flights with consistent, high-quality
DSMAC, the fixed blend=0.8 remains the safe choice. Both methods hold well under
0.1% RMSE/path on the long dataset.

---

## Output plots

| File | Content |
|---|---|
| `_out/pipeline_short_baseline.png` | Short dataset, blend=0.8 |
| `_out/pipeline_short_autotune.png` | Short dataset, autotune |
| `_out/pipeline_long_baseline.png` | Long dataset, blend=0.8 |
| `_out/pipeline_long_autotune.png` | Long dataset, autotune |
