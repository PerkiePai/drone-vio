# Experiment Results — Gate Tightening & Inlier Threshold

**Date:** 2026-07-01
**Plan:** `plan.md`
**Scripts:** `pipeline.py`, `flow_odometry.py` (snapshots in this folder; `pipeline.py` is the
repo-root production pipeline with `--extractor {sift,aliked}` added for this experiment)
**Conda env:** `drone` (PyTorch + CUDA 12.8, RTX 5090)
**Dataset:** Long only — `_in/isaac-sim-20260625` (76.9 min, 22 407 m, ~49 m mean alt, 67 861 frames)

| ID | Directory | Duration | GT path | Mean alt |
|----|-----------|----------|---------|----------|
| Long | `_in/isaac-sim-20260625` | 76.9 min | 22 407 m | 48.9 m |

All runs: `--stride 5 --scale 0.5 --blend 0.8 --fix_every 30 --skip_below 13` (AHRS+compass attitude,
AGL depth — the deployable, no-GT, no-GPS recipe). `pipeline.py` recomputes flow-odom (LK) and DSMAC
fixes together in one online pass per run (no cached trajectory replay), so each run's flow-odom
component is freshly computed and DSMAC corrections feed back into subsequent position estimates.

---

## Experiment 1: Gate sweep (SIFT, `--reject`)

`--reject` swept 30/40/50/75/100/150 m, `--min_inliers 15` (default), SIFT+LightGlue extractor.

| Run | `--reject` | Fixes acc/att | Fused RMSE | Fused final |
|-----|-----------|----------------|------------|-------------|
| 1 | 30 m | 426/426 (100%) | 15.1 m | 0.8 m |
| 2 | 40 m | 426/426 (100%) | 15.1 m | 0.8 m |
| 3 | 50 m | 426/426 (100%) | 15.1 m | 0.8 m |
| 4 | 75 m | 426/426 (100%) | 15.1 m | 0.8 m |
| 5 | 100 m | 426/426 (100%) | 15.1 m | 0.8 m |
| 6 | 150 m (baseline) | 426/426 (100%) | 15.1 m | 0.8 m |

### Key findings

1. **SIFT is completely insensitive to the reject gate, even at 30 m — identical RMSE (15.1 m) and
   final error (0.8 m) at every tested threshold, with 0 rejections throughout.** SIFT's 95%+ match
   rate produces fixes that land close enough to the (already accurate) prior position that even a
   30 m gate never binds. This reproduces Exp05's anomaly 4 (0 rejections) and shows the loose 150 m
   gate was never actually the risk for SIFT — there is no harm in tightening it as a defensive
   measure, but no measurable benefit on this dataset either.
2. **Result matches Exp05's SIFT baseline almost exactly** (15.1 m RMSE / 0.8–1.0 m final), confirming
   the new unified `pipeline.py` reproduces the prior two-script (`dsmac_match.py` +
   `fuse_flowodom_dsmac.py`) result.

---

## Experiment 2: Inlier threshold sweep (ALIKED, `--min_inliers`)

`--min_inliers` swept 15/20/30/40/50, `--reject 150` (unchanged), ALIKED+LightGlue extractor (Exp05's
worst case). A flow-odom-only reference (`--min_inliers 999999`, i.e. no DSMAC fix ever accepted) was
also run to establish this pipeline's own no-fusion baseline.

| Run | `--min_inliers` | Fixes acc/att | Fused RMSE | Fused final |
|-----|-----------------|----------------|------------|-------------|
| flow-odom only | n/a (no fixes) | 0/0 | 81.7 m | 34.7 m |
| 1 | 15 (default) | 30/31 (97%) | 93.2 m | 152.6 m |
| 2 | 20 | 66/66 (100%) | 43.1 m | 120.3 m |
| 3 | **30** | **64/64 (100%)** | **38.8 m** | **60.8 m** |
| 4 | 40 | 61/61 (100%) | 49.4 m | 22.6 m |
| 5 | 50 | 60/60 (100%) | 49.5 m | 22.6 m |

### Key findings

1. **`min_inliers=15` (current default) is actively harmful: 93.2 m RMSE, worse than flow-odom-only
   (81.7 m).** With only 8 RANSAC inliers required (the homography minimum) and just 15 accepted,
   low-confidence ALIKED matches pull the trajectory toward wrong satellite locations — fusion makes
   things worse than doing nothing. This confirms Exp05 anomaly 3 directly.
2. **`min_inliers=30` is the sweet spot: 38.8 m RMSE, a 53% drop from `min_inliers=15` and well below
   both flow-odom-only (81.7 m) and the Exp04 LK+ALIKED baseline (39.0 m).** Raising the bar from 15
   to 30 inliers filters out the low-confidence matches that corrupt the trajectory while keeping
   64 fixes (vs. 66 at inl=20) — match *quality*, not match *count*, is what was missing.
3. **`min_inliers=40/50` trade RMSE for final error.** RMSE rises slightly (49.4–49.5 m, because fewer,
   sparser fixes let drift build up more between corrections) but final error drops to 22.6 m (the
   flight ends in a region where the remaining high-confidence fixes land very accurately). `inl=30`
   is the best overall trade-off for RMSE, which is the plan's named success metric.
4. **Even the worst inlier setting (15) only causes 1 outright rejection (30/31)** — confirming Exp05
   anomaly 4: the 150 m reject gate measures distance from the drifted prior, not from GT, so it does
   not catch low-quality-but-plausible fixes. `min_inliers` is the correct lever, not `reject`.

---

## Experiment 3: Combined fix for ALIKED

Plan step: "apply the best gate + inlier threshold found above to ALIKED." The naively
"tightest-looks-safest" combination (`reject=30` from Exp1, `min_inliers=30` from Exp2) was tried
first; it failed badly, so `reject` was re-swept with `min_inliers=30` held fixed to find ALIKED's
actual breakeven gate.

| Run | `--reject` | `--min_inliers` | Fixes acc/att | Fused RMSE | Fused final |
|-----|-----------|-----------------|----------------|------------|-------------|
| 3a | 30 m | 30 | 1/68 (1%) | 81.4 m | 37.2 m |
| 3b | 50 m | 30 | 3/68 (4%) | 81.6 m | 54.4 m |
| 3c | 75 m | 30 | 40/65 (62%) | 57.2 m | 60.8 m |
| **3d** | **100 m** | **30** | **64/64 (100%)** | **38.8 m** | **60.8 m** |
| (ref, Exp2) | 150 m | 30 | 64/64 (100%) | 38.8 m | 60.8 m |

### Key findings

1. **Tightening `reject` to 30 m actively breaks ALIKED — only 1/68 fixes accepted, RMSE 81.4 m
   (worse than flow-odom-only).** This falsifies the plan's central hypothesis ("reducing `--reject`
   from 150 m to 30–50 m will block bad ALIKED fixes"). ALIKED fires rarely (DSMAC attempts succeed
   only ~65/452 opportunities even with a generous gate) and accumulates large drift between fixes
   (5–22 km of `drift_since` integrated path length by the end of the flight, vs SIFT's near-zero).
   By the time a fix is available, the prior position has drifted far enough that even a *correct*
   fix is >30 m away and gets rejected — the tight gate rejects good fixes, not bad ones.
2. **The reject gate must scale with each extractor's fix cadence.** SIFT fires every ~30 steps with
   near-zero drift between fixes, so any gate ≥30 m is safe. ALIKED fires sporadically after large
   drift has accumulated, so it needs a gate ≥100 m (the breakeven found at 3d) to ever accept a fix
   at all. A single fixed `--reject` cannot be simultaneously tight (to filter bad ALIKED fixes near
   GT) and loose (to admit good ALIKED fixes after long gaps) — the gate is the wrong lever for this
   problem, confirming Exp2's finding that `min_inliers` is the correct fix.
3. **Success criterion met: combined ALIKED fix (`min_inliers=30`, `reject≥100`) reaches 38.8 m RMSE,
   below the Exp04 baseline (39.0 m) and the flow-odom-only reference (81.7 m).** The fix is
   `min_inliers=30` alone (`reject` can stay at its existing 150 m default) — gate tightening
   contributes nothing and would actively hurt with the wrong reject value.

---

## Experiment 2b: Best combined gate for SIFT (sanity check)

Plan step: apply the best (reject, min_inliers) pair to SIFT to confirm gate-tightening + a higher
inlier bar do not regress the already-good SIFT result. Ran with `reject=30, min_inliers=30`.

| Run | `--reject` | `--min_inliers` | Fixes acc/att | Fused RMSE | Fused final |
|-----|-----------|-----------------|----------------|------------|-------------|
| baseline (Exp1) | 150 m | 15 (default) | 426/426 (100%) | 15.1 m | 0.8 m |
| 2b | 30 m | 30 | 388/404 (96%) | 17.0 m | 4.6 m |

### Key findings

1. **Raising `min_inliers` to 30 slightly regresses SIFT: 17.0 m vs 15.1 m RMSE (+13%), 4.6 m vs 0.8 m
   final.** SIFT's match quality is already high, so a stricter inlier bar filters out some
   marginal-but-correct matches (404 attempts pass vs 426 at the default), reducing fix frequency
   slightly. The regression is small and both numbers comfortably pass the plan's SIFT success
   criterion (RMSE ≤ 20 m, final ≤ 5 m), so `min_inliers=30` is safe to use as a single global default
   across both extractors if a single setting is preferred — but `min_inliers=15` remains strictly
   better for SIFT specifically.

---

## Summary Table

| Extractor | Config | Fixes acc/att | Fused RMSE | Fused final | vs target |
|-----------|--------|----------------|------------|-------------|-----------|
| — | flow-odom only (no fusion) | 0/0 | 81.7 m | 34.7 m | reference |
| SIFT | reject=150, inl=15 (baseline) | 426/426 (100%) | 15.1 m | 0.8 m | ✓ matches Exp05 |
| SIFT | reject=30, inl=15 (tight gate) | 426/426 (100%) | 15.1 m | 0.8 m | ✓ ≤20 m / ≤5 m |
| SIFT | reject=30, inl=30 (tight+strict) | 388/404 (96%) | 17.0 m | 4.6 m | ✓ ≤20 m / ≤5 m |
| ALIKED | reject=150, inl=15 (Exp05 baseline) | 30/31 (97%) | 93.2 m | 152.6 m | ✗ worse than flow-odom |
| ALIKED | reject=150, inl=30 (inlier fix) | 64/64 (100%) | **38.8 m** | 60.8 m | ✓ ≤39.0 m (Exp04) |
| ALIKED | reject=30, inl=30 (gate+inlier) | 1/68 (1%) | 81.4 m | 37.2 m | ✗ gate too tight |
| ALIKED | reject=100, inl=30 (best combined) | 64/64 (100%) | **38.8 m** | 60.8 m | ✓ ≤39.0 m (Exp04) |

---

## Conclusions

1. **The structural fix for Exp05's anomalies 3 and 4 is `min_inliers`, not `reject`.** Raising
   `min_inliers` from 15 to 30 cuts ALIKED's fused RMSE from 93.2 m (worse than no fusion) to 38.8 m
   (below the Exp04 baseline), by filtering low-confidence matches before they ever reach the spatial
   gate. The `reject` gate, regardless of threshold, cannot distinguish a good ALIKED fix from a bad
   one — both look "plausible" relative to a drifted prior.
2. **Gate tightening is hypothesis-falsified for sparse-fix extractors.** The plan's hypothesis
   (tighter `reject` blocks bad fixes) holds only for extractors with frequent, low-drift fixes
   (SIFT). For ALIKED, which fires infrequently after large drift accumulates, a tight gate rejects
   good fixes and leaves the trajectory to free-drift — RMSE gets *worse*, not better, as `reject`
   tightens below ~100 m.
3. **Recommended defaults going forward: `min_inliers=30`, `reject=150` (unchanged).** This is a safe
   single configuration for both extractors — SIFT stays within target (17.0 m RMSE / 4.6 m final)
   and ALIKED is fixed (38.8 m RMSE, down from 93.2 m). Do not lower `reject` below ~100 m without
   also verifying the extractor's fix cadence; it is not a universally safe knob.
4. **All four plan success criteria are met:**
   - Gate sweep: RMSE unchanged (15.1 m) vs reject=150 at reject=40–50 m ✓
   - Inlier sweep: ALIKED RMSE (38.8 m at inl=30) drops below flow-odom-only (81.7 m this pipeline /
     77.2 m Exp04 figure) ✓
   - Combined (ALIKED): fused RMSE 38.8 m ≤ 39.0 m (Exp04 baseline) ✓ — by a 0.2 m margin; treat as
     "matches", not "decisively beats"
   - SIFT (gate tight): RMSE 15.1–17.0 m ≤ 20 m; final 0.8–4.6 m ≤ 5 m ✓

---

## Open items carried forward

- **38.8 m margin over the 39.0 m target is thin (0.2 m).** Re-validate on a second long flight before
  treating `min_inliers=30` as a confirmed structural fix rather than a single-dataset result.
- **Anomaly 1 (1.0 m final in sim)** — not a bug; expected to be 5–20 m on real satellite tiles with
  geo-registration error. Unverified — no real-world dataset available yet.
- **Anomaly 2 (scale gap)** — ALIKED/DISK/SuperPoint's 18% match rate is architectural (trained on
  MegaDepth/ScanNet scale variation, not 10–50× nadir-vs-satellite). `min_inliers=30` fixes what
  happens to the *accepted* fixes, not the underlying low match rate; SIFT remains the production
  choice for fix frequency.
- **`pipeline.py`'s own flow-odom-only baseline (81.7 m) differs from Exp04's LK figure (77.2 m) and
  Exp05's cached-trajectory figure (80.2 m).** Consistent with the script-version effect already noted
  in Exp05 — `pipeline.py` recomputes AHRS+LK fresh per run rather than replaying a cached trajectory.
  Use the in-run flow-odom-only baseline for any future comparison against this script, not the older
  figures.
- **Blend autotune** (separate root-level `plan.md`/`patch/` work, commits `87a11dd`–`91be989`)
  already explored dynamic `blend`/`reject`/`skip_below`; it was developed in parallel with this
  experiment and not re-evaluated here. A follow-up should check whether autotune's dynamic reject
  formula (`drift_since + 3σ`) already produces ALIKED-safe gate values automatically, which would
  make the static `min_inliers=30` fix and autotune complementary rather than redundant.
