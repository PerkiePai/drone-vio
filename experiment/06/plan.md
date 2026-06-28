# Experiment 06 — Gate Tightening & Inlier Threshold

## Goal

Verify the four anomalies identified in Exp05 and apply the structural fixes for
anomalies 3 and 4: tighten the rejection gate and add a per-fix minimum inlier
count threshold so that low-confidence ALIKED fixes cannot corrupt the trajectory.

---

## Anomaly recap (from Exp05 plots)

| # | Anomaly | Root cause |
|---|---------|------------|
| 1 | SIFT 1.0 m final error | Sim has no geo-registration error; dense SIFT fixes reach the endpoint — sim-only result, ~5–20 m error expected on real satellite tiles |
| 2 | SIFT 95% / ALIKED 18% fix rate | SIFT is scale-invariant; ALIKED/SuperPoint/DISK trained on MegaDepth/ScanNet (modest scale variation), never on 49 m nadir vs satellite tile (~10–50× scale gap) |
| 3 | ALIKED fused 76.2 m > Exp04 39.0 m | Bad fixes pass 150 m gate and pull trajectory further from GT: flow-odom at ~130 m from GT, ALIKED fix accepted within 150 m of drifted prior but 120–150 m from GT; blend=0.8 drags position away |
| 4 | 0 rejections | Gate measures distance from drifted prior, not from GT — no quality signal against plausible-but-wrong fixes |

Anomalies 1 and 2 are observations, not bugs.  Anomalies 3 and 4 share one root
cause: the 150 m gate is too loose and measures from the wrong reference.

---

## Hypothesis

Reducing `--reject` from 150 m to 30–50 m will block the bad ALIKED fixes that
move position further from GT, because a bad fix that is 120 m from GT but within
150 m of a drifted prior is typically ≥ 50 m from that same prior when flow-odom
RMSE is 80 m.  Adding `--min_inliers 30` (vs current 15) further filters
low-confidence fixes that tend to land further from the true position.

---

## Experiment 1 — Gate sweep (reject parameter)

Run SIFT pipeline (Exp05 winner) with tighter gates.  SIFT is used because it has
95% fix rate, so gate changes produce clear statistical signal.

```bash
for r in 30 40 50 75 100 150; do
    conda run -n drone python pipeline.py \
        --dir _in/isaac-sim-20260625 \
        --blend 0.8 --reject $r
done
```

**Metrics to record per run:**

| Metric | Current (reject=150) |
|---|---|
| Fused RMSE | ~15 m |
| Final error | ~1 m |
| DSMAC fixes accepted / attempted | — |
| % fixes rejected | — |

Track whether RMSE and final error improve or degrade as reject tightens.  A gate
too tight will start rejecting valid fixes and let flow-odom drift.

---

## Experiment 2 — Inlier threshold sweep

Add `--min_inliers` flag to `pipeline.py` (currently hardcoded to 15 in
`_dsmac_fix`).  Sweep with ALIKED (worst case from Exp05) to test whether a higher
inlier threshold filters the harmful fixes:

```bash
for inl in 15 20 30 40 50; do
    conda run -n drone python pipeline.py \
        --dir _in/isaac-sim-20260625 \
        --blend 0.8 --reject 150 --min_inliers $inl
done
```

Also run the best (reject, min_inliers) pair from Exp1+2 combined for SIFT.

---

## Experiment 3 — Combined fix for ALIKED

Apply the best gate + inlier threshold found above to ALIKED (the Exp05 loser) to
confirm fixes bring ALIKED fused RMSE below flow-odom-only (77.2 m) and below
Exp04's 39.0 m:

```bash
conda run -n drone python pipeline.py \
    --dir _in/isaac-sim-20260625 \
    --blend 0.8 --reject <best_r> --min_inliers <best_inl>
    # with ALIKED extractor — requires --extractor flag from Exp05 scripts
    # or swap make_sift_lg() to make_aliked_lg() manually
```

---

## Script changes required

**`pipeline.py`:**

1. Expose `--min_inliers` as a CLI argument (currently hardcoded in `_dsmac_fix`):
   ```python
   ap.add_argument("--min_inliers", type=int, default=15,
                   help="min RANSAC inliers to accept a DSMAC fix")
   ```
   Pass `args.min_inliers` into `_dsmac_fix`.  This is a one-line change; the
   argument exists in the parser already (line 473) but `_dsmac_fix` uses the
   local variable `args.min_inliers` via closure — verify the closure actually
   captures `args` or pass it explicitly.

2. Optionally expose `--extractor {sift,aliked}` to avoid code edits for Exp3.

---

## Success criteria

| Test | Pass condition |
|---|---|
| Gate sweep | RMSE does not increase vs reject=150 at reject=40–50 m |
| Inlier sweep | ALIKED RMSE drops below flow-odom-only (77.2 m) with inl≥30 |
| Combined (ALIKED) | Fused RMSE ≤ 39.0 m (Exp04 baseline) |
| SIFT (gate tight) | RMSE stays ≤ 20 m; final error stays ≤ 5 m |

---

## Open items carried from Exp05

- Anomaly 1 (1.0 m final in sim) — not a bug, expected to be 5–20 m on real data;
  verify on a real-world dataset when available.
- Anomaly 2 (scale gap) — ALIKED/DISK/SuperPoint architecturally cannot close this
  gap without fine-tuning on satellite-scale pairs; document as known limitation.
- Blend autotune (see root `plan.md`) — implement after gate/threshold are fixed so
  autotune warm-up runs on good fixes only.
