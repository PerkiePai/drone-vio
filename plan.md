# Implementation Plan — Blend Autotune

## Goal

Replace the fixed `--blend` parameter in `pipeline.py` with a self-calibrating blend
that estimates DSMAC noise from an initial warm-up phase and then adjusts blend
per-fix based on how much flow-odom has drifted since the last fix.  No GT required.

---

## Background

The current pipeline uses a single fixed `blend=0.8` for every DSMAC correction:

```python
pos = pos + blend * (fix - pos)
```

The optimal blend actually varies with how far flow-odom has drifted (more drift →
trust the fix more) and with DSMAC noise (noisier fix → trust it less).  The
Kalman-style formula is:

```
blend = drift_var / (drift_var + dsmac_var)
```

where:
- `drift_var` grows with distance walked since the last fix (rough: ~5% drift rate
  → `drift_std ≈ drift_since * 0.05`)
- `dsmac_var` is estimated from fix-to-prior residuals during warm-up, then frozen

An inlier-confidence scale (`min(1.0, inl/50)`) further down-weights low-inlier
fixes.  The blend is clipped to `[0.3, 1.0]` to prevent over-trust or near-zero
corrections.

---

## Changes to `pipeline.py`

### 1. New CLI flags

```
--autotune        enable blend autotune (default: off, keeps --blend behavior)
--warmup_fixes N  number of warmup fixes to collect dsmac_std (default: 6)
```

### 2. Warmup state variables (add after line 289 near `drift_since = 0.0`)

```python
warmup_jumps = []   # |fix - pos| residuals collected at blend=1.0
dsmac_std    = None # frozen after warmup; None means still warming up
```

### 3. DSMAC block replacement (inside the `if fix is not None:` block, ~line 330)

Replace:

```python
if acc:
    pos = np.array([pos[0] + args.blend * (eE - pos[0]),
                    pos[1] + args.blend * (eN - pos[1])])
    drift_since = 0.0
```

With:

```python
if acc:
    if args.autotune:
        if len(warmup_jumps) < args.warmup_fixes:
            # warm-up: snap fully to fix, record residual
            warmup_jumps.append(d)
            blend = 1.0
        else:
            if dsmac_std is None:
                dsmac_std = np.std(warmup_jumps) if len(warmup_jumps) > 1 else d
            inlier_conf = min(1.0, inl / 50)
            flow_std    = drift_since * 0.05       # rough ~5% LK drift rate
            blend       = (flow_std ** 2) / (flow_std ** 2 + dsmac_std ** 2)
            blend       = float(np.clip(blend * inlier_conf, 0.3, 1.0))
    else:
        blend = args.blend

    pos = np.array([pos[0] + blend * (eE - pos[0]),
                    pos[1] + blend * (eN - pos[1])])
    drift_since = 0.0
```

### 4. Stats panel update (in `report_and_plot`)

Add `blend` row: when autotune is on, display `autotune (warmup={N})` instead of
the fixed value.

---

## Practical tuning guidance (for reference)

| Situation | Suggested blend |
|---|---|
| Similar altitude, similar terrain | 0.8 (keep default) |
| Poor texture, DSMAC often wrong | 0.6–0.7 |
| High-quality ortho, low AGL noise | 0.9–1.0 |
| Very noisy flow-odom (high alt) | 0.9+ |

When autotune is not used, sweep blends with:

```bash
for b in 0.5 0.6 0.7 0.8 0.9 1.0; do
    conda run -n drone python pipeline.py --dir _in/your-dataset --blend $b
done
```

Pick the blend that minimises RMSE.  Since `--reject 150` already gates out gross
outliers, blend only matters for the fine-grained correction quality.

---

## Validation run

After implementation, compare on the long dataset:

```bash
# baseline (fixed blend)
conda run -n drone python pipeline.py --dir _in/isaac-sim-20260625 --blend 0.8

# autotune
conda run -n drone python pipeline.py --dir _in/isaac-sim-20260625 --autotune
```

Expected: autotune RMSE ≤ fixed-blend RMSE, with tighter variance across altitude
segments.  Warm-up phase (first 6 fixes) will show blend=1.0 then it adapts.

---

## Files changed

| File | Change |
|---|---|
| `pipeline.py` | `--autotune`, `--warmup_fixes` flags; warmup state; per-fix blend logic |

No new files.  Autotune is opt-in; `--blend` continues to work unchanged when
`--autotune` is not passed.
