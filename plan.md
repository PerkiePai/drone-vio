# Implementation Plan — Autotune reject + skip_below

Blend autotune is already implemented (`--autotune`, lines 334–346 of `pipeline.py`).
The warmup infrastructure (`warmup_jumps`, `dsmac_std`) is already in place.
This plan extends autotune to cover `--reject` and `--skip_below` using the same
warmup data — no GT required.

---

## reject

**Current:** fixed gate `d <= args.reject` (150 m), measured from the drifted prior.
**Problem:** too loose when drift is low (lets bad fixes through — Exp05 anomaly 3);
too tight when drift is high (rejects valid fixes).

**Fix: per-fix dynamic gate**

```python
reject = drift_since + 3 * dsmac_std
```

- `drift_since` — already tracked; accounts for how wrong the prior could be
- `3 * dsmac_std` — 3-sigma DSMAC noise, estimated from warmup residuals

Right after a correction (`drift_since ≈ 0`) the gate is ~3 × dsmac_std (~40 m for
SIFT).  After 500 m of dead-reckoning it widens to ~500 + 40 = 540 m.  A bad fix
that is 120 m from GT but only 80 m from a drifted prior gets rejected when the gate
is 80 + 40 = 120 m — it lands exactly on the boundary, whereas the current fixed
150 m accepts it unconditionally.

**During warmup** (`dsmac_std is None`): fall back to `args.reject` so the warmup
fixes themselves are not rejected by an uninitialized gate.

---

## skip_below

**Current:** fixed threshold `drift_since >= args.skip_below` (13 m).
**Problem:** hardcoded to 13 m, which happens to match SIFT on Isaac Sim but is
too low for noisy extractors (ALIKED `dsmac_std` much higher — attempting fixes
too early injects noise before flow-odom has drifted enough to make the fix
worthwhile).

**Fix: set threshold to dsmac_std**

```python
skip_below = dsmac_std   # attempt only when expected flow error ≥ DSMAC noise
```

Only fire DSMAC once flow-odom has drifted enough that the fix can actually improve
position.  Below this threshold, DSMAC noise exceeds the benefit.

**During warmup** (`dsmac_std is None`): use `args.skip_below` (default 13 m) so
the warmup fixes fire normally.

---

## Changes to `pipeline.py`

All changes are inside the DSMAC block (~line 326).  The warmup state variables
(`warmup_jumps`, `dsmac_std`) already exist.

### 1. Dynamic reject gate (replace fixed `acc` check)

```python
# line 332 — replace:
acc = d <= args.reject

# with:
if args.autotune and dsmac_std is not None:
    reject = drift_since + 3 * dsmac_std
else:
    reject = args.reject
acc = d <= reject
```

### 2. Dynamic skip_below (replace fixed skip condition)

```python
# line 327 — replace:
if step % args.fix_every == 0 and drift_since >= args.skip_below:

# with:
skip = dsmac_std if (args.autotune and dsmac_std is not None) else args.skip_below
if step % args.fix_every == 0 and drift_since >= skip:
```

### 3. Stats panel (`report_and_plot`)

Update the `reject` and `skip_below` rows to show `autotune` when active:

```python
f"{'reject':<22} {'autotune (drift+3σ)' if args.autotune else str(args.reject) + ' m'}\n"
f"{'skip_below':<22} {'autotune (=dsmac_std)' if args.autotune else str(args.skip_below) + ' m'}\n"
```

---

## Ordering dependency

`dsmac_std` is frozen after the first `warmup_fixes` accepted fixes.  Until then,
all three autotuned values (`blend`, `reject`, `skip_below`) fall back to their
fixed CLI defaults.  The warmup phase must complete before autotune takes effect —
this happens automatically once `len(warmup_jumps) == args.warmup_fixes`.

---

## Validation run

```bash
# baseline
conda run -n drone python pipeline.py --dir _in/isaac-sim-20260625

# full autotune (blend + reject + skip_below)
conda run -n drone python pipeline.py --dir _in/isaac-sim-20260625 --autotune
```

Expected: fewer bad fixes accepted (dynamic reject tighter early in each leg),
ALIKED fused RMSE drops below flow-odom-only (77.2 m), SIFT RMSE stays ≤ 20 m.

---

## Files changed

| File | Change |
|---|---|
| `pipeline.py` | dynamic `reject` and `skip_below` inside DSMAC block; stats panel labels |

No new flags needed — both extend the existing `--autotune` switch.
