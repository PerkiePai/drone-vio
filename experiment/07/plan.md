# Experiment 07 — Grilling Audit: GT Dependency, Autotune Numerics, Engineering Fixes

**Date:** 2026-06-30
**Source:** `/grill-with-docs` audit of `pipeline.py`
**Conda env:** `drone`
**Script:** `pipeline.py` (modified in-place, as per Exp05/autotune convention)
**Datasets:**
- Short: `_in/isaac-sim-20260624_2337` (1 773 m, 7.5 min, 14 070 frames)
- Long: `_in/isaac-sim-20260625` (22 407 m, 76.9 min, 67 861 frames)

---

## Issues addressed

| Issue | Severity | Action |
|-------|----------|--------|
| Q1: GT init + GT-derived compass leak into metrics | High | Exp 1 |
| Q2: `dsmac_std` inflated by warmup drift, not DSMAC noise | High | Exp 2 |
| Q8: Ortho RAM unquantified at zoom 19 for long flight | High | Engineering fix |
| Q3: `flow_std = drift × 0.05` coefficient is uncalibrated | Medium | Exp 2 |
| Q4: Blend floor 0.3 applies 30% of any accepted fix unconditionally | Medium | Exp 2 |
| Q5: Baro AGL fallback makes DSMAC warp wrong (predicted: < 5% fix rate) | Medium | Exp 3 |
| Q9: File handles not closed in `build_ortho` and `run_pipeline` | Medium | Engineering fix |
| Q12: Right/bottom ortho boundary not clamped in `_dsmac_fix` | Medium | Engineering fix |

**Deferred:** Q6 (homography chain — correct but fragile; add a comment), Q11 (adaptive RANSAC threshold — carry to Exp07)

---

## Engineering Fixes

Apply these directly to `pipeline.py` before running the experiments. They have no effect on
numeric results (Q8 is diagnostic-only), so no baseline re-run is needed.

### Fix 1 — File handle leaks (Q9)

`build_ortho` line ~108 and `run_pipeline` lines ~219, 223 open files without `with` blocks.
On a run fetching hundreds of zoom-19 tiles, GC may not keep up.

```python
# pipeline.py, build_ortho, tile download (~line 108)
# Before:
open(fp, "wb").write(urllib.request.urlopen(req, timeout=20).read())
# After:
with open(fp, "wb") as fh:
    fh.write(urllib.request.urlopen(req, timeout=20).read())

# pipeline.py, run_pipeline, geo/georef reads (~lines 219, 223)
# Before:
geo = list(csv.DictReader(open(os.path.join(D, "geo.csv"))))
g   = json.load(open(os.path.join(D, "georef.json")))
# After:
with open(os.path.join(D, "geo.csv")) as fh:
    geo = list(csv.DictReader(fh))
with open(os.path.join(D, "georef.json")) as fh:
    g = json.load(fh)
```

### Fix 2 — Ortho boundary clamp (Q12)

`_dsmac_fix` clamps `x0, y0` to `>= 0` but not to `<= meta["W/H"] - 2*win`, so a drone
near the right or bottom edge gets a smaller-than-expected search window without warning.

```python
# pipeline.py, _dsmac_fix (~lines 263–264)
# Before:
x0 = max(0, int(cx - args.win))
y0 = max(0, int(cy - args.win))
# After:
x0 = int(np.clip(cx - args.win, 0, meta["W"] - 2 * args.win))
y0 = int(np.clip(cy - args.win, 0, meta["H"] - 2 * args.win))
```

This guarantees the window is exactly `2*args.win × 2*args.win` everywhere. The trade-off
is that near the boundary the prior is no longer window-centered; but `x0 + p[0]` already
remaps correctly, so the fix position is unaffected.

### Fix 3 — Ortho memory warning (Q8)

Insert after `W, H = ...` in `build_ortho`:

```python
# pipeline.py, build_ortho (~line 96)
W, H = (xb - xa + 1) * 256, (yb - ya + 1) * 256
mem_mb = W * H * 3 / 1e6
print(f"  estimated ortho RAM: {mem_mb:.0f} MB  ({W}×{H} px)")
if mem_mb > 500:
    print(f"  WARNING: ortho > 500 MB — consider reducing zoom (current z={z})")
```

Record the actual size printed for both datasets in result.md.

### Deferred comment (Q6)

In `_dsmac_fix`, add a comment before the `warp_north_up` + homography block explaining
that `cpt` is the image-centre in the warped-`q` pixel frame, and that `Hm` maps `q`
keypoints → `win` keypoints, so `Hm @ cpt` gives the fix in `win`-space:

```python
# pipeline.py, _dsmac_fix (~line 267)
# Hm maps q-keypoints → win-keypoints.  cpt is the drone image centre in q-space
# (output of warp_north_up).  Hm @ cpt → fix position in win-space → add (x0,y0)
# for ortho-pixel coords → px_to_enu.
q, cpt = warp_north_up(im, yaw, f)
```

---

## Experiment 1 — GT Dependency Audit (Q1)

### Motivation

The pipeline docstring says "GT is used ONLY for final scoring." In practice:
- `pos = recs[0]["gt"][:2].copy()` — initial position is exact GT.
- `compute_ahrs_attitude(mag_gain=1.0)` — yaw is corrected from GT heading at every frame.

We want to know: how much does the "RMSE ~15 m / final ~1 m" headline depend on these?
Can DSMAC recover from a bad init? Is the compass load-bearing?

### CLI additions to pipeline.py

```python
ap.add_argument("--init_offset_m", type=float, default=0.0,
                help="σ of Gaussian init position noise in metres (seed 42; 0=GT init)")
ap.add_argument("--compass_gain",  type=float, default=1.0,
                help="AHRS compass correction strength (0=pure gyro/accel, 1=current default)")
```

In `run_pipeline`, after `pos = recs[0]["gt"][:2].copy()`:
```python
if args.init_offset_m > 0:
    rng = np.random.default_rng(42)
    pos = pos + rng.normal(0, args.init_offset_m, size=2)
    print(f"  init offset applied: {np.linalg.norm(pos - recs[0]['gt'][:2]):.1f} m")
```

Replace the `compute_ahrs_attitude` call:
```python
att_R = fo.compute_ahrs_attitude(D, recs, Kp=1.0, mag_gain=args.compass_gain)
```

### Runs

**Sweep A — init error (long dataset, compass_gain=1.0):**
```bash
for offset in 0 25 50 100; do
    conda run -n drone python pipeline.py \
        --dir _in/isaac-sim-20260625 \
        --init_offset_m $offset --compass_gain 1.0 \
        --out _out/exp06/long_offset${offset}m.png
done
```

**Sweep B — compass gain (long dataset, init_offset_m=0):**
```bash
for gain in 0.0 0.5 1.0; do
    conda run -n drone python pipeline.py \
        --dir _in/isaac-sim-20260625 \
        --init_offset_m 0 --compass_gain $gain \
        --out _out/exp06/long_gain${gain}.png
done
```

**Sweep C — cross (short dataset; faster; shows DSMAC recovery):**
```bash
for offset in 0 50 100; do
    for gain in 0.0 1.0; do
        conda run -n drone python pipeline.py \
            --dir _in/isaac-sim-20260624_2337 \
            --init_offset_m $offset --compass_gain $gain \
            --out "_out/exp06/short_off${offset}_gain${gain}.png"
    done
done
```

### Metrics table

| Dataset | Init offset (m) | Compass gain | RMSE | Final error | Fix rate | Notes |
|---------|----------------|-------------|------|-------------|---------|-------|
| Long | 0 | 1.0 | 15.1 m (expected) | 0.8 m | ~427/427 | baseline |
| Long | 25 | 1.0 | | | | |
| Long | 50 | 1.0 | | | | |
| Long | 100 | 1.0 | | | | |
| Long | 0 | 0.0 | | | | |
| Long | 0 | 0.5 | | | | |
| Short | 0 | 1.0 | 11.6 m | 5.0 m | | baseline |
| Short | 50 | 1.0 | | | | |
| Short | 100 | 1.0 | | | | |
| Short | 0 | 0.0 | | | | |
| Short | 50 | 0.0 | | | | |
| Short | 100 | 0.0 | | | | |

### Pass conditions

| Question | Pass |
|----------|------|
| Can DSMAC recover from 50 m init error? | RMSE within 2× baseline by end of flight |
| Is the compass load-bearing? | compass_gain=0 RMSE within 2× baseline (< 30 m long, < 23 m short) |
| Are the headline metrics honest? | Document clearly which GT inputs they depend on |

---

## Experiment 2 — Autotune Numerics (Q2, Q3, Q4)

### Motivation

Three concerns about the autotune blend formula:
- **Q2:** `dsmac_std = std(warmup_jumps)` where each warmup jump `d = |fix − prior|`. If
  flow-odom has already drifted 80 m when the warmup fix fires, `d` is dominated by
  odometry error, not DSMAC noise. The result: `dsmac_std` is inflated, blend is
  underestimated, and DSMAC fixes are under-trusted throughout the flight.
- **Q3:** `flow_std = drift_since × 0.05`. The 5% coefficient was not derived from data.
- **Q4:** `blend = clip(formula, 0.3, 1.0)`. Floor 0.3 means even a suspicious fix
  moves the estimate by 30%, which may be harmful if the fix is bad.

### Warmup diagnostic print (Q2)

Add a temporary diagnostic during warmup. Remove it once Q2 is resolved.

```python
# pipeline.py, autotune warmup block (~line 340)
if len(warmup_jumps) < args.warmup_fixes:
    gt_err = float(np.linalg.norm(pos - recs[i]["gt"][:2]))  # GT used only for diag
    print(f"  warmup {len(warmup_jumps)+1}/{args.warmup_fixes}: "
          f"d(fix-prior)={d:.1f} m  d(flow-odom-GT)={gt_err:.1f} m  "
          f"ratio={d/max(gt_err, 1e-3):.2f}")
    warmup_jumps.append(d)
```

If `ratio ≈ 1.0`, then `d ≈ gt_err`: warmup residuals ARE flow-odom drift, not DSMAC
noise. Fix: cap each warmup entry at a plausible DSMAC noise level, e.g.:
```python
warmup_jumps.append(min(d, args.reject / 3))
```
If `ratio ≪ 1.0`, then the warmup correctly captures DSMAC noise and Q2 is not a problem.

### CLI additions to pipeline.py

```python
ap.add_argument("--flow_std_coeff", type=float, default=0.05,
                help="drift-to-uncertainty ratio for autotune blend (default 0.05)")
ap.add_argument("--blend_floor",    type=float, default=0.3,
                help="minimum blend weight in autotune mode (default 0.3; 0=no floor)")
```

In the autotune blend formula (~line 347–349):
```python
# Before:
flow_std = drift_since * 0.05
blend    = float(np.clip(blend * inlier_conf, 0.3, 1.0))
# After:
flow_std = drift_since * args.flow_std_coeff
blend    = float(np.clip(blend * inlier_conf, args.blend_floor, 1.0))
```

### Runs (short dataset, `--autotune` on)

**flow_std_coeff sweep (blend_floor=0.3 fixed):**
```bash
for c in 0.01 0.02 0.05 0.10 0.20; do
    conda run -n drone python pipeline.py \
        --dir _in/isaac-sim-20260624_2337 --autotune \
        --flow_std_coeff $c --blend_floor 0.3 \
        --out _out/exp06/short_coeff${c}.png
done
```

**blend_floor sweep (flow_std_coeff=0.05 fixed):**
```bash
for f in 0.0 0.1 0.2 0.3 0.5; do
    conda run -n drone python pipeline.py \
        --dir _in/isaac-sim-20260624_2337 --autotune \
        --flow_std_coeff 0.05 --blend_floor $f \
        --out _out/exp06/short_floor${f}.png
done
```

**Best-pair validation on long dataset:**
```bash
# Run the best (coeff, floor) found above on the long dataset
conda run -n drone python pipeline.py \
    --dir _in/isaac-sim-20260625 --autotune \
    --flow_std_coeff <best_c> --blend_floor <best_f> \
    --out _out/exp06/long_best_autotune.png
```

### Metrics table

| flow_std_coeff | blend_floor | Short RMSE | Short final | Notes |
|---------------|------------|------------|-------------|-------|
| 0.05 | 0.3 | 7.7 m (expected) | 1.2 m | current default |
| 0.01 | 0.3 | | | low flow uncertainty → high blend |
| 0.02 | 0.3 | | | |
| 0.10 | 0.3 | | | |
| 0.20 | 0.3 | | | high → under-trust fixes |
| 0.05 | 0.0 | | | no floor |
| 0.05 | 0.1 | | | |
| 0.05 | 0.2 | | | |
| 0.05 | 0.5 | | | floor > coeff effect |

### Pass conditions

| Question | Pass |
|----------|------|
| Q2: Is warmup biased? | `ratio = d / gt_err` printed for each warmup fix; if > 2× → apply warmup cap |
| Q3: Best coeff | If any coeff beats 0.05 by ≥ 10% RMSE, update `--flow_std_coeff` default |
| Q4: Best floor | If floor=0.0 or 0.1 beats 0.3 by ≥ 10% RMSE, update `--blend_floor` default |

---

## Experiment 3 — Baro Fallback (Q5)

### Motivation

When `agl_cache.npz` is absent, `agl` is set from baro (`r["h"]` = pressure altitude above
takeoff). At cruise altitude, baro gives ~5–10× too low a value vs true AGL over terrain.
The DSMAC warp factor `f = agl / (fx × GSD)` shrinks the drone image to a fraction of its
correct size, creating a scale mismatch that should cause RANSAC homography to fail.

### Setup

```bash
# Backup AGL cache for short dataset
mv _in/isaac-sim-20260624_2337/agl_cache.npz \
   _in/isaac-sim-20260624_2337/agl_cache.npz.bak
```

```bash
# Run pipeline on short dataset without AGL cache → hits baro fallback
conda run -n drone python pipeline.py \
    --dir _in/isaac-sim-20260624_2337 \
    --out _out/exp06/short_baro_fallback.png \
    2>&1 | tee _out/exp06/baro_fallback_log.txt
```

```bash
# Restore
mv _in/isaac-sim-20260624_2337/agl_cache.npz.bak \
   _in/isaac-sim-20260624_2337/agl_cache.npz
```

### Metrics

| Metric | Expected | Actual |
|--------|----------|--------|
| Warning printed | "WARNING: agl_cache.npz not found" | |
| Fix attempts | same as AGL baseline | |
| Matched fixes (RANSAC passed) | < 5% | |
| Fused RMSE | ≈ flow-odom-only (~11 m on short) | |
| Any valid fix position error | — | record if any slip through |

### Pass conditions

Confirm the baro path fails as predicted. If any fixes slip through with correct position
(< 30 m error), document the threshold at which RANSAC tolerates scale error — this informs
how tolerant the system is and whether Q11 (adaptive RANSAC threshold) is worth pursuing.

---

## Output directory

```
_out/exp06/
  long_offset0m.png        long_offset25m.png
  long_offset50m.png       long_offset100m.png
  long_gain0.0.png         long_gain0.5.png      long_gain1.0.png
  short_off0_gain0.0.png   short_off0_gain1.0.png
  short_off50_gain0.0.png  short_off50_gain1.0.png
  short_off100_gain0.0.png short_off100_gain1.0.png
  short_coeff0.01.png  short_coeff0.02.png  short_coeff0.05.png
  short_coeff0.10.png  short_coeff0.20.png
  short_floor0.0.png  short_floor0.1.png  short_floor0.2.png
  short_floor0.3.png  short_floor0.5.png
  long_best_autotune.png
  short_baro_fallback.png
  baro_fallback_log.txt
```

---

## Execution order

1. Apply engineering fixes (Q9, Q12, Q8, Q6 comment) to `pipeline.py` and commit.
2. Run Exp 3 (baro fallback) — fastest, confirm the failure mode before other runs.
3. Run Exp 1 Sweep A (init error, long dataset).
4. Run Exp 1 Sweep B (compass gain, long dataset).
5. Run Exp 1 Sweep C (cross, short dataset).
6. Run Exp 2 warmup diagnostic first (read the prints) → decide if warmup cap is needed.
7. If warmup cap needed, add it and re-run baseline before sweeps.
8. Run Exp 2 flow_std_coeff sweep (short dataset).
9. Run Exp 2 blend_floor sweep (short dataset).
10. Run Exp 2 best-pair validation (long dataset).
11. Write result.md.

---

## Overall success criteria

| Issue | Pass condition |
|-------|---------------|
| Q1 init | DSMAC recovers from 50 m offset within 5 km; if not, document the recovery limit |
| Q1 compass | compass_gain=0 RMSE < 2× baseline; if not, report the GT-compass dependency honestly |
| Q2 warmup bias | Bias confirmed or refuted with data; if confirmed, cap is applied and re-validated |
| Q3 flow_std_coeff | Default updated if a better value found (≥ 10% RMSE improvement) |
| Q4 blend_floor | Default updated if a better value found (≥ 10% RMSE improvement) |
| Q5 baro fallback | < 5% fix rate confirmed; RANSAC tolerance to scale error characterized |
| Q8 memory | Actual ortho size printed and recorded; no OOM for either dataset |
| Q9 file handles | No bare `open()` in pipeline.py |
| Q12 ortho boundary | Window always exactly 2×win × 2×win |

---

## Open items carried forward

- Q6: Comment added to `_dsmac_fix` (this plan).
- Q11: Adaptive RANSAC threshold — defer to Exp07 if Exp 3 shows fixes slipping through
  despite wrong scale (would suggest the threshold is too loose rather than too tight).
- Exp05 open items: compass noise sensitivity, short-flight tilt gap (structural).
