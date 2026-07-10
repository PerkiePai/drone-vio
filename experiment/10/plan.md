# Experiment 10 — Terrain-Relief/DEM Correlation (Canopy Global-Fix Add-On)

**Date:** 2026-07-10
**Source:** Open item carried from `experiment/08/result.md` and restated in
`experiment/09/result.md`'s conclusion: DSMAC (image correlation against a satellite ortho)
scores **zero fixes** on canopy/forest terrain (ds2, ds4) and stays low on repetitive
farmland (ds3, ds6) — not a weak-matcher problem, a genuine absence of correspondable visual
texture. "A real correction source for canopy-covered legs (e.g. terrain-relief/DEM
correlation instead of optical matching) is still needed if canopy fix rate is ever to move
off zero." This experiment builds that source.
**Conda env:** `drone` (project standing rule)
**Scripts:** `pipeline.py` (repo root, modified in place per Exp05–09 convention, then
snapshotted), `frontend/flow-odom/flow_odometry.py` (canonical copy, modified in place, then
snapshotted) — both imported by `pipeline.py` via `sys.path.insert(... "frontend/flow-odom")`.

---

## Goal

DSMAC bypasses the monocular-scale problem by matching *appearance* against a pre-loaded
satellite image. Where appearance fails (canopy has no correspondable texture between
sim-rendered canopy and Esri imagery), the terrain still has **3D relief** — canopy-top
height variation, hills — that can be sensed and correlated against a pre-loaded **DEM**
instead of an RGB ortho. This is the TERCOM half of the classic TERCOM+DSMAC odometry/
map-matching pattern (this project already has the DSMAC half).

**Deliverable:** a `--relief_gate {off,on}` flag on `pipeline.py` that, when DSMAC's existing
canopy gate (`is_canopy_nonviable`, Exp09) flags a fix attempt as hopeless for image
matching, tries a terrain-relief correlation fix instead — additive, not a replacement:
ds1's already-working DSMAC-only behavior is untouched (`--relief_gate` defaults `off`).

**Explicit non-goals / expected honest nulls:** ds3/ds6 are repetitive farmland, which tends
to be geometrically flat — relief correlation may find no discriminative signal there even
though the mechanism works. That is a legitimate result of this experiment, not a bug to
chase away.

---

## Design recap (user-approved)

**Why two independent passes, not one.** Building the reference DEM from the *same live
flight's own* triangulation would make correlation a tautology (matching a signal against a
copy of itself) — the same trap Exp06's ablations already hit with self-triangulated AGL
("Case B... cannot recover metric scale... same unobservability that sinks OpenVINS").
So the design is split:

1. **Offline reference DEM raster** — a one-time, whole-flight pass using **GT poses**
   (the same idealized-truth role `compute_true_agl` already plays for AGL) that triangulates
   tracked features across the *entire* flight and keeps each point's full `(X, Y, Z)`
   instead of collapsing to a median `Z`, gridded into a genuine 2D elevation raster. This is
   the "pre-loaded DEM you'd have downloaded before the mission" stand-in — same idealized
   framing as the existing AGL work, just retaining horizontal extent.

2. **Online sensed-relief signal** — reuses whatever AGL source `pipeline.py` already picked
   per-dataset (`lidar.csv` → triangulated cache → baro fallback, Exp09's existing loader):
   `terrain_elev(i) = baro_altitude(i) − agl(i)`. For ds2/4/5/6 this is **real lidar**, zero
   triangulation, the textbook TERCOM measurement. For ds3 (no `lidar.csv`), `agl` falls back
   to the triangulated cache — which, like `compute_true_agl` itself, is *also*
   GT-pose-based. This is a known, already-accepted idealized-proxy convention in this
   codebase (see `frontend/CLAUDE.md`: "deploy by replacing `compute_true_agl` with a
   rangefinder/DEM lookup"), not a new circularity this experiment introduces — flagged here
   so it isn't mistaken for one, and carried as an open item for real deployment (ds3's result
   should be read as another idealized upper-bound PoC data point, like the rest of the AGL
   story, not as a fully realistic no-hardware result).

**Search + correlation (TERCOM-style, chosen over a 2D DSMAC-mirrored patch search — simpler,
degrades more gracefully with sparse/noisy depth, which matters most exactly where canopy
already degrades feature quality).** At each fix attempt: take the flow-odom dead-reckoned
position trace for the last `--relief_window` steps (already accumulated every step in
`run_pipeline`'s loop) plus the parallel `terrain_elev` values: slide a 2D grid of candidate
`(dE, dN)` offsets over the DEM raster, bilinear-sample the DEM along the offset path,
affine-fit sensed vs. DEM values (unknown scale/bias — the sensed signal's absolute meaning
is not trusted, only its shape), score by Pearson correlation. Best-scoring candidate above
threshold, with both signals showing enough spread to be meaningful (flat-terrain guard) →
fix, fed through the *same* distance-from-prior reject gate and blend logic `_dsmac_fix`
already uses.

**Fires only when DSMAC's canopy gate says DSMAC is hopeless** (per the approved scope) — so
it never touches ds1's already-working path, and on ds2–ds6 it only activates in the windows
`is_canopy_nonviable` already flags.

---

## Task 1 — `build_dem_raster()` in `frontend/flow-odom/flow_odometry.py`

Insert after `compute_true_agl` (currently ends at line 253, before `def run(`):

```python
def build_dem_raster(recs, K, R_CtoI, scale, cell_m=5.0, W=10, step=4, min_pts=200):
    """Offline, whole-flight DEM stand-in for terrain-relief correlation (Exp10).
    Same triangulation primitive as compute_true_agl (GT-pose triangulation — the
    idealized-truth role that function already plays for AGL) but keeps each
    point's full (X, Y, Z) instead of collapsing to a median Z per window, then
    grids the accumulated point cloud into a 2D elevation raster. This DEM is
    built once, offline, from the whole flight's GT poses -- it plays the role
    of a pre-loaded reference map (like DSMAC's Esri ortho tiles), NOT a live
    per-step estimate, so correlating a live/online relief signal against it is
    not the self-triangulation circularity Exp06 already ruled out.

    Returns a dict: raster (ny,nx) float32 grid of elevation (NaN outside the
    triangulated points' convex hull -- deliberately NOT filled, so a
    correlation search step must fail closed rather than match against
    extrapolated terrain it never actually observed), e0/n0 (grid origin,
    metres), cell_m, nx, ny, n_points (diagnostic)."""
    Ks = K.copy(); Ks[:2, :] *= scale
    Kinv = np.linalg.inv(Ks)
    lk = dict(winSize=(21, 21), maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    feat = dict(maxCorners=400, qualityLevel=0.01, minDistance=10, blockSize=7)

    def load(p):
        im = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        return cv2.resize(im, None, fx=scale, fy=scale,
                           interpolation=cv2.INTER_AREA) if scale != 1 else im

    pts_e, pts_n, pts_z = [], [], []
    for i in range(0, len(recs) - W, step):
        r0, r1 = recs[i], recs[i + W]
        im0, im1 = load(r0["img"]), load(r1["img"])
        p0 = cv2.goodFeaturesToTrack(im0, mask=None, **feat)
        if p0 is None:
            continue
        p1, stt, _ = cv2.calcOpticalFlowPyrLK(im0, im1, p0, None, **lk)
        stt = stt.reshape(-1).astype(bool)
        p0g, p1g = p0.reshape(-1, 2)[stt], p1.reshape(-1, 2)[stt]
        C0, C1 = r0["gt"], r1["gt"]
        R0, R1 = r0["R_wb"] @ R_CtoI, r1["R_wb"] @ R_CtoI
        for (u0, v0), (u1, v1) in zip(p0g, p1g):
            d0 = R0 @ (Kinv @ np.array([u0, v0, 1.0])); d0 /= np.linalg.norm(d0)
            d1 = R1 @ (Kinv @ np.array([u1, v1, 1.0])); d1 /= np.linalg.norm(d1)
            X = _triangulate_midpoint(C0, d0, C1, d1)
            if X is not None and X[2] < C0[2] - 1.0:      # ground below camera
                pts_e.append(X[0]); pts_n.append(X[1]); pts_z.append(X[2])

    pts_e, pts_n, pts_z = np.array(pts_e), np.array(pts_n), np.array(pts_z)
    if len(pts_e) < min_pts:
        raise RuntimeError(
            f"build_dem_raster: only {len(pts_e)} triangulated ground points "
            f"(need >= {min_pts}) -- too sparse to build a usable DEM raster")

    e_min, e_max = pts_e.min(), pts_e.max()
    n_min, n_max = pts_n.min(), pts_n.max()
    nx = max(2, int((e_max - e_min) / cell_m) + 2)
    ny = max(2, int((n_max - n_min) / cell_m) + 2)
    grid_e = e_min + cell_m * np.arange(nx)
    grid_n = n_min + cell_m * np.arange(ny)
    GE, GN = np.meshgrid(grid_e, grid_n)

    from scipy.interpolate import griddata
    raster = griddata((pts_e, pts_n), pts_z, (GE, GN), method="linear")

    return dict(raster=raster.astype(np.float32), e0=float(e_min), n0=float(n_min),
                cell_m=float(cell_m), nx=nx, ny=ny, n_points=int(len(pts_e)))
```

**Verification:** `conda run -n drone python -c "import sys; sys.path.insert(0,'frontend/flow-odom'); import flow_odometry"` — imports cleanly (new function is only called explicitly, not at import time).

**Commit message:** `feat(flow_odometry): add build_dem_raster, whole-flight GT-pose DEM stand-in (Exp10, unwired)`

---

## Task 2 — DEM cache + `terrain_elev` in `pipeline.py`

Add near the top of `run_pipeline`, right after the existing AGL-loading block (after the
`if agl is None: ... WARNING` block, currently ending around line 352), a DEM-cache
loader and the online relief signal:

```python
    # DEM raster + relief signal (Exp10) -- only built if actually needed
    dem = None
    terrain_elev = None
    if args.relief_gate == "on":
        dem_path = os.path.join(D, "dem_cache.npz")
        if os.path.exists(dem_path):
            z = np.load(dem_path)
            dem = dict(raster=z["raster"], e0=float(z["e0"]), n0=float(z["n0"]),
                       cell_m=float(z["cell_m"]), nx=int(z["nx"]), ny=int(z["ny"]))
            print(f"  DEM raster: {dem['nx']}x{dem['ny']} cells @ {dem['cell_m']} m "
                  f"(cached)")
        else:
            print("  building offline DEM raster (whole-flight GT-pose triangulation) ...")
            dem = fo.build_dem_raster(recs, K, R_CtoI, args.scale)
            np.savez(dem_path, raster=dem["raster"], e0=dem["e0"], n0=dem["n0"],
                     cell_m=dem["cell_m"], nx=dem["nx"], ny=dem["ny"])
            print(f"  DEM raster: {dem['nx']}x{dem['ny']} cells @ {dem['cell_m']} m, "
                  f"{dem['n_points']} triangulated points")
        baro_h = np.array([r["h"] for r in recs])
        terrain_elev = baro_h - agl
```

**Verification:** `conda run -n drone python -c "import pipeline"` — imports cleanly (block
is inside `run_pipeline`, not executed at import time).

**Commit message:** `feat(pipeline): load/build DEM raster cache + terrain_elev signal (Exp10, unwired)`

---

## Task 3 — `_relief_fix()` + CLI flags in `pipeline.py`

Add alongside `_dsmac_fix` (after its closing `except Exception: return None`, still inside
`run_pipeline` so it closes over `dem`, `terrain_elev`, `args`):

```python
    def _relief_fix(fused_list, frame_idx_list):
        """TERCOM-style relief-correlation fix: slide the last `--relief_window`
        steps' dead-reckoned position trace over a 2D grid of candidate offsets
        in the DEM raster; affine-fit the sensed terrain_elev shape against the
        DEM values at each candidate (unknown scale/bias -- only shape is
        trusted); accept the best-scoring candidate if both signals show real
        spread and the correlation clears the threshold. Returns (eE, eN, corr)
        or None."""
        W = args.relief_window
        if len(fused_list) < W:
            return None
        path   = np.array(fused_list[-W:])                       # (W,2) ENU
        idxs   = frame_idx_list[-W:]
        sensed = np.array([terrain_elev[fi] for fi in idxs])
        if np.std(sensed) < args.relief_min_sensed_std:
            return None    # sensed profile itself is flat -- no shape to match

        raster = dem["raster"]; e0, n0, cell_m = dem["e0"], dem["n0"], dem["cell_m"]
        ny, nx = raster.shape

        def sample(cand_path):
            gx = (cand_path[:, 0] - e0) / cell_m
            gy = (cand_path[:, 1] - n0) / cell_m
            if gx.min() < 0 or gy.min() < 0 or gx.max() >= nx - 1 or gy.max() >= ny - 1:
                return None
            x0 = np.floor(gx).astype(int); y0 = np.floor(gy).astype(int)
            fx = gx - x0; fy = gy - y0
            v00 = raster[y0,     x0    ]; v10 = raster[y0,     x0 + 1]
            v01 = raster[y0 + 1, x0    ]; v11 = raster[y0 + 1, x0 + 1]
            vals = (v00 * (1 - fx) * (1 - fy) + v10 * fx * (1 - fy)
                    + v01 * (1 - fx) * fy + v11 * fx * fy)
            return None if np.isnan(vals).any() else vals

        best = None
        offsets = np.arange(-args.relief_win, args.relief_win + 1e-6, dem["cell_m"])
        for dE in offsets:
            for dN in offsets:
                dem_vals = sample(path + np.array([dE, dN]))
                if dem_vals is None or np.std(dem_vals) < args.relief_min_relief_std:
                    continue
                corr = np.corrcoef(sensed, dem_vals)[0, 1]
                if np.isfinite(corr) and (best is None or corr > best[2]):
                    best = (dE, dN, corr)

        if best is None or best[2] < args.relief_min_corr:
            return None
        dE, dN, corr = best
        eE, eN = path[-1, 0] + dE, path[-1, 1] + dN
        return eE, eN, float(corr)
```

Add these CLI flags in `main()`, next to `--canopy_gate`:

```python
    ap.add_argument("--relief_gate", choices=["off", "on"], default="off",
                    help="try a terrain-relief/DEM correlation fix (Exp10) when the "
                         "canopy gate flags DSMAC hopeless -- additive, DSMAC-only "
                         "behaviour is unchanged when this is off (default off)")
    ap.add_argument("--relief_window", type=int, default=30,
                    help="number of recent flow-odom steps used as the sensed "
                         "relief profile (default 30, matches --fix_every)")
    ap.add_argument("--relief_win", type=float, default=150.0,
                    help="relief-fix search half-window in metres (default 150, "
                         "matches the default --reject distance)")
    ap.add_argument("--relief_min_corr", type=float, default=0.6,
                    help="min Pearson correlation to accept a relief fix (default 0.6)")
    ap.add_argument("--relief_min_relief_std", type=float, default=3.0,
                    help="min DEM elevation std (m) within a candidate window to "
                         "bother scoring it -- skips flat terrain (default 3.0)")
    ap.add_argument("--relief_min_sensed_std", type=float, default=1.0,
                    help="min sensed terrain_elev std (m) to attempt a relief fix "
                         "at all -- skips a flat sensed profile (default 1.0)")
```

**Verification:** `conda run -n drone python -c "import pipeline"` still imports cleanly
(not wired into `run_pipeline`'s loop yet — that's Task 4).

**Commit message:** `feat(pipeline): add _relief_fix + CLI flags (Exp10, unwired)`

---

## Task 4 — Wire `_relief_fix` into the fix-attempt branch

Currently (`_dsmac_fix` internally checks `is_canopy_nonviable` and returns `None` before
matching if flagged). The outer loop's fix-attempt block is:

```python
        if step % args.fix_every == 0 and drift_since >= skip:
            fix = _dsmac_fix(i, pos)
            if fix is not None:
                eE, eN, inl = fix
                ...
```

Replace with (adds a lightweight outer canopy check purely to decide the branch — a few
lines of duplicated window math vs. `_dsmac_fix`'s internal check, kept separate rather than
changing `_dsmac_fix`'s existing return contract):

```python
        if step % args.fix_every == 0 and drift_since >= skip:
            fix, source = None, None
            nonviable = False
            if args.canopy_gate != "off":
                cx, cy = enu_to_px(pos[0], pos[1])
                x0 = int(np.clip(cx - args.win, 0, meta["W"] - 2 * args.win))
                y0 = int(np.clip(cy - args.win, 0, meta["H"] - 2 * args.win))
                probe = ortho[y0:y0 + 2 * args.win, x0:x0 + 2 * args.win]
                nonviable = probe.shape[0] >= 50 and probe.shape[1] >= 50 \
                    and is_canopy_nonviable(probe, args.canopy_gate)
            if not nonviable:
                fix = _dsmac_fix(i, pos)
                source = "dsmac"
            elif args.relief_gate == "on":
                fix = _relief_fix(fused, frame_idx_list)
                source = "relief"
            if fix is not None:
                eE, eN, score = fix
```

The rest of that block (unchanged in shape, but renaming `inl` → `score` throughout since
`_relief_fix` returns a 0–1 correlation, not an inlier count — `inlier_conf` must branch by
`source` rather than dividing a correlation by 50) becomes:

```python
            if fix is not None:
                eE, eN, score = fix
                d   = math.hypot(eE - pos[0], eN - pos[1])
                if args.autotune and dsmac_std is not None:
                    reject = drift_since + 3 * dsmac_std
                else:
                    reject = args.reject
                acc = d <= reject
                if acc:
                    if args.autotune:
                        if len(warmup_jumps) < args.warmup_fixes:
                            gt_err = float(np.linalg.norm(pos - recs[i]["gt"][:2]))
                            print(f"  warmup {len(warmup_jumps)+1}/{args.warmup_fixes}: "
                                  f"d(fix-prior)={d:.1f} m  d(flow-GT)={gt_err:.1f} m  "
                                  f"ratio={d/max(gt_err, 1e-3):.2f}")
                            warmup_jumps.append(d)
                            blend = 1.0
                        else:
                            if dsmac_std is None:
                                dsmac_std = np.std(warmup_jumps) if len(warmup_jumps) > 1 else d
                            inlier_conf = (min(1.0, score / 50) if source == "dsmac"
                                           else float(np.clip(score, 0.0, 1.0)))
                            flow_std    = drift_since * args.flow_std_coeff
                            blend       = (flow_std ** 2) / (flow_std ** 2 + dsmac_std ** 2)
                            blend       = float(np.clip(blend * inlier_conf, args.blend_floor, 1.0))
                    else:
                        blend = args.blend
                    pos = np.array([pos[0] + blend * (eE - pos[0]),
                                    pos[1] + blend * (eN - pos[1])])
                    drift_since = 0.0
                fixes.append((step, eE, eN, acc, d, score, source))
```

Also add `frame_idx_list = []` next to the other accumulator initializations (near
`fixes = []`), and `frame_idx_list.append(i)` next to the existing `fused.append(pos.copy())`
inside the loop.

Update `report_and_plot`'s fix-counting: the existing aggregate line reads
`f"  DSMAC fixes att/acc : {len(fixes)} / {nacc} ..."`, which becomes misleading once
`fixes` mixes DSMAC and relief entries — rename its label to `"Fix attempts att/acc"` and
add a per-source breakdown immediately below it:

```python
    for src in ("dsmac", "relief"):
        att = [f for f in fixes if f[6] == src]
        acc = [f for f in att if f[3]]
        print(f"    {src:<8}: {len(acc)}/{len(att)} accepted/attempted")
```

**Verification:** `conda run -n drone python pipeline.py --dir _in/isaac-sim-20260630_152940 --max_frames 3000 --canopy_gate off --relief_gate off` — must reproduce the existing ds1 behaviour byte-for-byte (this code path is unchanged when both gates are off).

**Commit message:** `feat(pipeline): wire relief-correlation fix into canopy-gated branch (Exp10)`

---

## Task 5 — Snapshot into `experiment/10/`

```bash
cp pipeline.py experiment/10/pipeline.py
cp frontend/flow-odom/flow_odometry.py experiment/10/flow_odometry.py
```

**Commit message:** `docs(experiment/10): snapshot pipeline.py + flow_odometry.py with relief-correlation`

---

## Task 6 — Build DEM caches + run the target datasets

**Datasets** (relief-correlation target set + ds1 as the regression check, same six from
Exp09):

| ID | Directory | Terrain |
|----|-----------|---------|
| ds1 | `isaac-sim-20260630_152940` | farmland, DSMAC works (regression check only) |
| ds2 | `isaac-sim-20260704_205334` | canopy (real `lidar.csv`) |
| ds3 | `isaac-sim-20260704_193743` | repetitive, no `lidar.csv` (triangulated-cache AGL) |
| ds4 | `isaac-sim-20260705_230815` | canopy (real `lidar.csv`) |
| ds5 | `isaac-sim-20260706_105804` | (real `lidar.csv`) |
| ds6 | `isaac-sim-20260705_220937` | repetitive (real `lidar.csv`) |

**Runs — ds2 through ds6, gate off vs. on** (ds1 is DSMAC-only, no relief gate needed):

```bash
for ds in isaac-sim-20260704_205334 isaac-sim-20260704_193743 isaac-sim-20260705_230815 \
          isaac-sim-20260706_105804 isaac-sim-20260705_220937; do
  conda run -n drone python pipeline.py --dir _in/$ds --canopy_gate color_texture \
      --relief_gate off --out _out/exp10_${ds}_relief_off.png \
      > /tmp/exp10_${ds}_off.log 2>&1
  conda run -n drone python pipeline.py --dir _in/$ds --canopy_gate color_texture \
      --relief_gate on  --out _out/exp10_${ds}_relief_on.png \
      > /tmp/exp10_${ds}_on.log 2>&1
done
```

**Regression check — ds1** (confirm relief-gate machinery does not touch the working path):

```bash
conda run -n drone python pipeline.py --dir _in/isaac-sim-20260630_152940 \
    --canopy_gate color_texture --relief_gate off \
    --out _out/exp10_ds1_regression.png > /tmp/exp10_ds1.log 2>&1
```

**Metrics to record per dataset:** relief-fix attempts vs. accepted (from the `report_and_plot`
per-source breakdown, Task 4), position error of accepted relief fixes vs. GT, fused RMSE /
final error with `--relief_gate off` vs. `on`, DEM raster build time + point count (diagnostic
for how sparse triangulation gets under canopy), wall-clock overhead of the added search.

**Verification:** all six runs exit 0; ds1's fused RMSE matches Exp09's ds1 baseline (370.3 m)
within normal run-to-run variance; each ds2–ds6 run's log shows a `relief:` breakdown line.

---

## Task 7 — `experiment/10/result.md`

Write up, following `experiment/01/result.md`/`experiment/02/result.md`'s structure:
- Dataset table (as above) + per-dataset DEM raster stats (cell count, triangulated point
  count — expect fewer points under canopy than farmland; report honestly if any dataset's
  point count is too sparse to grid meaningfully, per Task 1's `min_pts` guard).
- Per-dataset table: relief-fix attempts/accepted, position error of accepted fixes vs. GT,
  fused RMSE gate-off vs. gate-on, DSMAC fixes for comparison (from Exp09's numbers).
- ds1 regression check: pass/fail against Exp09's 370.3 m baseline.
- Explicit split of outcomes: canopy (ds2/ds4/ds5) vs. repetitive-farmland (ds3/ds6) —
  since flat terrain is expected to yield legitimate zero relief-fixes, this split is the
  main way to tell "mechanism doesn't help here because terrain is flat" apart from
  "mechanism is broken."
- ds3's idealized-AGL caveat (Design recap) restated in the conclusion, same as `agl_cache.npz`
  is already caveated elsewhere in this project.
- Summary table against the target bar (nonzero, bounded relief-fixes on canopy legs where
  DSMAC scored 0/0 in Exp09; no fused-RMSE regression on ds1).
- Open items: real (non-idealized) DEM source for deployment (real elevation product or a
  genuine Cesium-terrain-API query, superseding the GT-pose-triangulated stand-in used here —
  same "upper-bound PoC → deploy by replacing with a real lookup" pattern the AGL work already
  followed), and whether the 1D/along-track TERCOM formulation should be revisited in favour
  of the 2D patch alternative if canopy relief turns out to need finer spatial resolution than
  a single dead-reckoned track line provides.

**Commit message:** `docs(experiment/10): terrain-relief/DEM correlation results`

---

## Deliverables checklist

- [ ] `frontend/flow-odom/flow_odometry.py`: `build_dem_raster()`
- [ ] `pipeline.py` (root): DEM cache load/build, `terrain_elev`, `_relief_fix`, CLI flags,
      wiring, `fixes` source-tagging, `report_and_plot` per-source breakdown
- [ ] `experiment/10/pipeline.py`, `experiment/10/flow_odometry.py` (final snapshots)
- [ ] `dem_cache.npz` built for ds2–ds6 (left in each dataset's `_in/<name>/`, gitignored)
- [ ] `experiment/10/result.md`
- [ ] `_out/exp10_*_relief_{off,on}.png` (10 plots) + `_out/exp10_ds1_regression.png`,
      moved into `experiment/10/out/` per the project's plot-storage convention
