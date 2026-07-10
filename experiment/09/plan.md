# Experiment 09 — Canopy/Repetitive-Terrain Gate (Latency, NOT an Accuracy Fix)

**Date:** 2026-07-10
**Source:** Follow-up to `experiment/08/result.md`'s conclusion
**Conda env:** `drone` (project standing rule)
**Scripts:** `pipeline.py` (modified in-place at repo root, per Exp05/06/07 convention, then
snapshotted into this folder), `flow_odometry.py` (unmodified snapshot, needed for import)

---

## Goal

Exp08 found that canopy/forest terrain (datasets 2, 4) gives DSMAC **zero** clearing fixes
regardless of AGL or zoom — SIFT already finds ~1,000+ keypoints on both sides, LightGlue's
match rate is genuinely near-zero (absence of correspondable structure, not a weak-matcher
problem). **A canopy detector cannot fix this.** What it *can* do is stop paying for a SIFT
extract + LightGlue match GPU call on every single DSMAC attempt over terrain already known
to be hopeless, and (via a second, more speculative signal) also skip attempts over locally
*repetitive* farmland — the other unresolved failure mode from Exp08 (datasets 3, 6 stayed
low even on non-forest terrain).

**Explicitly out of scope:** this experiment does not attempt to raise fix rate or accuracy
on datasets 2/4. Those numbers are expected to stay at 0/0 fixes with or without the gate —
the gate's payoff is measured in **skipped matcher calls / wall-clock time**, not RMSE.

**Deliverable:** a `--canopy_gate {off,color,color_texture}` flag on `pipeline.py`, validated
against Exp08's existing 164-sample ground truth (no new SIFT+LightGlue calls needed for
calibration), plus two live smoke tests, written up in `experiment/09/result.md`.

---

## Design recap (user-approved)

Two independent signals, computed on the **color** ortho search-window crop (`pipeline.py`
already builds `ortho` as color and only discards it by converting to `orthog` grayscale
right after `build_ortho` — no re-fetch needed, the color array just needs to stay in scope):

1. **`green_dominance(window_bgr)`** — mean(G−R) over the crop. Real Esri imagery over
   canopy renders strongly green-dominant; farmland/urban does not (confirmed cheaply on a
   handful of random tiles before this plan was written: canopy datasets 2/4 showed
   green-dominance ~23–27, farmland dataset 1 ~5–17 on a small sample — promising but not
   yet validated per-window against real match outcomes, which is what Task 4 does).
2. **`repetitiveness(window_gray)`** — height of the strongest *secondary* peak in the
   window's autocorrelation surface (a central sub-patch matched against the full window via
   `cv2.matchTemplate(..., TM_CCOEFF_NORMED)`, masking a patch-sized box around the trivial
   true peak). A periodic/repetitive field pattern matches itself again somewhere else in the
   window (high secondary peak); a locally unique patch does not. Targets the "locally
   repetitive farmland" gap Exp08 left unresolved — untested until Task 4.

**Two gate modes**, compared empirically rather than assumed: `color` (signal 1 alone) vs.
`color_texture` (signal 1 OR signal 2). `--canopy_gate off` is the default — nothing changes
unless the flag is passed.

**Threshold policy:** calibrated once against Exp08's 164 known-outcome samples (Task 4),
picking the highest-recall threshold subject to **precision ≥ 0.95** on "flag ⇒ actually
failed to clear 30 inliers." Precision is the binding constraint because a false positive
(flagging a window DSMAC would actually have matched) silently suppresses a real fix — much
costlier than a false negative, which only wastes one matcher call. Baked in as constants in
`pipeline.py` (same convention as `--min_inliers` defaulting to the Exp06-validated 30, not
an exposed per-flight knob).

---

## Task 1 — Add gate functions to `pipeline.py`

Insert after `warp_north_up` (currently ends at line 160), before the `# ─── LK optical-flow
odometry` section header:

```python
# ─── canopy / repetitive-terrain gate ────────────────────────────────────────
# Exp08 found forest canopy gives DSMAC ~0% clear rate regardless of AGL or zoom
# (absence of correspondable structure between sim-rendered canopy and real Esri
# imagery, not a weak-matcher problem -- SIFT already finds 1000+ keypoints on
# both sides). This gate does NOT fix that; it only skips the SIFT+LightGlue
# call on windows already known to be hopeless, saving GPU time. Thresholds
# calibrated in experiment/09/validate_gate.py against Exp08's 164 known-outcome
# samples -- see experiment/09/result.md for the calibration and its caveats.
GREEN_DOMINANCE_THRESH = None   # set by Task 5 after running validate_gate.py
REPETITIVENESS_THRESH  = None   # set by Task 5 after running validate_gate.py


def green_dominance(window_bgr):
    """Mean(G-R) over a color ortho crop. Canopy/forest renders strongly
    green-dominant in real Esri imagery vs. farmland/urban."""
    b, g, r = cv2.split(window_bgr.astype(np.float32))
    return float((g - r).mean())


def repetitiveness(window_gray, patch_frac=0.3, exclusion_mult=1.0):
    """Height of the strongest secondary autocorrelation peak: a central
    sub-patch matched against the full window via normalized cross-
    correlation, masking a patch-sized box around the trivial true peak
    (the patch always matches itself there with score ~1.0). High = the
    window contains another region nearly identical to its own centre
    (periodic/repetitive terrain); low = locally unique."""
    h, w = window_gray.shape
    ph, pw = int(h * patch_frac), int(w * patch_frac)
    cy0, cx0 = (h - ph) // 2, (w - pw) // 2
    patch = window_gray[cy0:cy0 + ph, cx0:cx0 + pw]
    corr = cv2.matchTemplate(window_gray, patch, cv2.TM_CCOEFF_NORMED)
    ey, ex = int(ph * exclusion_mult), int(pw * exclusion_mult)
    y0e, y1e = max(0, cy0 - ey), min(corr.shape[0], cy0 + ey)
    x0e, x1e = max(0, cx0 - ex), min(corr.shape[1], cx0 + ex)
    masked = corr.copy()
    masked[y0e:y1e, x0e:x1e] = -1.0
    return float(masked.max())


def is_canopy_nonviable(window_bgr, mode):
    """True if this DSMAC search window should be skipped before SIFT+LightGlue.
    mode: 'color' (green-dominance only) or 'color_texture' (also gated on
    repetitiveness). Thresholds calibrated in experiment/09/validate_gate.py."""
    if green_dominance(window_bgr) >= GREEN_DOMINANCE_THRESH:
        return True
    if mode == "color_texture":
        window_gray = cv2.cvtColor(window_bgr, cv2.COLOR_BGR2GRAY)
        if repetitiveness(window_gray) >= REPETITIVENESS_THRESH:
            return True
    return False
```

**Verification:** `conda run -n drone python -c "import pipeline"` — must import cleanly
(the `None` thresholds are fine at import time; they only need real values before
`--canopy_gate` is actually used, enforced in Task 2's wiring).

**Commit message:** `feat(pipeline): add canopy/repetitiveness gate functions (Exp09, unwired)`

---

## Task 2 — Wire the gate into `_dsmac_fix` + CLI flags

**Step 2a — add `skipped_canopy` counter and gate check inside `_dsmac_fix`.**

In `run_pipeline`, immediately before the `for i in range(args.stride, N, args.stride):` loop
(currently preceded by `prev = _load(recs[0]["img"])`), add:

```python
    skipped_canopy = [0]   # mutable counter, closed over by _dsmac_fix
```

Inside `_dsmac_fix`, right after the existing:
```python
        if win.shape[0] < 50 or win.shape[1] < 50:
            return None
```
add:
```python
        if args.canopy_gate != "off":
            win_bgr = ortho[y0:y0 + 2 * args.win, x0:x0 + 2 * args.win]
            if is_canopy_nonviable(win_bgr, args.canopy_gate):
                skipped_canopy[0] += 1
                return None
```
(`ortho`, the color array, is already in scope in `run_pipeline` — it's built by
`build_ortho` two lines before `orthog = cv2.cvtColor(...)` and never reassigned. `_dsmac_fix`
is a nested closure inside `run_pipeline`, so it can reference `ortho` the same way it already
references `orthog`, `meta`, `fx`, `GSD`, etc. No change needed to how `ortho` is built.)

**Step 2b — return the counter.** Change `run_pipeline`'s return statement from:
```python
    return (np.array(fused),
            np.array(gt_list),
            (np.array(ts_list) - ts_list[0]) / 1e9,
            fixes,
            n_used)
```
to:
```python
    return (np.array(fused),
            np.array(gt_list),
            (np.array(ts_list) - ts_list[0]) / 1e9,
            fixes,
            n_used,
            skipped_canopy[0])
```

**Step 2c — thread it through `report_and_plot`.** Change the signature:
```python
def report_and_plot(fused, GT, tvec, fixes, n_used, skipped_canopy, args):
```
and in the stats block (the `stats = (...)` f-string), add a line right after the
`DSMAC fixes` row:
```python
        f"{'DSMAC fixes':<22} {nacc}/{len(fixes)} accepted\n"
        f"{'Canopy-gate skips':<22} {skipped_canopy} ({args.canopy_gate})\n"
```
Also add a plain-text print alongside the existing `print(f"  DSMAC fixes att/acc : ...")`
block near the top of `report_and_plot`:
```python
    print(f"  Canopy-gate skips   : {skipped_canopy}  (mode={args.canopy_gate})")
```

**Step 2d — CLI flags.** In `main()`'s argparse block, add after `--min_inliers`:
```python
    ap.add_argument("--canopy_gate", choices=["off", "color", "color_texture"],
                    default="off",
                    help="skip SIFT+LightGlue on windows flagged non-viable before "
                         "matching (Exp09) -- latency optimization only, does NOT "
                         "improve fix rate/accuracy on genuinely hopeless terrain "
                         "(default off)")
    ap.add_argument("--max_frames", type=int, default=0,
                    help="truncate to the first N loaded frames (0=all); for fast "
                         "smoke tests, mirrors flow_odometry.py's existing flag")
```

**Step 2e — apply `--max_frames`.** In `run_pipeline`, right after:
```python
    K, R_CtoI, recs = fo.load_dataset(D)
    N = len(recs)
```
add:
```python
    if args.max_frames:
        recs = recs[:args.max_frames]
        N = len(recs)
```

**Step 2f — update the call site.** In `main()`, change:
```python
    fused, GT, tvec, fixes, n_used = run_pipeline(args)
    report_and_plot(fused, GT, tvec, fixes, n_used, args)
```
to:
```python
    fused, GT, tvec, fixes, n_used, skipped_canopy = run_pipeline(args)
    report_and_plot(fused, GT, tvec, fixes, n_used, skipped_canopy, args)
```

**Verification:** `conda run -n drone python pipeline.py --dir _in/isaac-sim-20260630_152940
--canopy_gate off --max_frames 500` must run to completion without error (thresholds are
still `None` at this point, but `is_canopy_nonviable` is never called when `canopy_gate ==
"off"`, so `None` thresholds don't matter yet).

**Commit message:** `feat(pipeline): wire canopy gate + --max_frames CLI flag (Exp09)`

---

## Task 3 — Snapshot into `experiment/09/`

```bash
cp pipeline.py experiment/09/pipeline.py
cp frontend/flow-odom/flow_odometry.py experiment/09/flow_odometry.py
```

**Verification:** `diff pipeline.py experiment/09/pipeline.py` → no output (identical).

**Commit message:** (fold into Task 5's commit once thresholds are filled in — an
intermediate snapshot with `None` thresholds isn't independently useful to commit.)

---

## Task 4 — `experiment/09/validate_gate.py`: calibrate against Exp08's ground truth

No new SIFT+LightGlue calls — reuses `experiment/08/sweep_combined.csv`'s already-known
`inliers`/`cleared_min_inliers` outcome for all 164 samples. Only needs each sample's GT
position (to relocate the same ortho window) and the color ortho array; **does not** need
per-frame yaw/AGL/attitude, since the window's pixel location and size depend only on the GT
easting/northing and the fixed `--win` half-width, not on heading or altitude.

```python
#!/usr/bin/env python3
"""Calibrate & validate the canopy/repetitiveness gate against experiment/08's
164 already-matched samples (experiment/08/sweep_combined.csv) -- no new
SIFT+LightGlue calls. For each row, rebuilds the exact color ortho search
window (same dataset/frame/GT-position recipe as sweep_matchability.py),
computes green_dominance + repetitiveness, and reports how well each signal
predicts the already-known cleared_min_inliers==False outcome.

Run (drone env):
  conda run -n drone python experiment/09/validate_gate.py
"""
import csv, json, math, os, sys

import cv2
import numpy as np

HERE  = os.path.dirname(os.path.abspath(__file__))
EXP08 = os.path.join(os.path.dirname(HERE), "08")
sys.path.insert(0, HERE)
sys.path.insert(0, EXP08)
import flow_odometry as fo
import pipeline as pl

IN_DIR    = os.path.join(os.path.dirname(os.path.dirname(HERE)), "_in")
SWEEP_CSV = os.path.join(EXP08, "sweep_combined.csv")
WIN       = 420


def build_color_ortho(D):
    pl._ensure_geo_georef(D)
    geo = list(csv.DictReader(open(os.path.join(D, "geo.csv"))))
    lat_c = (min(float(r["lat_deg"]) for r in geo) + max(float(r["lat_deg"]) for r in geo)) / 2
    lon_c = (min(float(r["lon_deg"]) for r in geo) + max(float(r["lon_deg"]) for r in geo)) / 2
    zoom = pl._probe_zoom(lat_c, lon_c, 19)
    ortho, meta = pl.build_ortho(geo, zoom, 0.0016, os.path.join(D, "ortho_tiles"))
    g = json.load(open(os.path.join(D, "georef.json")))
    lat0, lon0 = g["origin"]["latitude"], g["origin"]["longitude"]
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat0))
    nN = 2 ** meta["z"]

    def enu_to_px(E, Nn):
        lat = lat0 + Nn / mlat
        lon = lon0 + E / mlon
        gx = (lon + 180) / 360 * nN
        gy = (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * nN
        return (gx - meta["xa"]) * 256, (gy - meta["ya"]) * 256

    return ortho, meta, enu_to_px


def eval_thresh(signal, thresh, failed, cleared):
    flagged = signal >= thresh
    tp = int(np.sum(flagged & failed))
    fp = int(np.sum(flagged & cleared))
    fn = int(np.sum(~flagged & failed))
    tn = int(np.sum(~flagged & cleared))
    precision = tp / max(tp + fp, 1)
    recall = tp / max(tp + fn, 1)
    return dict(thresh=round(float(thresh), 2), tp=tp, fp=fp, fn=fn, tn=tn,
                precision=round(precision, 3), recall=round(recall, 3),
                flagged_frac=round(float(flagged.mean()), 3))


def main():
    rows = list(csv.DictReader(open(SWEEP_CSV)))
    ds_list = []
    for r in rows:
        if r["dataset"] not in ds_list:
            ds_list.append(r["dataset"])

    results = []
    for ds in ds_list:
        D = os.path.join(IN_DIR, ds)
        print(f"=== {ds} ===")
        _, _, recs = fo.load_dataset(D)
        gt_by_frame = {r["frame"]: r["gt"][:2] for r in recs}
        ortho, meta, enu_to_px = build_color_ortho(D)

        for r in [r for r in rows if r["dataset"] == ds]:
            frame = int(r["frame"])
            if frame not in gt_by_frame:
                continue
            gt_xy = gt_by_frame[frame]
            cx, cy = enu_to_px(gt_xy[0], gt_xy[1])
            x0 = int(np.clip(cx - WIN, 0, meta["W"] - 2 * WIN))
            y0 = int(np.clip(cy - WIN, 0, meta["H"] - 2 * WIN))
            window_bgr = ortho[y0:y0 + 2 * WIN, x0:x0 + 2 * WIN]
            if window_bgr.shape[0] < 50 or window_bgr.shape[1] < 50:
                continue
            gd = pl.green_dominance(window_bgr)
            window_gray = cv2.cvtColor(window_bgr, cv2.COLOR_BGR2GRAY)
            rep = pl.repetitiveness(window_gray)
            results.append(dict(
                dataset=ds, frame=frame, green_dominance=gd, repetitiveness=rep,
                cleared=(r["cleared_min_inliers"] == "True"), inliers=int(r["inliers"])))

    out_csv = os.path.join(HERE, "gate_signals.csv")
    with open(out_csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["dataset", "frame", "green_dominance",
                                            "repetitiveness", "cleared", "inliers"])
        w.writeheader()
        w.writerows(results)
    print(f"\nsaved {out_csv} ({len(results)} rows)")

    gd = np.array([r["green_dominance"] for r in results])
    rep = np.array([r["repetitiveness"] for r in results])
    cleared = np.array([r["cleared"] for r in results])
    failed = ~cleared
    print(f"\n{cleared.sum()} cleared / {failed.sum()} failed  (n={len(results)})")

    print("\n--- green_dominance threshold sweep ---")
    for t in np.percentile(gd, [50, 60, 70, 75, 80, 85, 90, 95, 99]):
        print(eval_thresh(gd, t, failed, cleared))

    print("\n--- repetitiveness threshold sweep ---")
    for t in np.percentile(rep, [50, 60, 70, 75, 80, 85, 90, 95, 99]):
        print(eval_thresh(rep, t, failed, cleared))

    print("\n--- per-dataset false-positive check (dataset 1 is the regression risk: "
          "it has real successes to lose) ---")
    for ds in ds_list:
        idxs = [i for i, r in enumerate(results) if r["dataset"] == ds]
        n_cleared_here = int(cleared[idxs].sum())
        print(f"  {ds:<28} n={len(idxs):3d}  cleared={n_cleared_here}")


if __name__ == "__main__":
    main()
```

**Verification:** run it, read the printed threshold sweeps + per-dataset breakdown. Must
complete without exceptions and produce `experiment/09/gate_signals.csv` with 164 rows
(matching `sweep_combined.csv`'s row count, modulo any `window_too_small` drops).

**Commit message:** `feat(exp09): validate_gate.py — calibrate canopy gate against Exp08 ground truth`

---

## Task 5 — Pick thresholds, hardcode, re-snapshot

From Task 4's printed sweep, for each signal (`green_dominance`, `repetitiveness`)
independently: pick the **lowest** threshold (maximizing recall / flagged fraction, i.e.
maximizing compute savings) subject to **precision ≥ 0.95**. If no threshold reaches 0.95
precision for a signal, do not force one — record in `result.md` that this signal isn't
viable as a gate on its own, and evaluate `color_texture` mode's *combined* false-positive
rate specifically (a row can be correctly saved by whichever of the two signals fires,
but a false positive requires BOTH signals to have missed — i.e. the OR logic in
`is_canopy_nonviable` only produces a false positive if `green_dominance` alone would have
been a false positive AND `repetitiveness` alone would have been a false positive on that
same row; check this by re-running `eval_thresh`-style logic on `(gd>=t1)|(rep>=t2)` at the
two chosen per-signal thresholds, not just trusting each signal's isolated report).

Pay special attention to the **dataset 1 false-positive count** printed by Task 4's last
block — dataset 1 is the only dataset with a meaningful number of real successes to lose;
zero false positives there is the practical bar, not just the aggregate precision number.

Edit `pipeline.py`'s two `None` constants to the chosen values, e.g.:
```python
GREEN_DOMINANCE_THRESH = 18.4   # placeholder shown for illustration only —
REPETITIVENESS_THRESH  = 0.62   # Task 4's actual sweep output determines the real numbers
```
(The exact values are Task 4's output, not predicted here — this plan cannot know them in
advance since they're calibrated from data the plan doesn't have access to yet.)

Re-run:
```bash
cp pipeline.py experiment/09/pipeline.py
```

**Verification:** `conda run -n drone python -c "
import sys; sys.path.insert(0, 'experiment/09')
import pipeline as pl
assert pl.GREEN_DOMINANCE_THRESH is not None
assert pl.REPETITIVENESS_THRESH is not None
print('thresholds set:', pl.GREEN_DOMINANCE_THRESH, pl.REPETITIVENESS_THRESH)
"`

**Commit message:** `feat(pipeline): calibrate canopy-gate thresholds from Exp09 validation (Exp09)`

---

## Task 6 — Smoke test A: dataset 2 (canopy) — latency, no accuracy change expected

```bash
conda run -n drone python pipeline.py --dir _in/isaac-sim-20260704_205334 \
    --max_frames 6000 --canopy_gate off \
    --out _out/exp09_ds2_gate_off.png > /tmp/exp09_ds2_off.log 2>&1
conda run -n drone python pipeline.py --dir _in/isaac-sim-20260704_205334 \
    --max_frames 6000 --canopy_gate color_texture \
    --out _out/exp09_ds2_gate_ct.png > /tmp/exp09_ds2_ct.log 2>&1
```

Compare wall-clock time (`time` prefix each command, or note start/end timestamps) and the
printed `Canopy-gate skips` / `DSMAC fixes att/acc` lines between the two runs. **Expected:**
fixes att/acc stays at (or near) 0/0 in both (dataset 2 has no real matchable structure
either way), `canopy_gate off` reports 0 skips, `canopy_gate color_texture` reports a
nonzero skip count with a measurably lower wall-clock time (fewer SIFT+LightGlue calls).

**Verification:** both runs exit 0; skip count in the gated run > 0; gated run's wall-clock
< ungated run's wall-clock.

---

## Task 7 — Smoke test B: dataset 1 (farmland success) — regression check

This is the critical check: confirm the gate does not suppress real fixes on the one dataset
that actually works.

```bash
conda run -n drone python pipeline.py --dir _in/isaac-sim-20260630_152940 \
    --canopy_gate off \
    --out _out/exp09_ds1_gate_off.png > /tmp/exp09_ds1_off.log 2>&1
conda run -n drone python pipeline.py --dir _in/isaac-sim-20260630_152940 \
    --canopy_gate color_texture \
    --out _out/exp09_ds1_gate_ct.png > /tmp/exp09_ds1_ct.log 2>&1
```

(Full flight, no `--max_frames` — this is the regression check, needs to be representative.)

**Verification:** `canopy_gate off` run's fused RMSE / fixes att-acc should reproduce
plan.md's recorded baseline (370.3 m / 2.96%, 92/89 fixes) within normal run-to-run variance
(per root `CLAUDE.md`: "single-run TUM-VI ATE is non-deterministic" — same caveat applies
here, so an exact match isn't expected, but it should be in the same ballpark). The
`color_texture` run's fixes accepted count must not be **meaningfully lower** than the
ungated run's — any material drop here means the calibrated threshold is too aggressive and
Task 5 needs revisiting before this gate is trustworthy to leave wired in.

---

## Task 8 — `experiment/09/result.md`

Write up:
- The Task 4 calibration table (both signals' threshold sweeps + chosen values + rationale).
- The dataset-1 false-positive check specifically (the regression risk).
- Smoke test A's wall-clock/skip-count numbers (the actual latency payoff, quantified).
- Smoke test B's before/after fixes-accepted/RMSE comparison (the regression check, pass/fail).
- An explicit restatement, in the conclusion, of this experiment's scope limit: canopy
  detection changed **zero** fix outcomes on datasets 2/4 by design — it is a compute
  optimization, not an accuracy improvement, and the open question from Exp08 (a real
  correction source for canopy-covered legs, e.g. terrain-relief/DEM correlation instead of
  optical matching) remains unsolved and out of scope here.

---

## Deliverables checklist

- [ ] `pipeline.py` (root): gate functions, CLI flags, wiring, calibrated thresholds
- [ ] `experiment/09/pipeline.py`, `experiment/09/flow_odometry.py` (final snapshots)
- [ ] `experiment/09/validate_gate.py`
- [ ] `experiment/09/gate_signals.csv` (164-row calibration data)
- [ ] `experiment/09/result.md`
- [ ] `_out/exp09_ds2_gate_off.png`, `_out/exp09_ds2_gate_ct.png` (smoke test A plots)
- [ ] `_out/exp09_ds1_gate_off.png`, `_out/exp09_ds1_gate_ct.png` (smoke test B plots)
