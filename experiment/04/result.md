# Experiment 04 — Results: LK vs Alternative Trackers

**Date:** 2026-06-28  
**Scripts:** `flow_odometry.py`, `compare_trackers.py`

---

## Run 1 — Short dataset (2026-06-28, earlier session)

**Dataset:** `_in/isaac-sim-20260624_2337` (7.5 min, 1 773 m, ~86 m mean AGL)  
**Config:** `--depth agl --stride 5 --attitude ahrs_compass` (no DSMAC fusion)

### Bug fixes applied

1. **`compare_trackers.py`** — `SyntaxError`: positional `width` after keyword `bottom=` in
   `ax.bar()`. Fixed to `width=width`.
2. **`flow_odometry.py` `_orb_match`** — `cv2.findFundamentalMat` raises `cv2.error` on
   degenerate near-planar nadir points (OpenCV 4.13). Wrapped in `try/except cv2.error`.

### Accuracy + Latency (budget 80 ms)

| Tracker | RMSE | Final | Inliers | vs LK | Total | Margin |
|---------|------|-------|---------|-------|-------|--------|
| lk | 21.4 m | 25.7 m | 576 | — | 4.2 ms | 75.8 ms |
| fast_lk | 19.7 m | 24.5 m | 579 | −8% | 4.2 ms | 75.8 ms |
| farneback | 21.0 m | 25.5 m | 578 | −2% | 34.1 ms | 45.9 ms |
| **dis** | **18.7 m** | **20.5 m** | 577 | **−13%** | 10.3 ms | 69.7 ms |
| orb | 23.4 m | 37.8 m | 366 | +9% | 5.2 ms | 74.8 ms |
| sparse_raft | 284.7 m | 7.6 m | 0 | FAIL | 47.0 ms | 33.0 ms |

### Key findings (short dataset)

- **DIS best accuracy**: 18.7 m RMSE (−13% vs LK), 10.3 ms (well within budget)
- **FAST+LK free gain**: 19.7 m at identical 4.2 ms — just swap the detector
- **RAFT failed**: FB error median 28 px (mean 72 px) — OOD on high-alt nadir, 0 inliers

---

## Run 2 — Long dataset with DSMAC fusion (2026-06-28)

**Dataset:** `_in/isaac-sim-20260625` (76.9 min, 22 407 m, ~79 m mean AGL)  
**Config:** `--depth agl --stride 5 --attitude ahrs_compass --skip_below 13 --reject 150`  
**New features added to Exp04 code:**
- `--skip_below 13`: skip initial frames where baro height < 13 m (takeoff, 124 frames)
- `--reject N`: after each tracker's flow-odom run, apply DSMAC satellite-ortho fusion
  with fix rejection threshold N metres (same algorithm as Exp02)

### Accuracy + DSMAC fusion (reject=150, fix_every=30, skip_below=13)

| Tracker | FO RMSE | Fused RMSE | Final | Fixes acc/att | Latency | Margin |
|---------|---------|------------|-------|---------------|---------|--------|
| **lk** | **77.2 m** | **39.0 m** | **24.1 m** | **71/72 (99%)** | 4.0 ms | 76.0 ms |
| fast_lk | 80.2 m | 121.9 m | 27.1 m | 44/64 (69%) | 4.4 ms | 75.6 ms |
| farneback | 79.3 m | 96.5 m | 40.8 m | 48/71 (68%) | 30.0 ms | 50.0 ms |
| dis | 107.6 m | 133.0 m | 50.0 m | 42/60 (70%) | 10.0 ms | 70.0 ms |
| orb | 184.1 m | 183.2 m | 95.9 m | 5/10 (50%) | 4.6 ms | 75.4 ms |
| sparse_raft | 5628.2 m | 5628.2 m | 230.9 m | 0/0 | 25.3 ms | 54.7 ms |

### Key findings (long dataset)

1. **LK wins outright on the long flight.** Flow-odom RMSE 77.2 m → fused 39.0 m (−50%).
   DSMAC accepts 71/72 fixes (99%) — LK's drift is consistent enough that the prior
   stays within 150 m of the fix position at nearly every cadence.

2. **DSMAC fusion hurts all non-LK trackers.** fast_lk, farneback, DIS, and ORB all have
   fused RMSE *higher* than their flow-odom-only RMSE. The fix acceptance rates explain
   why: these trackers accumulate drift unevenly — sometimes within 150 m, sometimes
   not — causing accepted fixes to land on the wrong side of the sawtooth and pull
   position away from GT.

3. **DIS regresses badly on the long dataset (107.6 m vs LK 77.2 m).** DIS was the
   best tracker on the short dataset (18.7 m, −13% vs LK). The reversal suggests DIS
   has higher per-step velocity variance or scale bias that compounds over 22 km but
   averages out over 1.8 km. The DSMAC fusion cannot recover it (133.0 m fused).

4. **ORB only gets 5/10 fixes accepted.** High drift variance means the prior wanders
   beyond 150 m before most fix opportunities, so DSMAC fires rarely and has no
   sustained effect.

5. **RAFT: same complete failure as on the short dataset.** Zero inliers across the
   entire 22 km flight — OOD domain issue persists.

### Short vs long dataset reversal

| Tracker | Short RMSE | Long FO RMSE | Long Fused RMSE |
|---------|------------|--------------|-----------------|
| **lk** | 21.4 m | **77.2 m** | **39.0 m** |
| fast_lk | 19.7 m | 80.2 m | 121.9 m |
| dis | **18.7 m** | 107.6 m | 133.0 m |

On the short dataset DIS led. On the long dataset LK leads by a wide margin.
**For fused long-flight navigation, LK is the correct choice.**

---

## Output files

- `experiment/04/isaac-sim-20260624_2337/compare_trajectories.png` — short dataset
- `experiment/04/isaac-sim-20260624_2337/compare_latency.png` — short dataset
- `experiment/04/isaac-sim-20260625/compare_trajectories.png` — long dataset + fused
- `experiment/04/isaac-sim-20260625/compare_latency.png` — long dataset

---

## Recommendation

| Use case | Best tracker | Config |
|----------|-------------|--------|
| Short flight (< 5 km), no DSMAC | **DIS** | 18.7 m RMSE, 10.3 ms |
| Short flight (< 5 km), no DSMAC, speed priority | **FAST+LK** | 19.7 m RMSE, 4.2 ms |
| Long flight (> 5 km) + DSMAC fusion | **LK** | 39.0 m RMSE / 22 km, 4.0 ms |

---

## Open items → Exp05

- **Compass noise sensitivity** (`mag_noise_deg > 0`): real magnetometer noise effect
- **Short-flight tilt gap**: roll/pitch scale error (1.31×) needs rangefinder or multi-cam
- **RAFT on nadir**: requires aerial fine-tuning (e.g. GMFlow-large) — currently OOD
- **DIS long-flight regression**: investigate whether scale bias or step-variance is the cause
