# Test Plan — Real-time Latency Profiling

**Conda env:** `drone`  
**Scripts to copy into this folder:** `flow_odometry.py`, `fuse_flowodom_dsmac.py`, `dsmac_match.py`

---

## Context

Exp01 and Exp02 validated accuracy. Latency was deferred both times.
This experiment answers: **can the pipeline run in real-time at 12.5 Hz (80 ms/frame)?**

No live stream is needed — replay saved frames from `_in/` with a timer to simulate real-time.

---

## Dataset

Use **short dataset only** (`_in/isaac-sim-20260624_2337`, 7.5 min).
Long dataset is not needed — latency is hardware/algorithm dependent, not flight-length dependent.

---

## Method: Simulated Real-time Replay

Feed frames one-by-one from disk, time each processing step, and track budget usage.

```python
import time

FRAME_PERIOD = 1 / 12.5  # 80 ms at 12.5 Hz
FIX_BUDGET   = 30 * FRAME_PERIOD  # 2.4 s — DSMAC fires every 30 frames

for i, frame in enumerate(frames):
    t0 = time.perf_counter()
    process_frame(frame)          # flow-odom step
    latency = time.perf_counter() - t0

    slack = FRAME_PERIOD - latency
    if slack > 0:
        time.sleep(slack)
    else:
        print(f"frame {i}: OVER BUDGET by {-slack*1000:.1f} ms")
```

DSMAC fixes run in a **background thread** — they must not block the main loop.
Measure fix latency separately and confirm it fits within the 2.4 s window.

---

## Experiment 1: Flow-odom Per-frame Latency

Profile each sub-step of a single flow-odom frame:

| Sub-step | Expected |
|----------|----------|
| Image load from disk | < 5 ms |
| LK feature detect (`goodFeaturesToTrack`) | < 5 ms |
| LK optical flow (`calcOpticalFlowPyrLK`) | < 10 ms |
| Forward-backward check | < 3 ms |
| De-rotate + LS solve | < 2 ms |
| **Total per frame** | **< 20 ms** |

Run with `--stride 5`, `--scale 0.5` (same as Exp01/02).

**Metrics to record:**

| Metric | How |
|--------|-----|
| Mean per-frame latency (ms) | `time.perf_counter()` around each step |
| P95 per-frame latency (ms) | worst-case frames (scene changes, many features) |
| Real-time margin (ms) | `80 ms − mean latency` |
| Over-budget frame count | frames where total > 80 ms |

---

## Experiment 2: DSMAC Per-fix Latency

Profile one complete DSMAC fix cycle:

| Sub-step | Expected |
|----------|----------|
| Satellite tile fetch / cache lookup | < 10 ms (cached) |
| ALIKED keypoint detection | 100–300 ms |
| LightGlue matching | 100–400 ms |
| RANSAC homography | < 20 ms |
| **Total per fix** | **200–800 ms** |

Fix budget: 30 frames × 80 ms = **2.4 s**.

**Metrics to record:**

| Metric | How |
|--------|-----|
| Mean fix latency (ms) | timed over all 45 fix attempts |
| P95 fix latency (ms) | worst-case fix |
| Fix budget headroom (ms) | `2400 ms − mean fix latency` |
| Must background? | fix latency > 80 ms → yes |

---

## Experiment 3: Full Pipeline (Flow-odom + DSMAC Background Thread)

Simulate the deployed architecture:
- Main loop runs flow-odom at 12.5 Hz
- DSMAC fix runs in a background thread, posts result to a queue
- Main loop applies fix from queue when available (non-blocking)

**Metrics to record:**

| Metric | How |
|--------|-----|
| Main loop frame rate (Hz) | wall-clock frames / total time |
| Fix application lag (frames) | how many frames pass between fix ready and applied |
| End-to-end position error | same RMSE as Exp02 G-L to confirm threading doesn't break accuracy |

**What to look for:**
- Main loop must sustain ≥ 12.5 Hz regardless of DSMAC
- Fix application lag should be ≤ 1 frame (fix posted → applied next frame)
- RMSE should match Exp02 G-L (35.7 m) within noise

---

## Commands

```bash
# Exp 1 — flow-odom latency
conda run -n drone python experiment/03/flow_odometry.py \
    --dir _in/isaac-sim-20260624_2337 --depth agl --stride 5 --benchmark

# Exp 2 — DSMAC fix latency
conda run -n drone python experiment/03/dsmac_match.py \
    --dir _in/isaac-sim-20260624_2337 --benchmark

# Exp 3 — full pipeline with background thread
conda run -n drone python experiment/03/fuse_flowodom_dsmac.py \
    --dir _in/isaac-sim-20260624_2337 --attitude ahrs_compass \
    --reject 45 --skip_below 13 --realtime
```

Add `--benchmark` / `--realtime` flags to the copied scripts for this experiment.

---

## Summary Table (targets)

| Component | Budget | Target | Pass condition |
|-----------|--------|--------|----------------|
| Flow-odom per frame (mean) | 80 ms | < 20 ms | 4× margin |
| Flow-odom per frame (P95) | 80 ms | < 40 ms | 2× margin |
| DSMAC per fix (mean) | 2 400 ms | < 800 ms | 3× margin |
| DSMAC per fix (P95) | 2 400 ms | < 1 200 ms | 2× margin |
| Main loop sustained rate | 12.5 Hz | ≥ 12.5 Hz | no drop |

---

## Open Items Carried from Exp02

- **Compass noise sensitivity:** test `mag_noise_deg > 0` in flow_odometry.py to see how much real magnetometer noise degrades C2 accuracy — add as Exp4 here or defer to Exp04
- **Short-flight tilt gap:** G (13.7 m) vs E (10.5 m) root cause is AHRS roll/pitch scale error (1.31) — not addressed yet
