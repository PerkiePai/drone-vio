# Front-end tracking comparison: OpenVINS-KLT vs ALIKED+LightGlue

Compares the **default OpenVINS feature front-end** against a **learned** one on
the *same* real MARS-LVIG **AMvalley** flight frames (nadir forest, ~80–130 m),
to see whether a learned detector+matcher tracks this footage better for VIO.

- **KLT** — FAST corners + pyramidal Lucas–Kanade optical flow with
  forward–backward consistency rejection. This is what OpenVINS actually uses
  (`TrackKLT`).
- **ALIKED+LightGlue** — learned detector (ALIKED) + learned matcher (LightGlue,
  `features="aliked"`), via the pip `lightglue` package.

No intrinsics needed: match quality is the **RANSAC fundamental-matrix inlier
ratio** (same no-calibration proxy as the other `frontend/` harnesses).

## Run (conda env `cv`: torch + lightglue)
```bash
# 1. extract a contiguous 200-frame cruise segment from the bag (ROS container)
docker run --rm -v "$PWD/../../_in":/data \
  -v "$PWD":/work openvins:noetic bash -lc \
  'source /opt/ros/noetic/setup.bash && python3 /work/extract_frames.py /data/mars-lvig/AMvalley01.bag /work/_frames 200 200'

# 2. compare (table + CSV + plots)
conda run -n cv python compare_tracking.py --scale 0.5 --gaps 1 2 3 5 10 --pairs 25 --surv_T 30 --viz
```
Outputs in `_out/`: `pairwise.csv/png`, `survival.csv/png`, `matches_gap{1,10}.png`.

## Results (1224×1024, max 1024 kpts each)

**Pairwise vs frame gap** (over 25 pairs; `min` = worst pair = VIO-survival floor):

| method | gap | matches (mean / **min**) | inlier ratio | latency |
|---|---|---|---|---|
| KLT | 1 | 1023 / **1017** | 1.000 | ~81 ms* |
| ALIKED+LightGlue | 1 | 751 / 725 | 0.998 | 2.8 ms† |
| KLT | 5 | 952 / **555** | 0.984 | ~46 ms* |
| ALIKED+LightGlue | 5 | 720 / 632 | 0.987 | 2.8 ms† |
| KLT | 10 | 862 / **73** ⚠️ | 0.953 | ~32 ms* |
| ALIKED+LightGlue | 10 | 684 / **501** | **0.967** | 2.8 ms† |

The gap-10 panel (`_out/matches_gap10.png`) shows this concretely: over low-texture
open ground, KLT collapses to 73 matches while ALIKED+LightGlue keeps 501.

\* KLT latency = FAST detect + **bidirectional** LK (CPU).
† LightGlue latency = **matcher only** (GPU); ALIKED extraction is separate/cached,
matching the `frontend/` convention — so these columns are *not* like-for-like wall time.

**Track survival** (seeded on frame 0, % surviving):

| frames | KLT | ALIKED+LightGlue |
|---|---|---|
| 1 | 100% | 74% |
| 5 | 100% | 46% |
| 10 | 99% | 36% |
| 30 | 98% | 25% |

## Takeaways for VIO
It's a genuine trade — neither wins outright:

- **KLT wins the common case (mean track count + persistence).** Inter-frame
  motion is tiny (1023/1024 at gap 1 → smooth low-parallax nadir cruise), so
  optical flow tracks the same patches for seconds (98% survival over 30 frames).
  MSCKF loves long tracks, so the default front-end fits the *typical* frame here.
- **ALIKED+LightGlue wins the worst case (robustness floor).** At gap 10 KLT's
  floor collapses to **73** matches on low-texture ground while ALIKED+LightGlue
  holds **501**, with a higher inlier ratio (0.967 vs 0.953) and ~15–30× cheaper
  matching (2.8 ms on the 5090). Its weakness is **chained-track persistence**:
  detect-then-match depends on detector repeatability, which decays each frame
  (~25% surviving by frame 30).
- **Net for VIO:** on smooth textured cruise, KLT's long continuous tracks are
  hard to beat — use it as the default. But KLT has a **collapse tail** on
  low-texture / wide-baseline frames, which is exactly when a VIO loses its track.
  A learned front-end (or KLT *backed by* ALIKED+LightGlue when the KLT count
  drops below the factor-graph minimum) buys a much higher survival floor — the
  same "adaptive fallback" idea as `frontend/`'s XFeat+LGdyn. This low-parallax
  regime is also why monocular VIO *init* is hard on this dataset (`backend/openvins`).

## Files
- `extract_frames.py` — pull frames from a MARS-LVIG bag (run in `openvins:noetic`).
- `compare_tracking.py` — the comparison (KLT vs ALIKED+LightGlue; `--viz` for plots).
- `_frames/`, `_out/` — gitignored (large images / results).
