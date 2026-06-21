# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Purpose

Drone visual-inertial odometry (VIO), in two parts:

- **`frontend/`** — feature-matching experiments comparing learned matchers
  (**SuperGlue**, **LightGlue**, **XFeat**) and extractors on top-down (nadir)
  aerial footage, measuring match quality and latency to decide a VIO front-end.
- **`backend/`** — a real VIO pipeline: **OpenVINS** (monocular, IMU + camera
  only, no GNSS) on the **MARS-LVIG** aerial dataset, run in a ROS1-Noetic Docker
  image. See `backend/openvins/README.md`.

## Environment

All Python runs in the conda env **`car-detection`** (torch 2.5.1+cu121 with
CUDA, OpenCV, and the pip-installed `lightglue` package). The base env has no
torch. Always invoke through the env:

```
conda run -n car-detection python <script> <args>
```

The above describes the original Windows/PowerShell setup. On the **Ubuntu 24.04
box** this repo is now developed on, the `car-detection` env does **not** exist:
use the **`cv`** conda env (torch 2.12 + CUDA; `pip install` of `lightglue` done)
for frontend/Python work, and **Docker** (`openvins:noetic`) for the backend.
`2>$null` (PowerShell) → `2>/dev/null` here.

## Commands

```bash
# 1. Extract one frame per second from every video in _in/ back into _in/
conda run -n car-detection python frontend/superglue/capture_frames.py

# 2. SuperGlue match — by second index, or by explicit image paths
conda run -n car-detection python frontend/superglue/superglue_match.py --n 0 --m 6
conda run -n car-detection python frontend/superglue/superglue_match.py --img0 a.jpg --img1 b.jpg

# 3. LightGlue match — identical CLI to the SuperGlue script
conda run -n car-detection python frontend/lightglue/lightglue_match.py --n 0 --m 1

# 4. XFeat match — same CLI plus --top_k / --min_cossim
conda run -n car-detection python frontend/xfeat/xfeat_match.py --n 0 --m 1

# 5. Matcher comparison across frame gaps -> table + compare/_out/comparison_<stem>.csv
conda run -n car-detection python frontend/compare/compare_matchers.py --gaps 1 3 6 12

# 6. Extractor comparison (SuperPoint/SIFT/DISK/ALIKED under LightGlue)
#    -> table + compare/_out/extractors_<stem>.csv
conda run -n car-detection python frontend/compare/compare_extractors.py --gaps 1 3 6 12

# 7. OpenVINS-KLT vs ALIKED+LightGlue tracking on real MARS-LVIG frames
#    (cv env) -> table + plots + CSV in frontend/openvins-alike-lightglue/_out/
conda run -n cv python frontend/openvins-alike-lightglue/compare_tracking.py \
    --scale 0.5 --gaps 1 2 3 5 10 --pairs 25 --surv_T 30 --viz
```

### Backend (OpenVINS VIO, Docker)

```bash
# build once; then inspect topics and run (headless or with live viz + mp4)
docker build -t openvins:noetic -f backend/openvins/Dockerfile backend/openvins
backend/openvins/inspect_bag.sh
backend/openvins/run_openvins_viz.sh        # see backend/openvins/README.md
```

## Layout

All matcher code lives under `frontend/`; `_in/` (shared data) stays at the
project root. Scripts resolve `_in` via `ROOT` (two levels up from a
`frontend/<sub>/` script) and the vendored SuperGlue repo via `FRONTEND` (one
level up).

- `_in/` — shared inputs at the project root: source `*.mp4`, extracted frames,
  and the MARS-LVIG `*.bag` under `_in/mars-lvig/`. **Gitignored.**
- `frontend/` — matcher code (superglue, lightglue, xfeat, compare), the vendored
  `SuperGluePretrainedNetwork/`, and `openvins-alike-lightglue/` (KLT vs
  ALIKED+LightGlue front-end comparison). See `frontend/CLAUDE.md`.
- `backend/` — `data/` (MARS-LVIG provenance: Drive IDs, calibration) and
  `openvins/` (Dockerfile, run scripts, configs, cloned `open_vins`). See
  `backend/openvins/README.md`.
- All `_out/`, `_result/`, `_frames/` dirs and `_in/` are gitignored, as is the
  cloned `backend/openvins/open_vins/`; only the scripts/configs are tracked.

## Architecture

The matcher/extractor architecture, the fairness harnesses, the XFeat +
LighterGlue + adaptive-confidence design, the KLT-vs-ALIKED+LightGlue comparison,
and the geometry-without-calibration notes live in **`frontend/CLAUDE.md`**.

The VIO backend (OpenVINS sensor config, the IMU g→m/s² fix, dynamic-init notes,
and the live-viz/mp4 pipeline) lives in **`backend/openvins/README.md`**.
