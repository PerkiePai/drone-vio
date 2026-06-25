# CLAUDE.md

Guidance for Claude Code working in this repo.

## Goal

A **GPS-denied drone self-navigator** localizing from cameras + IMU only (no GNSS).
The drone has **two independent monocular cameras** — **downward (nadir)** and
**forward FPV** — not a stereo rig (each is its own mono sensor, no cross-camera
constraints). Current phase: **downward camera first**. Everything here is
**monocular, IMU+camera, no GPS**; the focus is the nadir regime and its
low-parallax difficulty at altitude.

## Two parts

- **`frontend/`** — feature-matching experiments (SuperGlue, LightGlue, XFeat +
  extractors) on nadir aerial footage, plus `flow-odom/` (altitude-scaled
  optical-flow odometry). See `frontend/CLAUDE.md`.
- **`backend/`** — real VIO: **OpenVINS** (mono, IMU+camera, no GNSS) on MARS-LVIG,
  TUM-VI, EuRoC, and Isaac Sim, in a ROS1-Noetic Docker image. See `backend/CLAUDE.md`.

## Environment

- **Python:** conda env **`cv`** (torch + CUDA, `lightglue` pip-installed). Invoke as
  `conda run -n cv python <script>`.
- **Backend:** Docker image `openvins:noetic`.

(An older Windows/`car-detection` setup is deprecated; ignore references to it.)

## Commands

```bash
# Frontend (cv env)
conda run -n cv python frontend/superglue/capture_frames.py        # 1 frame/s from _in/*.mp4
conda run -n cv python frontend/superglue/superglue_match.py --n 0 --m 6
conda run -n cv python frontend/lightglue/lightglue_match.py --n 0 --m 1
conda run -n cv python frontend/xfeat/xfeat_match.py --n 0 --m 1
conda run -n cv python frontend/compare/compare_matchers.py --gaps 1 3 6 12
conda run -n cv python frontend/compare/compare_extractors.py --gaps 1 3 6 12
conda run -n cv python frontend/openvins-alike-lightglue/compare_tracking.py \
    --scale 0.5 --gaps 1 2 3 5 10 --pairs 25 --surv_T 30 --viz
# Altitude-scaled optical-flow odometry (--depth agl + --stride 5 best, ~0.3%)
conda run -n cv python frontend/flow-odom/flow_odometry.py --dir _in/isaac-sim-20260624_2337 \
    --scale 0.5 --depth agl --stride 5

# Backend (Docker)
docker build -t openvins:noetic -f backend/openvins/Dockerfile backend/openvins
backend/openvins/run_tumvi.sh          # known-good mono VIO + ATE (~5 cm)
backend/openvins/run_openvins_viz.sh   # MARS-LVIG; X11 RViz + mp4
backend/openvins/accuracy_sweep.sh     # try many configs, report ATE
```

## Result summary

Mono VIO **works on TUM-VI room1** (ATE ~5 cm) but **fails on MARS-LVIG** (km drift)
and **Isaac Sim above ~20 m**: high-altitude nadir is low-parallax, so monocular
scale is unobservable. A real init bug was found & fixed (`init_max_disparity: 80→1.5`
— dynamic init never fired) → scale locks below ~20 m but runs away as the drone
climbs over deep terrain. This is an **observability limit, not a tuning knob**.
Full-flight metric coords need a structural fix: **true AGL (rangefinder)**, the
**FPV camera** (multi-cam), or a **low-altitude re-record**. The depth-based
flow-odom path reaches **~0.3%** once given true AGL. See `backend/CLAUDE.md` and
`frontend/CLAUDE.md` for details. **Always repeat-run before claiming a VIO
improvement** — single-run TUM-VI ATE is non-deterministic (±~1.5 cm).

## Layout

- `_in/` — shared inputs at project root (source `*.mp4`, frames, MARS-LVIG `*.bag`).
  **Gitignored.** Scripts resolve it via `ROOT` (two levels up from `frontend/<sub>/`).
- `frontend/` — matcher code + vendored `SuperGluePretrainedNetwork/` (gitignored) +
  `flow-odom/`. See `frontend/CLAUDE.md`.
- `backend/` — `data/` (MARS-LVIG provenance) and `openvins/` (Dockerfile, run
  scripts, configs, cloned `open_vins/`). See `backend/CLAUDE.md`.
- All `_out/`, `_result/`, `_frames/`, `_in/`, and `open_vins/` are gitignored; only
  scripts/configs are tracked.
