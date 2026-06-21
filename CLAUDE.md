# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Purpose

Feature-matching experiments for drone visual-inertial odometry (VIO). The work
compares two learned matchers — **SuperGlue** and **LightGlue** — on top-down
(nadir) aerial drone forest footage, measuring match quality and latency to
decide what to use in a VIO front-end.

## Environment

All Python runs in the conda env **`car-detection`** (torch 2.5.1+cu121 with
CUDA, OpenCV, and the pip-installed `lightglue` package). The base env has no
torch. Always invoke through the env:

```
conda run -n car-detection python <script> <args>
```

Platform is Windows; the shell is PowerShell. When passing native-exe args,
mind PowerShell quoting (see the tool guidance). `2>$null` is handy to drop the
harmless `torch.load` FutureWarnings the SuperGlue weights emit.

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
```

## Layout

All matcher code lives under `frontend/`; `_in/` (shared data) stays at the
project root. Scripts resolve `_in` via `ROOT` (two levels up from a
`frontend/<sub>/` script) and the vendored SuperGlue repo via `FRONTEND` (one
level up).

- `_in/` — shared inputs at the project root: source `*.mp4` plus extracted
  frames. **Gitignored.**
- `frontend/` — all matcher code (superglue, lightglue, xfeat, compare) plus the
  vendored `SuperGluePretrainedNetwork/`. See `frontend/CLAUDE.md` for the
  per-folder layout and the full architecture.
- All `_out/` dirs and `_in/` are gitignored; only the scripts are tracked.

## Architecture

The matcher/extractor architecture, the fairness harnesses, the XFeat +
LighterGlue + adaptive-confidence design, and the geometry-without-calibration
notes live in **`frontend/CLAUDE.md`** (loaded on demand when working under
`frontend/`).
