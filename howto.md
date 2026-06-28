# How to run pipeline.py

Full GPS-denied navigation: LK optical-flow odometry fused with SIFT+LightGlue DSMAC geo-localization.

## Prerequisites

- conda env `drone` (CUDA 12.8, RTX 5090)
- Dataset directory with: `cam_calib.json`, `frames.csv`, `poses.csv`, `baro.csv`,
  `imu.csv`, `geo.csv`, `georef.json`, `agl_cache.npz`, `ortho_tiles/`

## Basic run

```bash
conda run -n drone python pipeline.py --dir _in/isaac-sim-20260625
```

Output plot saved to `_out/pipeline_<dataset>.png`.

## Common options

| Flag | Default | Description |
|---|---|---|
| `--dir` | `_in/isaac-sim-20260625` | Dataset directory |
| `--stride` | 5 | Process every Nth frame |
| `--scale` | 0.5 | Image resize factor |
| `--blend` | 0.8 | DSMAC correction weight: `pos += blend*(fix - pos)` |
| `--fix_every` | 30 | Attempt DSMAC every N flow-odom steps |
| `--reject` | 150.0 | Reject fix if farther than this from prior (m) |
| `--skip_below` | 13.0 | Skip DSMAC until estimated drift exceeds this (m) |
| `--min_inliers` | 15 | Min RANSAC inliers to accept a fix |
| `--win` | 420 | DSMAC ortho search half-window (px) |
| `--out` | auto | Output plot path |

## Autotune mode

Replaces the fixed `--blend` with a Kalman-style adaptive blend estimated from data.

```bash
conda run -n drone python pipeline.py --dir _in/isaac-sim-20260625 --autotune
conda run -n drone python pipeline.py --dir _in/isaac-sim-20260625 --autotune --warmup_fixes 8
```

How it works:
1. First `--warmup_fixes` accepted fixes (default 6): blend=1.0 (snap to fix), residuals collected.
2. After warmup: `dsmac_std` frozen from warmup residuals.
3. Per-fix blend: `blend = flow_var / (flow_var + dsmac_var)`, scaled by inlier confidence, clipped to [0.3, 1.0].

Use autotune for short flights or variable-quality DSMAC. Use fixed `--blend 0.8` for long, consistent flights.

## Sweep blends manually

```bash
for b in 0.5 0.6 0.7 0.8 0.9 1.0; do
    conda run -n drone python pipeline.py --dir _in/isaac-sim-20260625 --blend $b
done
```

## Short vs long dataset

```bash
# Short (~7.5 min, 1773 m)
conda run -n drone python pipeline.py --dir _in/isaac-sim-20260624_2337

# Long (~77 min, 22 km)
conda run -n drone python pipeline.py --dir _in/isaac-sim-20260625
```

## Key outputs

- Console: path length, duration, fix count, RMSE, final error
- Plot (2x2): top-down trajectory, error over time, config panel, LK inlier counts
