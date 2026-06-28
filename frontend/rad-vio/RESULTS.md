# RaD-VIO (homography + rangefinder) — reproduction results

**Paper:** *RaD-VIO: Rangefinder-aided Downward Visual-Inertial Odometry*,
arXiv:1810.08704.

**Reproduced core:** the downward camera sees ~one ground plane → estimate the
inter-frame **homography**, `cv2.decomposeHomographyMat` into {R, t/d, n}, pick the
physically-visible solution whose rotation matches the IMU/attitude-derived relative
rotation (IMU-aided), then fix monocular scale with the **rangefinder**:
`t_metric = (t/d)·d`, d = AGL. Global sign resolved from flow physics (no GT).

**Shared harness (for a fair comparison):** LK tracking, AGL rangefinder stand-in,
GT or Mahony AHRS attitude. No GT *position* used.

## Result — `_in/isaac-sim-20260624_2337` (1773 m / 450 s, scale 0.5, depth=agl, GT-att)

| stride | final | mean | RPE | scale |
|---|---|---|---|---|
| 5  | 6.30% (112 m) | 5.12% | 36.7% | 0.55 |
| 15 | 6.54% | 4.29% | 35.4% | 0.56 |
| 30 | 7.49% | 4.02% | 33.2% | 0.58 |

Plots: `rad_vio_agl_gt_s{5,15,30}.png` · metrics: `rad_vio_agl_gt_s*_metrics.json`

## Comparison (identical tracks / depth / attitude — only the estimation core differs)

| method | est. core | final | RPE | scale |
|---|---|---|---|---|
| **flow-odom (ours)** | per-point ray–ground LSQ, per-point depth | **0.3%** (4.6 m) | **2.42%** | 0.98 |
| Downfacing-VIO | global avg flow, uniform nadir depth | 1.33% | 5.46% | 0.95 |
| RaD-VIO (this) | ground-plane homography decomposition | 6.3% | 36.7% | 0.55 |

**Takeaway:** RaD-VIO is the **weakest** core here, and crucially the error is
**stable across stride** (5→30) — so it is *not* parallax-starvation. The single-plane
homography is systematically mis-scaled (scale ~0.56, ≈1.8× too small) and directionally
noisy (RPE ~35%) because high-altitude nadir terrain (AGL 1–145 m) is **not one plane**:
the fitted homography is a compromise that under-estimates parallax. RaD-VIO's design
regime is **low-altitude over near-planar ground** (where homography decomposition is
well-conditioned); on this dataset the per-point-depth model (flow-odom) is ~20× better
on RPE. This is the project's recurring thesis — high-altitude nadir breaks
planarity/low-parallax assumptions — seen through a third estimation backbone.

Run:
```
conda run -n cv python frontend/rad-vio/rad_vio.py \
    --dir _in/isaac-sim-20260624_2337 --scale 0.5 --depth agl --stride 5
# --attitude ahrs  for the no-GT deployable variant
```
