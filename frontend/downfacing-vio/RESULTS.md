# Downfacing-VIO (PX4FLOW core) — reproduction results

**Paper:** *Efficient and Accurate Downfacing Visual Inertial Odometry*,
Rutishauser et al., arXiv:2509.10021 (IEEE 2025).

**Reproduced core:** PX4FLOW model — gyro-de-rotate the image flow, take a single
robust **global average** flow vector, assume the ground is one flat plane at a
**uniform depth** Z = rangefinder AGL, convert to metric velocity
`v_xy = -(mean_derotated_flow/dt)·Z`. Rigid planar-motion model, 2-DOF translation.

**Shared harness (for a fair comparison with flow-odom):** LK pyramidal tracking,
AGL as the rangefinder stand-in (`agl_cache.npz` / `compute_true_agl`), GT or Mahony
AHRS attitude. No GT *position* used.

## Result — `_in/isaac-sim-20260624_2337` (1773 m / 450 s, scale 0.5, depth=agl, stride 5, GT-att)

| metric | value |
|---|---|
| final error | 23.6 m (**1.33%**) |
| mean error | 26.9 m (1.52%) |
| RMSE | 29.4 m |
| KITTI segment drift (RPE) | **5.46%** |
| scale (est/GT) | 0.95 |

Plot: `downfacing_vio_agl_gt_s5.png` · metrics: `downfacing_vio_agl_gt_s5_metrics.json`

## Comparison (identical tracks / depth / attitude — only the estimation core differs)

| method | est. core | final | RPE | scale |
|---|---|---|---|---|
| **flow-odom (ours)** | per-point ray–ground LSQ, per-point depth | **0.3%** (4.6 m) | **2.42%** | 0.98 |
| Downfacing-VIO (this) | global avg flow, uniform nadir depth | 1.33% (23.6 m) | 5.46% | 0.95 |
| RaD-VIO | ground-plane homography decomposition | 6.3% (112 m) | 36.7% | 0.55 |

**Takeaway:** the PX4FLOW global-average / uniform-depth model gets the *scale* right
(0.95) — robust median flow + median AGL — but ~5× more drift than flow-odom because a
single uniform depth is wrong over this steep terrain (AGL 1–145 m). It is faithful to
the paper's design regime (nano-UAV, low altitude, near-flat ground) and degrades
exactly where per-point depth (flow-odom) wins: varying terrain at altitude.

Run:
```
conda run -n cv python frontend/downfacing-vio/downfacing_vio.py \
    --dir _in/isaac-sim-20260624_2337 --scale 0.5 --depth agl --stride 5
# --attitude ahrs  for the no-GT deployable variant
```
