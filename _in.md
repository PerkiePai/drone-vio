# `_in/` — Dataset Inventory

All datasets live in `_in/` (gitignored). This file is the human-readable index.

---

## Real-world video clips (matcher experiments)

### `bev-forest/`
Nadir (bird's-eye-view) footage of a forest canopy. One MP4 + 14 sampled JPEG frames.
Used for early SuperGlue / LightGlue feature-matching experiments.

### `fpv-mountain-poor-cam/`
FPV forward-facing video over a mountain scene. Poor camera quality (motion blur, low bitrate).
One MP4 + 68 sampled JPEG frames. Used to stress-test matchers on degraded FPV input.

### `fpv-over-forest-poor/`
FPV forward-facing video over forest. Also poor quality (similar conditions to above).
One MP4 + 29 sampled JPEG frames.

---

## Isaac Sim synthetic flights (primary VIO / flow-odom testbed)

All four are recorded with the **PAI vio-recorder** in Isaac Sim over simulated Thai terrain
(~Bangkok coordinates). Each contains: `poses.csv` (GT ENU, starts at 0,0,0), `imu.csv`
(400 Hz, frame-synced), `baro.csv`, `cam_calib.json` (cam0 nadir), `takeoff.json`
(absolute ENU origin + attitude), `geo.csv` (GPS-equivalent lat/lon/alt for DSMAC),
and `images/cam0/` frames. Later recordings also have `cam1` (FPV), `agl.csv`, and
`georef.json`.

| Dataset | Date | Duration | GT range | Alt (mean) | cam0 frames | cam1 frames | Notes |
|---|---|---|---|---|---|---|---|
| `isaac-sim-20260623` | 2026-06-23 | 2.6 min | 203 m | 25.6 m | 1 925 | — | First rigid-extrinsic recording; VIO fails (terrain depth ≫ baro alt) |
| `isaac-sim-20260624` | 2026-06-24 | 3.8 min | 513 m | 27.9 m | 6 690 | 1 603 | Added cam1 + AGL bag; duplicate buggy calib files present |
| `isaac-sim-20260624_2337` | 2026-06-24 23:37 | 7.5 min | 457 m | 43.5 m | 14 070 | 7 433 | **Main ablation dataset.** Full suite of results: flow-odom vs GT, DSMAC geo-loc, RaD-VIO baselines, fused flow+DSMAC, tracker RPE comparison. Includes 144 ortho tiles for DSMAC |
| `isaac-sim-20260625` | 2026-06-25 | **76.9 min** | **9 522 m** | 48.9 m | 67 863 | 67 863 | **Long-mission dataset (~22 km).** Used for attitude comparison (GT vs Mahony vs Mahony+compass). Both cams matched frame-for-frame |

### Key findings stored per-dataset

- **20260623**: `flow_vs_gt.png`, `flow_vs_gt_agl.png`, `gt_trajectory_3d.png`, `gt_vs_vio.png`
- **20260624**: `flow_vs_gt_agl_s5.png`, `flow_vs_gt_baro.png`
- **20260624_2337**: `flow_track_RPE_compare.png`, `dsmac_*.png`, `fused_flowodom_dsmac.png`, `rad_vio_*.png`, `hybrid_caseA_exp1.png`, `crossover_longmission.png`, `attitude_compare.png` (copied)
- **20260625**: `attitude_compare.png`, `flow_vs_gt_agl.png` (half + second-half splits)

---

## Standard VIO benchmarks (backend / OpenVINS)

### `tumvi/`
[TUM-VI](https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset) indoor sequences.
- `dataset-room1_512_16.bag` (2.8 GB) — **primary OpenVINS testbed**; ATE ~5 cm mono
- `dataset-room2_512_16.bag` (2.9 GB) — secondary room sequence

### `euroc/`
[EuRoC MAV](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) dataset.
- `MH_01_easy.bag` (2.5 GB) — Machine Hall easy; untuned ATE ~33.8 cm

### `mars-lvig/`
[MARS-LVIG](https://mars.hku.hk/dataset.html) large-scale aerial VIO dataset.
- `AMvalley01.bag` (26 GB) — km-scale outdoor flight; OpenVINS currently drifts (km error)
- `calib/` — sensor extrinsics (`CAD_extrinsic.yaml`, `f_extrinsic/`)
