# CLAUDE.md — backend/

VIO backend: OpenVINS (monocular IMU+camera, no GNSS) in a ROS1 Noetic Docker image,
run on MARS-LVIG aerial bags, TUM-VI, EuRoC, and Isaac Sim. All runs happen inside
`docker run --rm openvins:noetic`; the host mounts bags/configs/scripts read-only,
only `_out/` and `_result/` are writable.

```
backend/
  data/                   # provenance docs only (mars_lvig_drive_ids.md, amvalley_calibration.md)
  openvins/
    Dockerfile            # osrf/ros:noetic-desktop-full + Ceres + catkin build
    open_vins/            # cloned rpng/open_vins (gitignored)
    config/{mars_lvig_amvalley,isaac_sim,isaac_sim_agl,...}/
    run_*.sh              # one script per dataset/task
    *.py                  # ROS helper nodes
```

## Docker image

```bash
docker build -t openvins:noetic -f backend/openvins/Dockerfile backend/openvins  # ~10 min
```
Rebuild when C++ in `open_vins/` changes; config-only changes are mounted at runtime.

### Dockerfile gotcha — image_transport v1.15.0
v1.15 has a class_loader bug: on `std::exit()`, static destructors unload a Poco-loaded
plugin → `LibraryUnloadException` → `terminate()`. Three fixes baked in:
1. `apt-get remove ros-noetic-compressed-depth-image-transport` (we never use depth).
2. Replace `image_transport::Publisher` with `ros::Publisher` in `ROS1Visualizer.h/.cpp`.
3. Any missing **required** YAML param also triggers `std::exit()` → same crash.
   **Rule:** every new optional OpenVINS YAML param MUST pass `required=false` as the
   3rd arg to `parse_config()`.

## Scripts

| Script | Dataset | Notes |
|--------|---------|-------|
| `run_tumvi.sh` | TUM-VI room1 | mono, ATE ~5 cm baseline (bag downloaded separately) |
| `run_openvins.sh` / `run_openvins_viz.sh` | MARS-LVIG AMvalley01 | headless / X11 RViz + mp4 |
| `run_marslvig_baro.sh` | MARS-LVIG AMvalley01 | real baro from `/dji_osdk_ros/local_position` |
| `run_euroc.sh` | EuRoC MH_01_easy | patches stereo→mono; ATE 33.8 cm untuned |
| `run_isaac_to_bag.sh` | Isaac Sim CSV | converts CSV+PNG → ROS bag |
| `run_isaac_sim.sh` | Isaac Sim bag | mono; 3rd arg = config subdir (e.g. `isaac_sim_agl`) |
| `inspect_bag.sh` | any | `rosbag info` in Docker |
| `accuracy_sweep.sh` / `repeat_eval.sh` | TUM-VI | config ATE sweep / repeat-run for stats |
| `sweep_init.sh` | MARS-LVIG | try multiple init methods |

**Python helper nodes** (run inside Docker): `imu_g_to_si.py` (Livox accel **g→m/s²**,
critical for MARS-LVIG), `relay_local_pos_as_baro.py` (ENU z-up → world z-down baro),
`synthetic_baro.py` (noisy GT-altitude fallback), `isaac_to_bag.py` (CSV+PNG → bag).

**`eval_isaac.py`** (run in the **`cv`** env, not Docker): Sim3-aligns
`_out/traj_est_isaac.txt` to GT (`poses.csv`), reports the fit **scale factor**
(~1.0 = correct metric scale) + rigid/Sim3 ATE, writes `_out/isaac_est_vs_gt.png`.
Use after every `run_isaac_sim.sh`. `--dir <gt_dataset>` selects the GT.

## Baro altitude updater (`UpdaterBarometer`)

In `open_vins/ov_msckf/src/update/UpdaterBarometer.{h,cpp}`. Model `y = p_z^G + noise`,
`H = [0,0,1]`. OpenVINS world-z points **DOWN** (gravity `+9.81`), so altitude →
`z_world = -altitude_rel` (the relay/converter nodes handle the negation). First
`try_update()` sets `_z_offset = z_meas - p_z` to align frames. Config (all
`required=false`): `use_baro`, `baro_noise_std` (0.3 sim / 0.5+ real),
`baro_chi2_multiplier`. **Limitation:** baro constrains `p_z` only — it cannot fix
monocular **scale**; once scale diverges the chi2 gate rejects every update.

## Datasets

**MARS-LVIG AMvalley01** (`config/mars_lvig_amvalley/`): Hikvision 2448×2048 @10 Hz
CompressedImage on `/left_camera/image`; Livox Avia IMU @200 Hz on `/livox/imu`
(**accel in g** → always `imu_g_to_si.py`); baro `/dji_osdk_ros/local_position`.
Pass `__name:=ov_msckf` to `rosrun` or topics land under `/run_subscribe_msckf/*`.
`republish compressed → image_raw` (config kalibr points at the raw topic).
`init_dyn_use: true` required (static never fires at altitude). **Result:** mono VIO
at 80–130 m nadir **fails** — scale unobservable over flat low-parallax ground; baro
can't rescue it. Dataset property, not a bug.

**TUM-VI room1** (upstream `tum_vi/`, sed → `use_stereo:false, max_cameras:1`): ATE ~5 cm.
**EuRoC MH_01_easy** (upstream `euroc_mav/`, sed → mono): ATE 33.8 cm untuned (built for
stereo); local RPE good (7.7 cm @8 m).

**Isaac Sim nadir** (`config/isaac_sim/`, latest `_in/isaac-sim-20260623`): 960×600
**12.5 Hz** pinhole (fx=fy=336.1, cx=480, cy=300, zero distortion, 110° FOV); IMU
**250 Hz** FRD, accel m/s²; baro AMSL → converter subtracts takeoff & negates.
Extrinsic is **RIGID** now (`extrinsic_is_constant: true`). GT attitude in `poses.csv`
is **FLU-in-ENU** while the extrinsic is vs the **FRD** IMU body (differ 180° about x).
Init: `init_dyn_use: true`, `init_max_disparity: 80.0`, `try_zupt: false`.

### Two independent scale paths — keep separate
1. **IMU-based scale (proper mono VIO).** Scale from the accelerometer, not depth —
   terrain irrelevant. This flight has healthy excitation (linear accel mean 1.65,
   max 9.3 m/s², 58% >1 m/s²), so scale IS observable at low altitude.
2. **Depth-based scale (baro-depth-prior, flow-odom).** Broken by terrain relief:
   Cesium camera-to-ground depth is **~6× the baro height-above-takeoff** (50–400 m
   vs 17–40 m), so the depth prior is ~6× too small. Needs a real metric AGL source.

### Isaac Sim results (Option A: lock scale from IMU, no AGL/re-record)
- **Init bug FIXED (keeper):** OV triggers *dynamic* init only when disparity
  **exceeds** `init_max_disparity`; below it, it tries *static* init, which needs an
  accel jerk the smooth climb never produces → stuck forever. Real disparity ~6–8 px
  but config had `80.0` → **lowered to 1.5**, dynamic init fires, gravity/bias/velocity
  recover. (Also disabled baro: the bag's `/baro_height` was raw AMSL ~2017 m.)
- **Scale locks <20 m then diverges:** est/GT ratio ~1.0 below 20 m → 11× at 25 m → up
  to 78× in cruise. Sim3 shape ATE ~75 m (decent), rigid ATE km — shape right, scale
  runs away as parallax vanishes over deep terrain. Deterministic (not frame drops).
- **`max_clones` 11→25 (negative):** no effect on divergence onset (~20–24 m). The
  limiter is observability, not clone-window length. Reverted.
- **OV + true-AGL depth-prior (negative, 2026-06-25):** rewrote bag so
  `/baro_height = -AGL` (`rewrite_baro_agl.py`) + config `isaac_sim_agl`. Still diverged
  (scale 0.010) — OV's prior only fires for features that FAIL triangulation; many
  still triangulate at wrong scale and pass the gate. A real fix needs a per-feature
  metric range factor (C++ change). flow-odom works because it applies AGL to *every*
  feature.
- **Working alternative (flow-odom + AGL):** on `_in/isaac-sim-20260624_2337` (1773 m)
  reaches **0.26%** (GT attitude) and **~1.5%** with NO ground truth. See `frontend/CLAUDE.md`.
- **Verdict:** the init fix proves IMU scale *can* lock (first ~20 m), but pure mono
  VIO **cannot hold scale through high-altitude deep-terrain cruise**. Baro can't help
  (z only; blowup is horizontal). Need a metric reference that survives at altitude:
  downward rangefinder / true AGL, or a low-altitude re-record.

CSV→bag: `run_isaac_to_bag.sh [run_dir] [out.bag]` → publishes
`/imu0 /cam0/image_raw /baro_height /gt_pose`.

## Config gotchas

**OpenCV YAML (`%YAML:1.0`):** files MUST start with `%YAML:1.0` on line 1 (`---`
errors); booleans MUST NOT have inline `# comments` (`true  # x` parses as `[]` →
`"invalid boolean type of []"`) — put the comment on the preceding line.

**IMU intrinsics:** OV always reads `Tw, R_IMUtoGYRO, Ta, R_IMUtoACC` (identity) and
`Tg` (zeros) from `kalibr_imu_chain.yaml` even with `calib_imu_intrinsics: false`.
Missing them → `"node Tw was not found"` → `std::exit()` crash.

**Init guide:**
| Situation | Config |
|-----------|--------|
| Handheld at rest (TUM-VI, EuRoC) | `init_dyn_use:false`, `init_imu_thresh:1.5`, `init_max_disparity:10` |
| UAV steady cruise (MARS-LVIG) | `init_dyn_use:true` (static never fires) |
| UAV rising from frame 1 (Isaac Sim) | `init_dyn_use:true`, `init_max_disparity:80→1.5`, `try_zupt:false` |

**Always repeat-run 3× before claiming an ATE improvement** — single-run ATE varies
±1.5 cm from realtime frame drops.

## Results

| Dataset | Mode | ATE | Notes |
|---------|------|-----|-------|
| TUM-VI room1 | mono KLT | ~5 cm | baseline; tuning gives no reliable gain |
| TUM-VI room1 | clones15+CLAHE | 4.6 cm | lowest variance |
| TUM-VI room2 | mono KLT | ~8.4 cm | tuning doesn't transfer |
| EuRoC MH_01 | mono KLT | 33.8 cm | built for stereo; local RPE good |
| MARS-LVIG AMvalley | mono (±baro) | km drift | scale unobservable at 80–130 m nadir |
| Isaac Sim 20260623 | mono (IMU scale) | OK <20 m, then km | init bug fixed; scale runs away with altitude |

**Bottom line:** (1) IMU-scaled mono VIO needs no depth — the live issue is init tuning
(fixed, but observability-limited at altitude). (2) Depth-scaled approaches need a
metric depth source (rangefinder / true AGL) and break over non-flat terrain. The
pipeline is correct; the limits are observability and data.
