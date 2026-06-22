# CLAUDE.md — backend/

VIO backend: OpenVINS (monocular IMU+camera, no GNSS) in a ROS1 Noetic Docker
image, running on MARS-LVIG aerial bags, TUM-VI, EuRoC, and Isaac Sim data.

---

## Architecture at a glance

```
backend/
  CLAUDE.md               ← this file
  data/                   # provenance docs only (no binaries)
    mars_lvig_drive_ids.md
    amvalley_calibration.md
  openvins/
    Dockerfile            # osrf/ros:noetic-desktop-full + Ceres + catkin build
    open_vins/            # cloned rpng/open_vins (gitignored)
    config/
      mars_lvig_amvalley/ # MARS-LVIG AMvalley01 (nadir UAV, 2448×2048)
      isaac_sim/          # Isaac Sim nadir drone dataset (960×600)
    run_*.sh              # one script per dataset or task
    *.py                  # ROS helper nodes (relay, rescale, convert)
    _out/                 # trajectories + logs (gitignored)
    _result/              # videos + results (gitignored)
```

All runs happen inside `docker run --rm openvins:noetic`. The host mounts bags,
configs, and scripts read-only; only `_out/` and `_result/` are writable.

---

## Docker image

```bash
# Build (takes ~10 min — compiles ov_msckf + ov_eval from source)
docker build -t openvins:noetic -f backend/openvins/Dockerfile backend/openvins

# Always rebuild when C++ source in open_vins/ changes
# Config-only changes don't need a rebuild (mounted at runtime)
```

### Known Dockerfile gotcha — image_transport v1.15.0 (2025-05-21)

`ros-noetic-compressed-*-image-transport` v1.15 has a class_loader bug: when
the binary exits via `std::exit()`, static destructors try to unload a plugin
that Poco loaded outside class_loader's tracking → `LibraryUnloadException` →
`terminate()`. Fixed in the Dockerfile with two changes:

1. **Remove `compressed-depth-image-transport`** (we never use depth images):
   ```dockerfile
   apt-get remove -y ros-noetic-compressed-depth-image-transport
   ```
2. **Replace `image_transport::Publisher` with `ros::Publisher`** in
   `open_vins/ov_msckf/src/ros/ROS1Visualizer.h/.cpp` — the ImageTransport
   constructor itself triggered the broken load/unload cycle.

A third trigger: our new YAML params (`use_baro`, etc.) default `required=true`
in `parse_config()`. Missing keys cause `parser->successful()` → false →
`std::exit()` → same crash. Fixed with `parse_config("use_baro", use_baro, false)`.

**Rule:** Any new optional OpenVINS YAML param MUST use `required=false` as the
third argument to `parse_config()`.

---

## Run scripts — what each does

| Script | Dataset | Notes |
|--------|---------|-------|
| `run_tumvi.sh` | TUM-VI room1 | mono, ATE ~5 cm baseline |
| `run_openvins.sh` | MARS-LVIG AMvalley01 | headless, no baro |
| `run_openvins_viz.sh` | MARS-LVIG AMvalley01 | X11 RViz + mp4 |
| `run_marslvig_baro.sh` | MARS-LVIG AMvalley01 | real baro from `/dji_osdk_ros/local_position` |
| `run_euroc.sh` | EuRoC MH_01_easy | patches stereo→mono; ATE 33.8 cm untuned |
| `run_isaac_to_bag.sh` | Isaac Sim CSV dataset | converts CSV+PNG → ROS bag |
| `run_isaac_sim.sh` | Isaac Sim bag | mono + baro; needs bag from `run_isaac_to_bag.sh` |
| `run_tumvi.sh` | TUM-VI room1 | downloads bag needed separately |
| `inspect_bag.sh` | any bag | `rosbag info` inside Docker |
| `accuracy_sweep.sh` | TUM-VI room1 | 9-config ATE sweep |
| `repeat_eval.sh` | TUM-VI room1 | repeat-run for statistical ATE |
| `sweep_init.sh` | MARS-LVIG AMvalley01 | try multiple init methods |

### Python helper nodes (run inside Docker)

| Script | Purpose |
|--------|---------|
| `imu_g_to_si.py` | Rescales Livox BMI088 accel from **g → m/s²** (critical for MARS-LVIG) |
| `relay_local_pos_as_baro.py` | Relays `/dji_osdk_ros/local_position` (ENU z-up) → `/baro_height` (world z-down for OV) |
| `synthetic_baro.py` | Adds Gaussian noise to NavSatFix GT altitude (fallback for datasets without real baro) |
| `isaac_to_bag.py` | Converts Isaac Sim CSV+PNG dataset to ROS1 bag |

---

## Baro altitude updater (`UpdaterBarometer`)

Added to `open_vins/ov_msckf/src/update/UpdaterBarometer.{h,cpp}`.

**Measurement model:** `y = p_z^G + noise`, `H = [0,0,1]` w.r.t. `state->_imu->p()`.

**World frame:** OpenVINS world-z points **DOWN** (gravity = [0,0,+9.81]).
More altitude = more negative `p_z^G`. So:
- ENU z-up altitude → `z_world = -altitude_rel`
- `relay_local_pos_as_baro.py` handles this negation for MARS-LVIG.
- `isaac_to_bag.py` handles it for Isaac Sim (subtracts takeoff AMSL then negates).

**Online z-offset:** First `try_update()` call sets `_z_offset = z_meas - p_z`.
This aligns baro frame to VIO world frame regardless of where VIO initialized.

**Config keys** (all `required=false` → safe to omit from any existing config):
```yaml
use_baro: false           # default off; set true in run_marslvig_baro.sh via sed
baro_noise_std: 0.5       # 1-sigma metres; 0.3 for clean sim, 0.5+ for real sensor
baro_chi2_multiplier: 1.0 # chi2 gate; 1.0 = tight; increase if filter diverges first
```

**Limitation:** Baro constrains `p_z` but cannot fix monocular **scale**. If scale
diverges (which it does at 80–130 m nadir for MARS-LVIG), the baro chi2 gate
rejects all updates because the residual grows into the hundreds of metres. The
baro works only when the VIO scale is approximately correct at init.

---

## Dataset details and configs

### MARS-LVIG AMvalley01 — `config/mars_lvig_amvalley/`

- **Camera:** Hikvision CA-050-11UC, 2448×2048, 10 Hz, CompressedImage on `/left_camera/image`
- **IMU:** Livox Avia BMI088 @ 200 Hz on `/livox/imu` — **accel in g, not m/s²** → always run `imu_g_to_si.py`
- **Baro:** `/dji_osdk_ros/local_position` (PointStamped, ENU z-up, 50 Hz, height above home point)
- **Node name:** must pass `__name:=ov_msckf` to `rosrun ov_msckf run_subscribe_msckf`, otherwise topics go under `/run_subscribe_msckf/*`
- **Camera topic:** `image_transport republish compressed in:=/left_camera/image raw out:=/left_camera/image_raw`
  The config kalibr points to the raw topic; the `republish` node creates it from the compressed source.
- **Init:** `init_dyn_use: true` is required — static init never converges at altitude because the drone alternates between maneuvering (disparity > 30px → rejected) and smooth cruise (IMU excitation < 1.5 → rejected).

**Result:** monocular VIO at 80–130 m nadir **fails** — scale unobservable (flat ground, low parallax). Even with baro enabled, scale diverges to km before baro offsets can correct it. This is a dataset property, not a pipeline bug.

### TUM-VI room1 — uses upstream config `tum_vi/`

- Patched to monocular via sed: `use_stereo: false, max_cameras: 1`
- ATE ~5 cm baseline; see README.md for full sweep results.

### EuRoC MH_01_easy — uses upstream config `euroc_mav/`

- Patched to monocular via sed: `use_stereo: false, max_cameras: 1`
- **ATE: 33.8 cm RMSE position** (untuned; EuRoC was designed for stereo)
- RPE @ 8 m: 7.7 cm median (1% relative error — local tracking is good)
- Bag: `_in/euroc/MH_01_easy.bag` (from Kaggle `chunai/euroc-mh-01-easy-ros-bag-dataset`)

### Isaac Sim nadir drone dataset — `config/isaac_sim/`

- **Camera:** 960×600, 10 Hz, pinhole fx=fy=336.1, cx=480, cy=300, zero distortion, mono8
- **IMU:** 50 Hz FRD body, specific force (az ≈ -9.81 at hover), accel in m/s²
- **Baro:** AMSL pressure altitude → converter subtracts takeoff alt and negates
- **Extrinsic:** GIMBAL-STABILISED (`extrinsic_is_constant: false`) — T_imu_cam in config is the takeoff snapshot only. **This is a hard blocker for classical VIO**: body yaw/pitch changes the real extrinsic but OpenVINS assumes it's fixed → feature tracks become inconsistent → filter diverges in seconds.
- **Init:** `init_dyn_use: true`, `init_max_disparity: 80.0`, `try_zupt: false` — drone rises at ~83 cm/s from frame 1, creating ~68 px/frame feature motion at 0.5 m altitude; static init never fires.
- **Result:** filter diverges to km-scale within 10 seconds due to gimbal extrinsic violation. **Re-record with `DOWN_STABILIZE=False`** for a proper rigid extrinsic before using this dataset.

**CSV→bag converter:**
```bash
backend/openvins/run_isaac_to_bag.sh [run_dir] [out.bag]
# Publishes: /imu0  /cam0/image_raw  /baro_height  /gt_pose
# Default run_dir: /home/innovation/vio_dataset/dataset(more function but more gb)/
```

---

## OpenCV YAML gotcha

All config files use `%YAML:1.0` (OpenCV's YAML variant). Two rules:

1. **Files MUST start with `%YAML:1.0`** on line 1. `---` causes `cv::Exception: Input file is invalid`.
2. **Boolean values MUST NOT have inline `# comments`**:
   ```yaml
   # WRONG — OpenCV parses 'true  # comment' as an empty sequence []
   init_dyn_use: true           # dynamic init
   
   # CORRECT — put comment on the preceding line
   # dynamic init: works during motion
   init_dyn_use: true
   ```
   This causes `"invalid boolean type of []"` and fails `parser->successful()`.

---

## IMU intrinsic matrices (always required in kalibr_imu_chain.yaml)

Even with `calib_imu_intrinsics: false`, OpenVINS always reads these from the kalibr IMU file.
Use identity matrices for a well-calibrated or simulated IMU:

```yaml
imu0:
  # ... noise params ...
  Tw:
    - [ 1.0, 0.0, 0.0 ]
    - [ 0.0, 1.0, 0.0 ]
    - [ 0.0, 0.0, 1.0 ]
  R_IMUtoGYRO:
    - [ 1.0, 0.0, 0.0 ]
    - [ 0.0, 1.0, 0.0 ]
    - [ 0.0, 0.0, 1.0 ]
  Ta:
    - [ 1.0, 0.0, 0.0 ]
    - [ 0.0, 1.0, 0.0 ]
    - [ 0.0, 0.0, 1.0 ]
  R_IMUtoACC:
    - [ 1.0, 0.0, 0.0 ]
    - [ 0.0, 1.0, 0.0 ]
    - [ 0.0, 0.0, 1.0 ]
  Tg:
    - [ 0.0, 0.0, 0.0 ]
    - [ 0.0, 0.0, 0.0 ]
    - [ 0.0, 0.0, 0.0 ]
```

Missing these causes `"the node Tw ... was not found"` → `parser->successful()` false →
`std::exit()` → class_loader crash at shutdown.

---

## Initialization guide

| Situation | Config |
|-----------|--------|
| Handheld, starts at rest (TUM-VI, EuRoC) | `init_dyn_use: false`, `init_imu_thresh: 1.5`, `init_max_disparity: 10` |
| UAV at altitude, steady cruise (MARS-LVIG) | `init_dyn_use: true` — static never fires (disparity too high when maneuvering, excitation too low when cruising) |
| UAV rising from frame 1 (Isaac Sim) | `init_dyn_use: true`, `init_max_disparity: 80.0`, `try_zupt: false` |

**Always repeat-run 3× before claiming an ATE improvement** — single-run VIO ATE
varies ±1.5 cm on TUM-VI room1 from realtime frame drops (non-deterministic).

---

## Results summary

| Dataset | Mode | ATE pos | Notes |
|---------|------|---------|-------|
| TUM-VI room1 | mono KLT | ~5 cm | baseline; parameter tuning gives no reliable gain |
| TUM-VI room1 | mono KLT, clones15+CLAHE | 4.6 cm | lowest variance (3 runs identical) |
| TUM-VI room2 | mono KLT, any config | ~8.4 cm | tuning doesn't transfer across sequences |
| EuRoC MH_01_easy | mono KLT (untuned) | 33.8 cm | designed for stereo; local RPE is good (7.7 cm@8m) |
| MARS-LVIG AMvalley | mono KLT | km drift | scale unobservable at 80–130 m nadir, flat ground |
| MARS-LVIG AMvalley + baro | mono KLT + baro | km drift | baro chi2 gate rejects updates once scale diverges |
| Isaac Sim | mono KLT + baro | km drift in <10s | gimbal extrinsic violates rigid-body assumption |

**Bottom line:** monocular nadir VIO at altitude (>30 m, flat ground) requires
either stereo, LiDAR depth, or a re-recorded dataset with a rigid extrinsic.
The pipeline itself is correct — the limitation is the observability of the data.
