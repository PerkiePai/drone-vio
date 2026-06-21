# Backend — VIO on MARS-LVIG with OpenVINS

True visual-inertial odometry on the MARS-LVIG aerial dataset using
[OpenVINS](https://github.com/rpng/open_vins). **Monocular, IMU + camera only —
no GNSS/GPS and no LiDAR fed to the estimator.** GNSS/RTK is kept solely as an
offline ground-truth yardstick.

## Why this setup
- MARS-LVIG bags are **ROS1** `.bag`; this box is Ubuntu 24.04 (native ROS would
  be ROS2 Jazzy). So OpenVINS runs in a **ROS1 Noetic Docker image** (`Dockerfile`)
  — CPU-only, isolated from the host. The RTX 5090 is not needed here.
- Target sequence: **AMvalley01** (forest/valley, nadir) — closest analog to the
  front-end bev-forest footage. ⚠️ 80–130 m altitude nadir footage is low-parallax,
  the hard regime for monocular VIO; expect drift / scale difficulty.

## Layout
```
backend/
  data/                            # kept at backend level (provenance docs)
    mars_lvig_drive_ids.md         # Google Drive file IDs for every sequence
    amvalley_calibration.md        # intrinsics/extrinsics/IMU-noise provenance
  openvins/                        # everything else lives here
    Dockerfile                     # OpenVINS + ROS1 Noetic + ffmpeg, builds ov_msckf/ov_eval
    inspect_bag.sh                 # rosbag info on a bag (confirm topic names)
    run_openvins.sh                # headless pipeline: decompress -> estimate -> save traj
    run_openvins_viz.sh            # same + live RViz/video/rqt_plot, records mp4 to _result/
    config/mars_lvig_amvalley/     # estimator_config + kalibr_imu/imucam chains
    open_vins/                     # cloned rpng/open_vins (gitignored)
    _out/                          # estimated trajectories, headless (gitignored)
    _result/                       # mp4 video + trajectory from viz run (gitignored)
```

## Visualize (live windows + recorded mp4)
`run_openvins_viz.sh` passes X11 through to the host desktop (DISPLAY :1) and opens:
- **RViz** — live 3D trajectory + SLAM/MSCKF feature cloud + IMU pose
- **rqt_image_view** on `/ov_msckf/trackhist` — camera frames with tracked features (the video)
- **rqt_plot** — estimated position x/y/z vs time (the realtime graph)

It records the annotated video to **`backend/openvins/_result/trackhist_amvalley01.mp4`**
and the estimated trajectory to `backend/openvins/_result/traj_est_amvalley01.txt`. Use
`PLAY_RATE=0.5 backend/openvins/run_openvins_viz.sh` if it lags realtime.

## Data
Bags live in `_in/mars-lvig/` (gitignored). Anonymous CLI download (gdown/curl)
is blocked by Google Drive's per-file quota — download via a logged-in **browser**:
- AMvalley01: https://drive.google.com/file/d/1NTecR3tb2-NYZDPH_p94bFy3lYmsQ53b/view
- save as `_in/mars-lvig/AMvalley01.bag`
- (other sequence IDs in `backend/data/mars_lvig_drive_ids.md`)

## Run
```bash
# 0. one-time: build the image (context = backend/openvins, which holds open_vins/)
docker build -t openvins:noetic -f backend/openvins/Dockerfile backend/openvins

# 1. confirm topic names (camera is CompressedImage; IMU likely /livox/imu)
backend/openvins/inspect_bag.sh
#    -> update CAM_COMPRESSED_TOPIC / IMU_TOPIC at the top of the run scripts

# 2a. headless: estimated trajectory -> backend/openvins/_out/traj_est_amvalley01.txt
backend/openvins/run_openvins.sh
# 2b. or with live viz + mp4 -> backend/openvins/_result/
backend/openvins/run_openvins_viz.sh
#    (prefix PLAY_RATE=0.5 if the estimator lags realtime)
```

## Results

### MARS-LVIG AMvalley01 (aerial nadir) — monocular VIO FAILS (expected)
Pipeline runs end-to-end (decompress → estimate → track), but the trajectory
**diverges to km scale** under every init method tried (`sweep_init.sh`). Root cause
is fundamental, not a bug:
- **IMU units:** Livox `/livox/imu` accel is in **g, not m/s²** → `imu_g_to_si.py`
  rescales to `/livox/imu_si`. (Without this, gravity leaks into velocity.)
- **Node name:** must pass `__name:=ov_msckf` or topics land under `/run_subscribe_msckf/*`.
- **Observability:** even with units fixed and dynamic init forced at the IMU-excited
  windows found by `scan_imu_excitation.py` (t≈10 s, 1093 s), the best run still drifts
  to ~28 m/s / 1.7 km. At 80–130 m nadir the scene is near-planar, low-parallax, and the
  flight is smooth → **monocular scale/gravity are not observable**. MARS-LVIG is a
  *LiDAR*-inertial dataset; pure-mono VIO is near its limit here.

### TUM-VI room1 (handheld indoor) — monocular VIO WORKS ✅
Switched to OpenVINS's native dataset to validate the pipeline (`run_tumvi.sh`,
monocular, built-in `tum_vi` config + ground truth):

| metric | value |
|---|---|
| **ATE** (abs. trajectory error, pos RMSE) | **0.068 m (6.8 cm)** |
| ATE orientation RMSE | 1.24° |
| **RPE** (relative pos error) | 2.4 cm @ 8 m → 7.5 cm @ 40 m |
| init | static, clean (velocity ≈ 0) |

Plot: `_out/tumvi_room1_trajectory.png` (estimate overlays GT). Annotated feature-track
video: `_result/tumvi_room1_tracks.mp4`. This is the working, ground-truth-evaluated
monocular VIO result; AMvalley's divergence is a property of the *data*, not the pipeline.

#### Accuracy investigation — tuning gains were within run-to-run noise
Tried 9 configs (`accuracy_sweep.sh`: more points/clones/SLAM, CLAHE, ZUPT, stereo),
then repeat-ran the top mono candidates **3× each at 0.5× playback** (`repeat_eval.sh`)
because single-run ATE is **non-deterministic** (realtime frame drops):

| config (3-run mean) | ATE pos | note |
|---|---|---|
| baseline mono | **5.1 cm** | runs: 4.3 / 6.8 / 4.3 |
| + max_clones 15 | 5.7 cm | sweep showed 4.0 cm — that was a lucky draw |
| + max_clones 20 | 5.3 cm | |
| + max_clones 15 + CLAHE | **4.6 cm** | identical 4.6 across all 3 runs (lowest variance) |

Lessons: (1) **single-run VIO ATE varies ±1.5 cm here — always repeat-run before claiming
an improvement**; the sweep's apparent "clones15 → 4.0 cm" win evaporated on repetition.
(2) ~5 cm is the achievable accuracy for monocular OpenVINS on this sequence (consistent
with published TUM-VI room results), and parameter tuning does not reliably beat it.
(3) The only robustly-better config is **`max_clones: 15` + `histogram_method: CLAHE`**
(4.6 cm, low variance) — adopt it, but the bigger takeaway is the measurement noise.
Stereo scored *worse* in the single sweep (7.1 cm) but wasn't repeat-tested, so that's
likely noise too — not a real mono-beats-stereo result.

**Generalization (room2):** confirmed the "best" config doesn't transfer. On TUM-VI
room2 (a harder sequence) baseline and `clones15+CLAHE` are identical — 8.4 vs 8.5 cm:

| config | room1 (3-run mean) | room2 (2 runs) |
|---|---|---|
| baseline mono | 5.1 cm | 8.4 / 8.4 cm |
| max_clones15 + CLAHE | 4.6 cm | 8.5 / 8.5 cm |

So the room1 tuning win was sequence-specific noise; across two sequences tuning gives
**no reliable improvement**. Final answer: monocular OpenVINS here is **~5 cm (room1) /
~8 cm (room2)** and that is the algorithm's accuracy on this data, not something config
tuning moves. (Interesting aside: room2's runs were perfectly deterministic — 8.4/8.4 —
so the realtime-frame-drop variance seen on room1 is sequence-dependent.)

### Scripts added for this investigation
- `imu_g_to_si.py` — Livox g→m/s² IMU converter (publishes `/livox/imu_si`).
- `scan_imu_excitation.py` — find well-excited windows in a bag for dynamic init.
- `sweep_init.sh` — try multiple init methods on AMvalley in one pass.
- `run_tumvi.sh` — monocular OpenVINS on TUM-VI room1 + `ov_eval` ATE/RPE.
- `plot_traj.py` — SE3-align + plot an estimate vs ground truth.
