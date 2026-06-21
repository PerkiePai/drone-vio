# AMvalley sensor calibration (for OpenVINS)

Pure visual-inertial: **camera + IMU only**. No GNSS/LiDAR fed to the estimator.

## Camera — Hikvision CA-050-11UC, global shutter, 5 mm lens
- Resolution: **2448 × 2048**, 10 Hz, message type `sensor_msgs/CompressedImage`
- Intrinsics (fx, fy, cx, cy): **1453.88, 1452.85, 1182.53, 1045.82**
- Distortion (OpenCV plumb_bob k1,k2,p1,p2,k3):
  `-0.052, 0.1168, 0.0015, 0.00013, -0.068564`
  - ⚠️ OpenVINS `radtan` supports only 4 coeffs → we use `[-0.052, 0.1168, 0.0015, 0.00013]`
    and **drop k3** (minor error at image corners; revisit by pre-undistorting if needed).

Source: UAVScenes `calibration_results.py` (per-scene MARS-LVIG cam–LiDAR calib),
cross-checked against the dataset's `CAD_extrinsic.yaml` nominal rotation.

## IMU — Livox Avia internal BMI088 @ 200 Hz
Noise params below are BMI088-datasheet values inflated ~4× for UAV vibration —
**the #1 tuning knob.** Datasheet: accel 230 µg/√Hz, gyro ~0.014°/s/√Hz.
- accelerometer_noise_density: 0.01    [m/s²/√Hz]
- accelerometer_random_walk:   0.001   [m/s³/√Hz]
- gyroscope_noise_density:     0.001   [rad/s/√Hz]
- gyroscope_random_walk:       0.0001  [rad/s²/√Hz]

## Camera ↔ IMU extrinsic
Composed `T_imu_cam = T_imu_lidar · inv(T_cam_lidar)`:
- `T_cam_lidar` (R,t) from UAVScenes AMvalley calib.
- `T_imu_lidar` = Livox Avia internal IMU↔LiDAR: R=I, t=[0.04165, 0.02326, -0.0284] m
  (standard FAST-LIO Avia extrinsic).

Result `T_imu_cam` = [R_CtoI | p_CinI] (p_CinI ≈ [0.093, 0.020, 0.028] m, ‖·‖≈0.10 m):
```
[ 0.0029810117, -0.0050461847,  0.9999806515,  0.0931578923]
[-0.9997203635,  0.0231269329,  0.0030965903,  0.0195505900]
[-0.0231465916, -0.9996994869, -0.0049755157,  0.0280173562]
[ 0.0,           0.0,           0.0,           1.0         ]
```
OpenVINS refines this online (`calib_cam_extrinsics: true`), so small error is OK.

## ROS topics — CONFIRM from `rosbag info AMvalley01.bag`
Placeholders used in config (update after inspecting the bag):
- Camera: `/left_camera/image`  (CompressedImage → must republish to raw Image)
- IMU:    `/livox/imu`
