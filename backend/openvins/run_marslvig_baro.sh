#!/usr/bin/env bash
# Run MARS-LVIG AMvalley01 with barometer-aided OpenVINS.
#
# Uses /dji_osdk_ros/local_position (PointStamped, 50 Hz, ENU z-up height above
# takeoff) relayed to /baro_height via relay_local_pos_as_baro.py.
# This is REAL sensor data from the DJI SDK, not synthetic/simulated.
#
# The baro altitude update constrains the z-axis, which is the unobservable
# dimension for monocular VIO at altitude (no parallax on flat ground).
#
# PREREQUISITE: Docker must be rebuilt with the baro updater:
#   docker build -t openvins:noetic -f backend/openvins/Dockerfile backend/openvins

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

BAG_REL="mars-lvig/AMvalley01.bag"
CAM_COMPRESSED_TOPIC="/left_camera/image"
IMU_TOPIC="/livox/imu"
CAM_RAW_TOPIC="/left_camera/image_raw"
LOCAL_POS_TOPIC="/dji_osdk_ros/local_position"   # PointStamped, ENU z-up, 50 Hz
PLAY_RATE="${PLAY_RATE:-0.5}"

CONFIG_DIR="$SCRIPT_DIR/config/mars_lvig_amvalley"
OUT_DIR="$SCRIPT_DIR/_out"; mkdir -p "$OUT_DIR"
[ -f "$ROOT/_in/$BAG_REL" ] || { echo "Bag not found: $ROOT/_in/$BAG_REL"; exit 1; }

docker run --rm \
  -v "$ROOT/_in":/data \
  -v "$CONFIG_DIR":/config:ro \
  -v "$SCRIPT_DIR":/scripts:ro \
  -v "$OUT_DIR":/out \
  -e CAM_COMPRESSED_TOPIC="$CAM_COMPRESSED_TOPIC" \
  -e CAM_RAW_TOPIC="$CAM_RAW_TOPIC" \
  -e IMU_TOPIC="$IMU_TOPIC" \
  -e LOCAL_POS_TOPIC="$LOCAL_POS_TOPIC" \
  -e BAG_REL="$BAG_REL" \
  -e PLAY_RATE="$PLAY_RATE" \
  openvins:noetic bash -lc '
    set -e
    source /opt/ros/noetic/setup.bash
    source /catkin_ws/devel/setup.bash

    # Writable config copy with baro enabled
    cp -r /config /tmp/cfg
    sed -i "s|^\(\s*rostopic:\).*|\1 ${CAM_RAW_TOPIC}|" /tmp/cfg/kalibr_imucam_chain.yaml
    sed -i "s|^\(\s*rostopic:\).*|\1 ${IMU_TOPIC}|"     /tmp/cfg/kalibr_imu_chain.yaml
    sed -i "s/^use_baro:.*/use_baro: true/" /tmp/cfg/estimator_config.yaml

    roscore >/dev/null 2>&1 & sleep 3

    # Decompress CompressedImage to raw Image
    rosrun image_transport republish compressed \
        in:=${CAM_COMPRESSED_TOPIC} raw out:=${CAM_RAW_TOPIC} &

    # Rescale Livox IMU accel from g → m/s²
    python3 /scripts/imu_g_to_si.py &
    sleep 2

    # Relay DJI local_position as baro (negate z: ENU up → world down)
    python3 /scripts/relay_local_pos_as_baro.py \
        _src_topic:=${LOCAL_POS_TOPIC} &
    sleep 1

    # Estimator with baro enabled
    rosrun ov_msckf run_subscribe_msckf __name:=ov_msckf \
        _config_path:=/tmp/cfg/estimator_config.yaml \
        _topic_baro:=/baro_height >/tmp/est.log 2>&1 &
    sleep 3

    # Save estimated trajectory
    rosrun ov_eval pose_to_file \
        _topic:=/ov_msckf/poseimu \
        _topic_type:=PoseWithCovarianceStamped \
        _output:=/out/traj_est_amvalley01_baro.txt >/dev/null 2>&1 &
    sleep 1

    echo ">>> playing bag at rate $PLAY_RATE  (20 min at 0.5x = ~40 min wall time) ..."
    rosbag play -r $PLAY_RATE --clock /data/$BAG_REL >/dev/null 2>&1

    sleep 5
    echo "=== init / status ==="
    grep -aE "successful init|baro|BARO|velocity =" /tmp/est.log | tail -5
    echo ">>> done. Trajectory saved to backend/openvins/_out/traj_est_amvalley01_baro.txt"
  '
