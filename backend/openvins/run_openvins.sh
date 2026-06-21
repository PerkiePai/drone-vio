#!/usr/bin/env bash
# Run OpenVINS (monocular VIO, IMU+camera only) over a MARS-LVIG bag inside the
# openvins:noetic container. Decompresses the CompressedImage topic on the fly,
# subscribes the estimator, plays the bag, and saves the estimated trajectory
# to backend/openvins/_out/ in ov_eval (TUM-style) format.
#
# Usage: backend/openvins/run_openvins.sh
# Set the topic names below from `backend/openvins/inspect_bag.sh` output first.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # backend/openvins
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"                       # drone-vio root

# ---- fill these from `rosbag info` (inspect_bag.sh) ----------------------------
BAG_REL="mars-lvig/AMvalley01.bag"          # under _in/
CAM_COMPRESSED_TOPIC="/left_camera/image"   # CompressedImage topic in the bag
IMU_TOPIC="/livox/imu"                       # sensor_msgs/Imu topic in the bag
CAM_RAW_TOPIC="/left_camera/image_raw"       # republished raw topic OpenVINS reads
PLAY_RATE="${PLAY_RATE:-1.0}"                # lower (e.g. 0.5) if estimator lags
# -------------------------------------------------------------------------------

CONFIG_DIR="$SCRIPT_DIR/config/mars_lvig_amvalley"
OUT_DIR="$SCRIPT_DIR/_out"
mkdir -p "$OUT_DIR"
[ -f "$ROOT/_in/$BAG_REL" ] || { echo "Bag not found: $ROOT/_in/$BAG_REL"; exit 1; }

docker run --rm -it \
  -v "$ROOT/_in":/data \
  -v "$CONFIG_DIR":/config:ro \
  -v "$OUT_DIR":/out \
  -e CAM_COMPRESSED_TOPIC="$CAM_COMPRESSED_TOPIC" \
  -e CAM_RAW_TOPIC="$CAM_RAW_TOPIC" \
  -e IMU_TOPIC="$IMU_TOPIC" \
  -e BAG_REL="$BAG_REL" \
  -e PLAY_RATE="$PLAY_RATE" \
  openvins:noetic bash -lc '
    set -e
    source /opt/ros/noetic/setup.bash
    source /catkin_ws/devel/setup.bash

    # writable config copy with the actual bag topics injected
    cp -r /config /tmp/cfg
    sed -i "s|^\(\s*rostopic:\).*|\1 ${CAM_RAW_TOPIC}|" /tmp/cfg/kalibr_imucam_chain.yaml
    sed -i "s|^\(\s*rostopic:\).*|\1 ${IMU_TOPIC}|"     /tmp/cfg/kalibr_imu_chain.yaml

    roscore & sleep 3

    # decompress CompressedImage -> raw Image on the topic OpenVINS subscribes to
    rosrun image_transport republish compressed \
        in:=$CAM_COMPRESSED_TOPIC raw out:=$CAM_RAW_TOPIC &

    # estimator (topics come from the config rostopic fields)
    rosrun ov_msckf run_subscribe_msckf __name:=ov_msckf _config_path:=/tmp/cfg/estimator_config.yaml &
    sleep 3

    # save estimated trajectory in ov_eval format
    rosrun ov_eval pose_to_file \
        _topic:=/ov_msckf/poseimu \
        _topic_type:=PoseWithCovarianceStamped \
        _output:=/out/traj_est_amvalley01.txt &
    sleep 1

    echo ">>> playing bag at rate $PLAY_RATE ..."
    rosbag play -r $PLAY_RATE --clock /data/$BAG_REL

    sleep 3
    echo ">>> done. Estimated trajectory: backend/_out/traj_est_amvalley01.txt"
  '
