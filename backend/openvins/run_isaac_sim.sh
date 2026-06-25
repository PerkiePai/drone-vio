#!/usr/bin/env bash
# Run OpenVINS on a converted Isaac Sim bag.
#
# Usage:
#   backend/openvins/run_isaac_sim.sh [bag_path] [play_rate]
#
# Defaults:
#   bag_path  = /home/innovation/vio_dataset/dataset(more function but more gb)/isaac_sim.bag
#   play_rate = 1.0  (sim bag has no realtime dependency, can run at 1x)
#
# Convert CSV→bag first if not done:
#   backend/openvins/run_isaac_to_bag.sh

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

BAG_PATH="${1:-$SCRIPT_DIR/../../_in/isaac-sim-20260624/isaac_sim.bag}"
PLAY_RATE="${2:-1.0}"
CONFIG_SUB="${3:-isaac_sim}"          # config subdir under config/ (e.g. isaac_sim_agl)

CONFIG_DIR="$SCRIPT_DIR/config/$CONFIG_SUB"
OUT_DIR="$SCRIPT_DIR/_out"; mkdir -p "$OUT_DIR"

[ -f "$BAG_PATH" ] || { echo "ERROR: bag not found: $BAG_PATH"; echo "Run: backend/openvins/run_isaac_to_bag.sh first"; exit 1; }
[ -d "$CONFIG_DIR" ] || { echo "ERROR: config dir not found: $CONFIG_DIR"; exit 1; }

BAG_DIR="$(dirname "$BAG_PATH")"
BAG_FILE="$(basename "$BAG_PATH")"

echo ">>> OpenVINS on Isaac Sim bag"
echo "    Bag:    $BAG_PATH"
echo "    Rate:   ${PLAY_RATE}x"

docker run --rm \
  -v "$BAG_DIR":/bag_dir:ro \
  -v "$CONFIG_DIR":/config:ro \
  -v "$OUT_DIR":/out \
  openvins:noetic bash -lc "
    set -e
    source /opt/ros/noetic/setup.bash
    source /catkin_ws/devel/setup.bash

    cp -r /config /tmp/cfg

    roscore >/dev/null 2>&1 & sleep 3

    rosrun ov_msckf run_subscribe_msckf __name:=ov_msckf \
        _config_path:=/tmp/cfg/estimator_config.yaml \
        _topic_imu:=/imu0 \
        _topic_camera0:=/cam0/image_raw \
        _topic_baro:=/baro_height >/tmp/est.log 2>&1 &
    sleep 3

    # Save trajectory for ov_eval
    rosrun ov_eval pose_to_file \
        _topic:=/ov_msckf/poseimu \
        _topic_type:=PoseWithCovarianceStamped \
        _output:=/out/traj_est_isaac.txt >/dev/null 2>&1 &
    sleep 1

    echo '>>> Playing bag at rate $PLAY_RATE ...'
    rosbag play -r $PLAY_RATE --clock /bag_dir/$BAG_FILE >/dev/null 2>&1

    sleep 5
    cp /tmp/est.log /out/est_isaac.log || true
    echo '=== init / status ==='
    grep -aE 'successful init|baro|BARO|velocity =' /tmp/est.log | tail -8

    echo '>>> done. Trajectory: backend/openvins/_out/traj_est_isaac.txt  Log: backend/openvins/_out/est_isaac.log'
  "
