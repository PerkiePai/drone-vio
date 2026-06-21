#!/usr/bin/env bash
# Run OpenVINS on a MARS-LVIG bag WITH live visualization (X11 from the container
# onto the host desktop) + record the annotated feature-track video.
#
# Shows:
#   - RViz: live 3D trajectory + SLAM/MSCKF feature point cloud + IMU pose
#   - rqt_image_view: camera frames with tracked features overlaid (the "video")
#   - rqt_plot: live estimated position x/y/z vs time (the "realtime graph")
#   - records /ov_msckf/trackhist to backend/openvins/_result/trackhist_amvalley01.mp4
#
# Usage: backend/openvins/run_openvins_viz.sh
# Set the bag topic names below from `backend/openvins/inspect_bag.sh` first.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # backend/openvins
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"                       # drone-vio root

# ---- fill from `rosbag info` (inspect_bag.sh) ---------------------------------
BAG_REL="mars-lvig/AMvalley01.bag"
CAM_COMPRESSED_TOPIC="/left_camera/image"
IMU_TOPIC="/livox/imu"
CAM_RAW_TOPIC="/left_camera/image_raw"
PLAY_RATE="${PLAY_RATE:-1.0}"     # lower if estimator/viz lag (e.g. 0.5)
HOST_DISPLAY="${HOST_DISPLAY:-:1}" # desktop display (Firefox is on :1 here)
# ------------------------------------------------------------------------------

CONFIG_DIR="$SCRIPT_DIR/config/mars_lvig_amvalley"
OUT_DIR="$SCRIPT_DIR/_result"
mkdir -p "$OUT_DIR"
[ -f "$ROOT/_in/$BAG_REL" ] || { echo "Bag not found: $ROOT/_in/$BAG_REL"; exit 1; }

# allow the container (root) to draw on the host X server
xhost +local:root >/dev/null 2>&1 || true
trap 'xhost -local:root >/dev/null 2>&1 || true' EXIT

docker run --rm -it \
  --net=host \
  -e DISPLAY="$HOST_DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$ROOT/_in":/data \
  -v "$CONFIG_DIR":/config:ro \
  -v "$OUT_DIR":/result \
  -e CAM_COMPRESSED_TOPIC="$CAM_COMPRESSED_TOPIC" \
  -e CAM_RAW_TOPIC="$CAM_RAW_TOPIC" \
  -e IMU_TOPIC="$IMU_TOPIC" \
  -e BAG_REL="$BAG_REL" \
  -e PLAY_RATE="$PLAY_RATE" \
  openvins:noetic bash -lc '
    set -e
    source /opt/ros/noetic/setup.bash
    source /catkin_ws/devel/setup.bash
    RVIZ_CFG=/catkin_ws/src/open_vins/ov_msckf/launch/display.rviz

    cp -r /config /tmp/cfg
    sed -i "s|^\(\s*rostopic:\).*|\1 ${CAM_RAW_TOPIC}|" /tmp/cfg/kalibr_imucam_chain.yaml
    sed -i "s|^\(\s*rostopic:\).*|\1 ${IMU_TOPIC}|"     /tmp/cfg/kalibr_imu_chain.yaml

    roscore & sleep 3

    rosrun image_transport republish compressed \
        in:=$CAM_COMPRESSED_TOPIC raw out:=$CAM_RAW_TOPIC &

    rosrun ov_msckf run_subscribe_msckf __name:=ov_msckf _config_path:=/tmp/cfg/estimator_config.yaml &
    sleep 4

    # ---- visualization ----
    rviz -d $RVIZ_CFG &
    rqt_image_view /ov_msckf/trackhist &
    rqt_plot /ov_msckf/odomimu/pose/pose/position/x \
             /ov_msckf/odomimu/pose/pose/position/y \
             /ov_msckf/odomimu/pose/pose/position/z &
    # record the annotated video (mp4) + the trajectory text into backend/_result
    rosrun image_view video_recorder image:=/ov_msckf/trackhist \
        _filename:=/result/trackhist_amvalley01.mp4 _fps:=10 _codec:=mp4v &
    rosrun ov_eval pose_to_file _topic:=/ov_msckf/poseimu \
        _topic_type:=PoseWithCovarianceStamped \
        _output:=/result/traj_est_amvalley01.txt &
    sleep 2

    echo ">>> playing bag at rate $PLAY_RATE (close windows / Ctrl-C to stop) ..."
    rosbag play -r $PLAY_RATE --clock /data/$BAG_REL

    sleep 2
    echo ">>> done. Video: backend/_result/trackhist_amvalley01.mp4  Traj: backend/_result/traj_est_amvalley01.txt"
  '
