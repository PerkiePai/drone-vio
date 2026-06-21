#!/usr/bin/env bash
# Try multiple OpenVINS init methods on AMvalley01 in one pass (IMU units fixed,
# node name fixed). Each config seeds init at a different excited window and we
# score trajectory sanity (max speed / end distance). Writes _out/sweep_<name>.txt.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SD="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$SD/_out"; mkdir -p "$OUT"
BAG="/data/mars-lvig/AMvalley01.bag"

# name | bag_start | init_max_disparity | init_imu_thresh | init_dyn_use | play_secs
CONFIGS=(
  "dyn_t10    5    8.0  0.1  true   70"
  "dyn_t66    58   8.0  0.1  true   70"
  "dyn_t1093  1085 8.0  0.1  true   70"
  "dyn_loose  5    8.0  0.03 true   70"
  "static0    0    30.0 0.5  false  70"
)

run_one() {
  local name=$1 start=$2 disp=$3 thr=$4 dyn=$5 secs=$6
  docker run --rm \
    -v "$ROOT/_in":/data -v "$SD":/work -v "$OUT":/out \
    -e CFG="$name" -e START="$start" -e DISP="$disp" -e THR="$thr" -e DYN="$dyn" -e SECS="$secs" -e BAG="$BAG" \
    openvins:noetic bash -lc '
      source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash
      cp -r /work/config/mars_lvig_amvalley /tmp/c
      sed -i "s|rostopic: /livox/imu|rostopic: /livox/imu_si|" /tmp/c/kalibr_imu_chain.yaml
      sed -i "s/init_dyn_use: .*/init_dyn_use: $DYN/" /tmp/c/estimator_config.yaml
      sed -i "s/init_imu_thresh: .*/init_imu_thresh: $THR/" /tmp/c/estimator_config.yaml
      sed -i "s/init_max_disparity: .*/init_max_disparity: $DISP/" /tmp/c/estimator_config.yaml
      roscore >/dev/null 2>&1 & sleep 3
      python3 /work/imu_g_to_si.py >/dev/null 2>&1 &
      rosrun image_transport republish compressed in:=/left_camera/image raw out:=/left_camera/image_raw >/dev/null 2>&1 &
      rosrun ov_msckf run_subscribe_msckf __name:=ov_msckf _config_path:=/tmp/c/estimator_config.yaml >/tmp/est.log 2>&1 &
      sleep 4
      rosrun ov_eval pose_to_file _topic:=/ov_msckf/poseimu _topic_type:=PoseWithCovarianceStamped _output:=/out/sweep_${CFG}.txt >/dev/null 2>&1 &
      sleep 1
      rosbag play --clock -s $START -u $SECS $BAG >/dev/null 2>&1
      sleep 2
      grep -aE "successful init|velocity =" /tmp/est.log | tail -2 | sed "s/^/    [${CFG}] /"
    ' 2>&1 | grep -avE "Bag Time|RUNNING"
}

for c in "${CONFIGS[@]}"; do
  echo ">>> running config: $c"
  run_one $c
done
echo "ALL DONE"
