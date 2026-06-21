#!/usr/bin/env bash
# Run OpenVINS (MONOCULAR) on TUM-VI room1 using OpenVINS's built-in tum_vi
# config + ground truth, then evaluate ATE/RPE. This is the "known-good" dataset
# fallback after MARS-LVIG monocular VIO failed (low-parallax aerial nadir).
# Images are raw (no decompression); platform starts static so static init works.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SD="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$SD/_out"; mkdir -p "$OUT"
BAG_REL="tumvi/dataset-room1_512_16.bag"
SEQ="dataset-room1_512_16"
[ -f "$ROOT/_in/$BAG_REL" ] || { echo "Bag not found: $ROOT/_in/$BAG_REL"; exit 1; }

docker run --rm -v "$ROOT/_in":/data -v "$OUT":/out openvins:noetic bash -lc "
  set -e
  source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash
  SRC=/catkin_ws/src/open_vins
  cp -r \$SRC/config/tum_vi /tmp/tcfg
  # monocular override (config ships stereo)
  sed -i 's/^use_stereo:.*/use_stereo: false/; s/^max_cameras:.*/max_cameras: 1/' /tmp/tcfg/estimator_config.yaml

  roscore >/dev/null 2>&1 & sleep 3
  rosrun ov_msckf run_subscribe_msckf __name:=ov_msckf _config_path:=/tmp/tcfg/estimator_config.yaml >/tmp/est.log 2>&1 &
  sleep 3
  rosrun ov_eval pose_to_file _topic:=/ov_msckf/poseimu _topic_type:=PoseWithCovarianceStamped _output:=/out/tumvi_${SEQ}_est.txt >/dev/null 2>&1 &
  sleep 1
  echo '>>> playing TUM-VI room1 ...'
  rosbag play --clock /data/$BAG_REL >/dev/null 2>&1
  sleep 3

  echo '=== init / status ==='; grep -aE 'successful init|velocity =' /tmp/est.log | tail -2
  echo '=== ATE/RPE vs ground truth (se3 align) ==='
  cp \$SRC/ov_data/tum_vi/${SEQ}.txt /out/tumvi_${SEQ}_gt.txt
  rosrun ov_eval error_singlerun se3 /out/tumvi_${SEQ}_gt.txt /out/tumvi_${SEQ}_est.txt 2>&1 | grep -aiE 'rmse|error|ate|rpe|traj|mean|median' | head -25
"
echo ">>> est: backend/openvins/_out/tumvi_${SEQ}_est.txt   gt: ..._gt.txt"
