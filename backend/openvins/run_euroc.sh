#!/usr/bin/env bash
# Run OpenVINS (MONOCULAR) on EuRoC MH_01_easy and evaluate ATE/RPE.
# Tests whether the pipeline generalises from TUM-VI (handheld) to real drone dynamics.
#
# Download the bag first:
#   https://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag
# Save as  _in/euroc/MH_01_easy.bag
#
# Expected result: ~10–15 cm ATE monocular (published EuRoC benchmarks).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SD="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$SD/_out"; mkdir -p "$OUT"

SEQ="${1:-MH_01_easy}"
BAG_REL="euroc/${SEQ}.bag"

[ -f "$ROOT/_in/$BAG_REL" ] || {
  echo "Bag not found: $ROOT/_in/$BAG_REL"
  echo "Download from:"
  echo "  https://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/${SEQ}/${SEQ}.bag"
  exit 1
}

docker run --rm -v "$ROOT/_in":/data -v "$OUT":/out openvins:noetic bash -lc "
  set -e
  source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash
  SRC=/catkin_ws/src/open_vins

  # Copy config and override to monocular (ships stereo)
  cp -r \$SRC/config/euroc_mav /tmp/ecfg
  sed -i 's/^use_stereo:.*/use_stereo: false/; s/^max_cameras:.*/max_cameras: 1/' /tmp/ecfg/estimator_config.yaml

  roscore >/dev/null 2>&1 & sleep 3

  rosrun ov_msckf run_subscribe_msckf __name:=ov_msckf \
    _config_path:=/tmp/ecfg/estimator_config.yaml >/tmp/est.log 2>&1 &
  sleep 3

  rosrun ov_eval pose_to_file \
    _topic:=/ov_msckf/poseimu \
    _topic_type:=PoseWithCovarianceStamped \
    _output:=/out/euroc_${SEQ}_est.txt >/dev/null 2>&1 &
  sleep 1

  echo '>>> playing EuRoC $SEQ ...'
  rosbag play --clock /data/$BAG_REL >/dev/null 2>&1
  sleep 3

  echo '=== init / status ==='
  grep -aE 'successful init|velocity =' /tmp/est.log | tail -3

  echo '=== ATE/RPE vs ground truth (se3 align) ==='
  cp \$SRC/ov_data/euroc_mav/${SEQ}.txt /out/euroc_${SEQ}_gt.txt
  rosrun ov_eval error_singlerun se3 \
    /out/euroc_${SEQ}_gt.txt /out/euroc_${SEQ}_est.txt 2>&1 \
    | grep -aiE 'rmse|error|ate|rpe|traj|mean|median' | head -25
"
echo ">>> est: backend/openvins/_out/euroc_${SEQ}_est.txt   gt: ..._gt.txt"
