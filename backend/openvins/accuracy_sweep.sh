#!/usr/bin/env bash
# Systematically improve OpenVINS accuracy on TUM-VI room1: try many configs,
# report ATE (pos RMSE) + RPE for each vs ground truth. Baseline mono = 6.8 cm.
# Each config runs the full bag + ov_eval error_singlerun. ~2.5 min/config.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_out"; mkdir -p "$OUT"
SEQ="dataset-room1_512_16"

# name | sed-overrides (semicolon-separated; applied to estimator_config.yaml)
CONFIGS=(
  "mono_base|s/^use_stereo:.*/use_stereo: false/; s/^max_cameras:.*/max_cameras: 1/"
  "mono_pts400|s/^use_stereo:.*/use_stereo: false/; s/^max_cameras:.*/max_cameras: 1/; s/^num_pts:.*/num_pts: 400/"
  "mono_clones15|s/^use_stereo:.*/use_stereo: false/; s/^max_cameras:.*/max_cameras: 1/; s/^max_clones:.*/max_clones: 15/"
  "mono_slam100|s/^use_stereo:.*/use_stereo: false/; s/^max_cameras:.*/max_cameras: 1/; s/^max_slam:.*/max_slam: 100/; s/^max_slam_in_update:.*/max_slam_in_update: 50/"
  "mono_tuned|s/^use_stereo:.*/use_stereo: false/; s/^max_cameras:.*/max_cameras: 1/; s/^num_pts:.*/num_pts: 400/; s/^max_clones:.*/max_clones: 15/; s/^max_slam:.*/max_slam: 100/; s/^max_slam_in_update:.*/max_slam_in_update: 50/; s/^max_msckf_in_update:.*/max_msckf_in_update: 60/"
  "mono_tuned_clahe|s/^use_stereo:.*/use_stereo: false/; s/^max_cameras:.*/max_cameras: 1/; s/^num_pts:.*/num_pts: 400/; s/^max_clones:.*/max_clones: 15/; s/^max_slam:.*/max_slam: 100/; s/^max_slam_in_update:.*/max_slam_in_update: 50/; s/^histogram_method:.*/histogram_method: \"CLAHE\"/"
  "mono_tuned_zupt|s/^use_stereo:.*/use_stereo: false/; s/^max_cameras:.*/max_cameras: 1/; s/^num_pts:.*/num_pts: 400/; s/^max_clones:.*/max_clones: 15/; s/^max_slam:.*/max_slam: 100/; s/^try_zupt:.*/try_zupt: true/"
  "stereo_base|"
  "stereo_tuned|s/^num_pts:.*/num_pts: 300/; s/^max_clones:.*/max_clones: 15/; s/^max_slam:.*/max_slam: 100/; s/^max_slam_in_update:.*/max_slam_in_update: 50/"
)

printf "%-18s %10s %10s %10s\n" "config" "ATE_pos_cm" "ATE_ori_deg" "RPE8m_cm"
for entry in "${CONFIGS[@]}"; do
  name="${entry%%|*}"; seds="${entry#*|}"
  res=$(docker run --rm -v "$ROOT/_in":/data -v "$OUT":/out \
    -e SEDS="$seds" -e SEQ="$SEQ" -e NAME="$name" \
    openvins:noetic bash -lc '
      source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash
      SRC=/catkin_ws/src/open_vins
      cp -r $SRC/config/tum_vi /tmp/c
      [ -n "$SEDS" ] && sed -i "$SEDS" /tmp/c/estimator_config.yaml
      roscore >/dev/null 2>&1 & sleep 3
      rosrun ov_msckf run_subscribe_msckf __name:=ov_msckf _config_path:=/tmp/c/estimator_config.yaml >/dev/null 2>&1 &
      sleep 3
      rosrun ov_eval pose_to_file _topic:=/ov_msckf/poseimu _topic_type:=PoseWithCovarianceStamped _output:=/out/acc_${NAME}.txt >/dev/null 2>&1 &
      sleep 1
      rosbag play --clock /data/tumvi/${SEQ}.bag >/dev/null 2>&1
      sleep 2
      cp $SRC/ov_data/tum_vi/${SEQ}.txt /out/_gt.txt
      rosrun ov_eval error_singlerun se3 /out/_gt.txt /out/acc_${NAME}.txt 2>&1 | grep -aE "rmse_pos|seg 8 -"
    ' 2>&1 | grep -avE "Bag Time|RUNNING")
  ate_pos=$(echo "$res" | grep -aoE "rmse_pos = [0-9.]+" | grep -aoE "[0-9.]+" | head -1)
  ate_ori=$(echo "$res" | grep -aoE "rmse_ori = [0-9.]+" | grep -aoE "[0-9.]+" | head -1)
  rpe8=$(echo "$res" | grep -a "seg 8 -" | grep -aoE "median_pos = [0-9.]+" | grep -aoE "[0-9.]+" | head -1)
  printf "%-18s %10s %10s %10s\n" "$name" \
    "$(awk "BEGIN{printf \"%.1f\", ${ate_pos:-0}*100}")" "${ate_ori:-?}" \
    "$(awk "BEGIN{printf \"%.1f\", ${rpe8:-0}*100}")"
done
echo "SWEEP DONE"
