#!/usr/bin/env bash
# Repeatability check: single-run ATE on TUM-VI has variance (realtime playback
# drops frames non-deterministically). Run each candidate config N times and report
# per-run + mean ATE, so an "improvement" is real and not noise. Also plays the bag
# at --rate 0.5 to reduce frame drops (more deterministic than realtime).
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_out"; mkdir -p "$OUT"
SEQ="dataset-room1_512_16"
N=3

CONFIGS=(
  "base|s/^use_stereo:.*/use_stereo: false/; s/^max_cameras:.*/max_cameras: 1/"
  "clones15|s/^use_stereo:.*/use_stereo: false/; s/^max_cameras:.*/max_cameras: 1/; s/^max_clones:.*/max_clones: 15/"
  "clones20|s/^use_stereo:.*/use_stereo: false/; s/^max_cameras:.*/max_cameras: 1/; s/^max_clones:.*/max_clones: 20/"
  "clones15_clahe|s/^use_stereo:.*/use_stereo: false/; s/^max_cameras:.*/max_cameras: 1/; s/^max_clones:.*/max_clones: 15/; s/^histogram_method:.*/histogram_method: \"CLAHE\"/"
)

run_once() {  # $1=seds  -> prints ATE_pos in cm
  docker run --rm -v "$ROOT/_in":/data -v "$OUT":/out -e SEDS="$1" -e SEQ="$SEQ" openvins:noetic bash -lc '
    source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash
    SRC=/catkin_ws/src/open_vins; cp -r $SRC/config/tum_vi /tmp/c
    [ -n "$SEDS" ] && sed -i "$SEDS" /tmp/c/estimator_config.yaml
    roscore >/dev/null 2>&1 & sleep 3
    rosrun ov_msckf run_subscribe_msckf __name:=ov_msckf _config_path:=/tmp/c/estimator_config.yaml >/dev/null 2>&1 &
    sleep 3
    rosrun ov_eval pose_to_file _topic:=/ov_msckf/poseimu _topic_type:=PoseWithCovarianceStamped _output:=/out/rep.txt >/dev/null 2>&1 &
    sleep 1
    rosbag play --clock --rate 0.5 /data/tumvi/${SEQ}.bag >/dev/null 2>&1
    sleep 2; cp $SRC/ov_data/tum_vi/${SEQ}.txt /out/_gt.txt
    rosrun ov_eval error_singlerun se3 /out/_gt.txt /out/rep.txt 2>&1 | grep -aoE "rmse_pos = [0-9.]+"
  ' 2>&1 | grep -avE "Bag Time|RUNNING" | grep -aoE "[0-9.]+$" | head -1
}

printf "%-16s %8s %8s %8s %8s\n" "config" "run1" "run2" "run3" "mean_cm"
for entry in "${CONFIGS[@]}"; do
  name="${entry%%|*}"; seds="${entry#*|}"; vals=()
  for r in $(seq 1 $N); do
    m=$(run_once "$seds"); cm=$(awk "BEGIN{printf \"%.1f\", ${m:-0}*100}"); vals+=("$cm")
  done
  mean=$(printf "%s\n" "${vals[@]}" | awk '{s+=$1;n++} END{printf "%.1f", s/n}')
  printf "%-16s %8s %8s %8s %8s\n" "$name" "${vals[0]}" "${vals[1]}" "${vals[2]}" "$mean"
done
echo "REPEAT DONE"
