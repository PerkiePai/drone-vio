#!/usr/bin/env bash
# Convert an Isaac Sim CSV dataset to a ROS1 bag inside the openvins:noetic container.
#
# Usage:
#   backend/openvins/run_isaac_to_bag.sh [run_dir] [out.bag] [extra args...]
#
# Defaults:
#   run_dir  = _in/isaac-sim-20260623
#   out.bag  = <run_dir>/isaac_sim.bag
#
# Examples:
#   backend/openvins/run_isaac_to_bag.sh
#   backend/openvins/run_isaac_to_bag.sh /path/to/run /tmp/out.bag --rgb
#
# The converter runs inside Docker where rosbag + ROS message types are available.

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

RUN_DIR="${1:-$SCRIPT_DIR/../../_in/isaac-sim-20260624}"
shift || true

# Output bag defaults to <run_dir>/isaac_sim.bag inside the container mount
OUT_BAG="${1:-}"
shift || true

EXTRA_ARGS=("$@")

[ -d "$RUN_DIR" ] || { echo "ERROR: run_dir not found: $RUN_DIR"; exit 1; }

# Mount the dataset read-only at /run_data, scripts at /scripts
# Output bag lands inside /run_data unless an explicit external path is given.
if [ -z "$OUT_BAG" ]; then
    CONTAINER_OUT="/run_data/isaac_sim.bag"
    HOST_OUT="$RUN_DIR/isaac_sim.bag"
    EXTRA_VOL=()
else
    OUT_PARENT="$(dirname "$OUT_BAG")"
    mkdir -p "$OUT_PARENT"
    CONTAINER_OUT="/bag_out/$(basename "$OUT_BAG")"
    HOST_OUT="$OUT_BAG"
    EXTRA_VOL=(-v "$(realpath "$OUT_PARENT")":/bag_out)
fi

echo ">>> Converting Isaac Sim dataset to ROS bag..."
echo "    Input:  $RUN_DIR"
echo "    Output: $HOST_OUT"

docker run --rm \
  -v "$(realpath "$RUN_DIR")":/run_data \
  "${EXTRA_VOL[@]}" \
  -v "$SCRIPT_DIR":/scripts:ro \
  openvins:noetic bash -lc "
    source /opt/ros/noetic/setup.bash
    python3 /scripts/isaac_to_bag.py /run_data \
        --out $CONTAINER_OUT ${EXTRA_ARGS[*]:-}
  "

echo ">>> Bag written to: $HOST_OUT"
echo "    Run OpenVINS on it with: backend/openvins/run_isaac_sim.sh $HOST_OUT"
