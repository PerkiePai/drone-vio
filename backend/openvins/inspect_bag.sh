#!/usr/bin/env bash
# Print topics/types/counts for a MARS-LVIG bag using the openvins:noetic image.
# Usage: backend/openvins/inspect_bag.sh [path-to-bag]  (default: _in/mars-lvig/AMvalley01.bag)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BAG="${1:-$ROOT/_in/mars-lvig/AMvalley01.bag}"
[ -f "$BAG" ] || { echo "Bag not found: $BAG"; exit 1; }
docker run --rm -v "$(dirname "$BAG")":/data openvins:noetic \
  bash -lc "source /opt/ros/noetic/setup.bash && rosbag info /data/$(basename "$BAG")"
