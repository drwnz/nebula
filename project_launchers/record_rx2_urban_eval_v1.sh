#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_OUTPUT_BASE="${SCRIPT_DIR}/bags/rx2_urban_eval_v1_$(date +%Y%m%d_%H%M%S)"
OUTPUT_BASE="${DEFAULT_OUTPUT_BASE}"
STORAGE_ID="${STORAGE_ID:-sqlite3}"
MAX_BAG_DURATION_SECONDS="${MAX_BAG_DURATION_SECONDS:-60}"
MAX_CACHE_SIZE_BYTES="${MAX_CACHE_SIZE_BYTES:-0}"

usage() {
    cat <<EOF
Usage: $(basename "$0") [output_base]

Records the RX2 Urban Eval v1 sensor topics into a split rosbag2 recording.

Arguments:
  output_base   Optional output bag path prefix.
                Default: ${DEFAULT_OUTPUT_BASE}

Environment overrides:
  STORAGE_ID                rosbag2 storage backend (default: ${STORAGE_ID})
  MAX_BAG_DURATION_SECONDS  Split interval in seconds (default: ${MAX_BAG_DURATION_SECONDS})
  MAX_CACHE_SIZE_BYTES      rosbag2 cache size in bytes (default: ${MAX_CACHE_SIZE_BYTES})

Notes:
  - rosbag2 creates one metadata.yaml for the split recording set.
  - This script records raw packet topics and decoded pointcloud topics.
EOF
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
    usage
    exit 0
fi

if [[ $# -gt 1 ]]; then
    usage
    exit 1
fi

if [[ $# -eq 1 ]]; then
    OUTPUT_BASE="$1"
fi

mkdir -p "$(dirname "${OUTPUT_BASE}")"

TOPICS=(
    /at128/pandar_packets
    /at128/pandar_points
    /at128/aw_points
    /at128/aw_points_ex
    /ftx140/pandar_packets
    /ftx140/pandar_points
    /ftx140/aw_points
    /ftx140/aw_points_ex
    /ftx180/pandar_packets
    /ftx180/pandar_points
    /ftx180/aw_points
    /ftx180/aw_points_ex
    /ot128/pandar_packets
    /ot128/pandar_points
    /ot128/aw_points
    /ot128/aw_points_ex
    /robinw/seyond_packets
    /robinw/seyond_points
    /hummingbirdd1/seyond_packets
    /hummingbirdd1/seyond_points
    /e1/robosense_packets
    /e1/robosense_info_packets
    /e1/robosense_points
    /e1/aw_points
    /e1/aw_points_ex
)

exec ros2 bag record \
    --include-unpublished-topics \
    --output "${OUTPUT_BASE}" \
    --storage "${STORAGE_ID}" \
    --max-bag-duration "${MAX_BAG_DURATION_SECONDS}" \
    --max-cache-size "${MAX_CACHE_SIZE_BYTES}" \
    "${TOPICS[@]}"
