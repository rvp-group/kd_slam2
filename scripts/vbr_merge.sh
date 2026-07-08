#!/bin/bash
# vbr_merge.sh -- convert and merge VBR ROS1 bag chunks into a single ROS2 bag
#
# Usage: vbr_merge.sh <seq_name> <chunks_dir>
#
#   seq_name   : sequence name (e.g. colosseo, diag)
#   chunks_dir : directory containing the *.bag ROS1 chunk files
#
# Output: $BAGS/<seq_name>/  (ROS2 mcap bag, ready for kd_runme.sh)
#
# Override env vars:
#   BAGS   -- output root     (default: ~/kd_slam/vbr/bags)
#   TOPICS -- topics to keep  (default: /ouster/points /ouster/imu)
#
# Requires: pipx install rosbags

set -e

SEQ=${1:?Usage: vbr_merge.sh <seq_name> <chunks_dir>}
CHUNKS_DIR=${2:?Usage: vbr_merge.sh <seq_name> <chunks_dir>}
BAGS=${BAGS:-~/kd_slam/vbr/bags}
TOPICS=${TOPICS:-"/ouster/points /ouster/imu"}
OUT=$(eval echo "$BAGS/$SEQ")

CONVERT=$(command -v rosbags-convert 2>/dev/null)
if [ -z "$CONVERT" ]; then
  echo "ERROR: rosbags-convert not found. Install with: pipx install rosbags"
  exit 1
fi

mapfile -t chunks < <(ls "$CHUNKS_DIR"/${SEQ}*.bag 2>/dev/null | sort)
if [ ${#chunks[@]} -eq 0 ]; then
  echo "ERROR: no .bag files found in $CHUNKS_DIR"
  exit 1
fi

if [ -d "$OUT" ]; then
  echo "ERROR: output $OUT already exists; remove it first"
  exit 1
fi

echo "=== $SEQ: ${#chunks[@]} chunk(s) ==="
for c in "${chunks[@]}"; do echo "  $c"; done

topic_args=()
for t in $TOPICS; do
  topic_args+=(--include-topic "$t")
done

mkdir -p "$(dirname "$OUT")"

echo "  converting -> $OUT ..."
"$CONVERT" --src "${chunks[@]}" --dst "$OUT" "${topic_args[@]}"

echo "=== done: $OUT ==="
