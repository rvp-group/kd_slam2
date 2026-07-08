#!/bin/bash
# kitti2standard.sh -- one-time KITTI dataset preparation
# Converts raw velodyne sequences to mcap bags and GT poses to TUM format.
# Run once when you get a new KITTI copy; then use run_kitti.sh for everything else.
#
# Usage: kitti2standard.sh [sequences...]
#   default: 00 01 02 03 04 05 06 07 08 09 10

set -e

KITTI=${KITTI:-~/kd_slam/kitti/dataset}
BAGS=${BAGS:-$KITTI/bags}
GT=${GT:-$KITTI/gt}

INSTALL=${INSTALL:-${KD_SLAM_ROS_WORKSPACE_INSTALL:-~/ws/install}/kd_eval_tools}
CONVERTER=$INSTALL/lib/kd_eval_tools/kitti2ros2bag
GT_CONV=$INSTALL/lib/kd_eval_tools/kitti_gt2tum

SEQUENCES=${@:-00 01 02 03 04 05 06 07 08 09 10}

mkdir -p "$BAGS" "$GT"

for seq in $SEQUENCES; do
  src=$KITTI/sequences/$seq
  bag=$BAGS/$seq
  poses=$KITTI/poses/${seq}.txt
  times=$src/times.txt
  calib=$src/calib.txt
  gt_out=$GT/${seq}_gt.tum

  [ -d "$src" ] || { echo "SKIP $seq: $src not found"; continue; }

  if [ -d "$bag" ]; then
    echo "SKIP $seq: bag already exists at $bag"
  else
    echo "=== $seq: converting velodyne -> mcap ==="
    $CONVERTER "$src" "$bag"
  fi

  if [ -f "$gt_out" ]; then
    echo "SKIP $seq: GT already exists at $gt_out"
  elif [ ! -f "$poses" ]; then
    echo "SKIP $seq GT: $poses not found (test sequence?)"
  else
    echo "=== $seq: converting GT -> TUM ==="
    $GT_CONV "$poses" "$times" "$gt_out" "$calib"
  fi
done

echo "=== ALL DONE ==="
