#!/bin/bash
# Usage: run.sh [--cuda]
# Starts an interactive kd_slam2 container with X11 and data mounted.
# Data root: $KD_SLAM_TEST (default: ~/kd_slam), available as /data inside.
# CUDA: requires CDI setup (sudo bash docker/setup_cdi.sh, once per host).

TARGET="cpu"

while [[ $# -gt 0 ]]; do
  case $1 in
    --cuda) TARGET="cuda"; shift ;;
    *) echo "Unknown option: $1"; exit 1 ;;
  esac
done

xhost +local:docker 2>/dev/null

CUDA_ARGS=""
[ "$TARGET" = "cuda" ] && CUDA_ARGS="--device nvidia.com/gpu=all"

exec docker run --rm -it \
    --network host \
    $CUDA_ARGS \
    -e DISPLAY="$DISPLAY" \
    -v "${KD_SLAM_TEST:-$HOME/kd_slam}:/data" \
    kd_slam2:$TARGET bash
