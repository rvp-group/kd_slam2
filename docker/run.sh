#!/bin/bash
# Usage: run.sh [--cuda] [--data <path>] <kd_slam_args...>
#
# Examples:
#   run.sh --data ~/kd_slam kd_slam -c /data/configs/kd_slam_icp_drive.conf -i /data/...
#   run.sh --cuda --data ~/kd_slam kd_slam -c /data/configs/...

set -e

CUDA=0
DATA=""

while [[ $# -gt 0 ]]; do
  case $1 in
    --cuda)  CUDA=1; shift ;;
    --data)  DATA="$2"; shift 2 ;;
    *)       break ;;
  esac
done

if [ $# -eq 0 ]; then
  echo "Usage: run.sh [--cuda] [--data <host_path>] <command> [args...]"
  echo "  --cuda          use the CUDA image (default: CPU)"
  echo "  --data <path>   mount host path as /data inside the container"
  exit 1
fi

IMAGE="kd_slam2:$([ $CUDA -eq 1 ] && echo cuda || echo cpu)"

GPU_FLAG=""
[ $CUDA -eq 1 ] && GPU_FLAG="--gpus all"

MOUNT=""
[ -n "$DATA" ] && MOUNT="-v $(realpath $DATA):/data"


DISPLAY_FLAG=""
[ -n "$DISPLAY" ] && DISPLAY_FLAG="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix"

exec docker run --rm --init \
    $GPU_FLAG \
    $MOUNT \
    $DISPLAY_FLAG \
    $IMAGE \
    bash -c "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && exec $*"
