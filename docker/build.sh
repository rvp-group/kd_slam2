#!/bin/bash
# Usage: build.sh [--cuda] [--no-cache]
# Run from anywhere -- script locates the workspace root automatically.

set -e

CUDA=0
NO_CACHE=""

while [[ $# -gt 0 ]]; do
  case $1 in
    --cuda)     CUDA=1; shift ;;
    --no-cache) NO_CACHE="--no-cache"; shift ;;
    *) echo "Unknown option: $1"; exit 1 ;;
  esac
done

TAG="kd_slam2:$([ $CUDA -eq 1 ] && echo cuda || echo cpu)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KD_SLAM2="$(realpath "$SCRIPT_DIR/..")"
KD_SLAM2_MSGS="$(realpath "$SCRIPT_DIR/../../srrg2_ros2/src/kd_slam2_msgs")"

CONTEXT="$(mktemp -d)"
trap "rm -rf $CONTEXT" EXIT

# -L dereferences all symlinks (including imgui inside kd_viewer)
rsync -rL --exclude='.git' --exclude='build/' --exclude='install/' --exclude='log/' \
    "$KD_SLAM2/"      "$CONTEXT/kd_slam2/"
rsync -r  --exclude='.git' --exclude='build/' --exclude='install/' --exclude='log/' \
    "$KD_SLAM2_MSGS/" "$CONTEXT/kd_slam2_msgs/"

echo "=== Building $TAG (CUDA=$CUDA) ==="
echo "    kd_slam2:      $KD_SLAM2"
echo "    kd_slam2_msgs: $KD_SLAM2_MSGS"

docker build \
    $NO_CACHE \
    --build-arg CUDA=$CUDA \
    -t $TAG \
    -f "$SCRIPT_DIR/Dockerfile" \
    "$CONTEXT"

echo "=== Done: $TAG ==="
