#!/bin/bash
# Pull srrg dependencies into the current ROS2 workspace src directory.
# Run from ~/ws/src after cloning kd_slam2.

set -e
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
vcs import < "$SCRIPT_DIR/../docker/repos.yml"
