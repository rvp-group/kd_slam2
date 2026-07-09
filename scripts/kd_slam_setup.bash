# kd_slam_setup.bash -- source this file to set up the KD-SLAM environment.
# Edit the two variables below, then source it when needed:
#   source ~/kd_slam_setup.bash

# --- User-editable -----------------------------------------------------------
unset KD_SLAM_ROS_WORKSPACE KD_SLAM_TEST
export KD_SLAM_ROS_WORKSPACE=$HOME/ws
export KD_SLAM_TEST=$HOME/kd_slam
# -----------------------------------------------------------------------------

if [ -f /.dockerenv ]; then
  export KD_SLAM_ROS_WORKSPACE=/ws
  export KD_SLAM_TEST=/data
fi

export KD_SLAM_ROS_WORKSPACE_INSTALL=$KD_SLAM_ROS_WORKSPACE/install
export KD_SLAM_CONFIGS=$KD_SLAM_ROS_WORKSPACE/src/kd_slam2/kd_slam2/configs

export PATH=$KD_SLAM_ROS_WORKSPACE_INSTALL/kd_slam2/bin:$KD_SLAM_ROS_WORKSPACE_INSTALL/kd_eval_tools/lib/kd_eval_tools:$PATH

echo "workspace: $KD_SLAM_ROS_WORKSPACE_INSTALL"
source "$KD_SLAM_ROS_WORKSPACE_INSTALL/setup.bash"

if [ ! -f "$HOME/dl.conf" ] && [ -f "$KD_SLAM_CONFIGS/dl.conf" ]; then
  cp "$KD_SLAM_CONFIGS/dl.conf" "$HOME/dl.conf"
  echo "kd_slam_setup: copied dl.conf to $HOME/dl.conf"
fi

if [ -f /.dockerenv ] && [ ! -f /dl.conf ] && [ -f "$KD_SLAM_CONFIGS/dl.conf" ]; then
  cp "$KD_SLAM_CONFIGS/dl.conf" /dl.conf
  echo "kd_slam_setup: copied dl.conf to /dl.conf (docker)"
fi

alias confviz='$KD_SLAM_ROS_WORKSPACE_INSTALL/srrg2_config_visualizer/bin/app_node_editor'
