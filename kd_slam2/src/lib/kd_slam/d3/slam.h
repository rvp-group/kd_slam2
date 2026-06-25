#pragma once
#include "kd_slam/d3/typedefs.h"
#include "kd_slam/slam/slam_proc_.h"
#include "kd_slam/localizer/localizer_proc_.h"
#include "kd_slam/bundler/bundler_proc_.h"


namespace kd_slam::slam {
  using SLAM3D = SLAM_<kd_slam::d3::NodeType>;
  using Bundler3D = Bundler_<kd_slam::d3::NodeType>;
  using Localizer3D = Localizer_<kd_slam::d3::NodeType>;
  void __attribute__((constructor)) kd_slam_registerSLAM3DTypes();
}
