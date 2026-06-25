#pragma once
#include "kd_slam/d2/typedefs.h"
#include "kd_slam/slam/slam_proc_.h"
#include "kd_slam/localizer/localizer_proc_.h"
#include "kd_slam/bundler/bundler_proc_.h"


namespace kd_slam::slam {
  using SLAM2D = SLAM_<kd_slam::d2::NodeType>;
  using Bundler2D = Bundler_<kd_slam::d2::NodeType>;
  using Localizer2D = Localizer_<kd_slam::d2::NodeType>;
  void __attribute__((constructor)) kd_slam_registerSLAM2DTypes();
}
