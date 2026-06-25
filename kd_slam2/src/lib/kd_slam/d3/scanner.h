#pragma once
#include "typedefs.h"

namespace kd_slam::d3 {

struct ScannerParams {
  float min_range    = 0.1f;
  float max_range    = 200.f;
  float alpha_max    =  M_PI;
  float alpha_min    = -M_PI;
  float phi_max      =  M_PI / 4;
  float phi_min      = -M_PI / 4;
  float scan_time    = 0.1f;
  int   num_h_samples = 1024;
  int   num_v_samples =  128;
};

struct Scanner {
  ScannerParams params;
  void makeScan(PointsVectorType& scan,
                const std::vector<VectorType>& scene,
                const IsometryType& start_pose = IsometryType::Identity(),
                const PerturbationPoseType& velocities = PerturbationPoseType::Zero(),
                double t_start = 0.0);
};

} // namespace kd_slam::d3
