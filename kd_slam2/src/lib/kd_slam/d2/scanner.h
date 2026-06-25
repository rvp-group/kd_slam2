#pragma once
#include "typedefs.h"

namespace kd_slam::d2 {

struct ScannerParams {
  float min_range = 0.1f;
  float max_range = 50.0f;
  float alpha_min = -M_PI;
  float alpha_max =  M_PI;
  float scan_time = 0.1f;
  int   num_beams = 360;
};

struct Scanner {
  ScannerParams params;
  void makeScan(PointsVectorType& scan,
                const std::vector<VectorType>& scene,
                const IsometryType& start_pose = IsometryType::Identity(),
                const PerturbationPoseType& velocities = PerturbationPoseType::Zero(),
                double t_start = 0.0);
};

} // namespace kd_slam::d2
