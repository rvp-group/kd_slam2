#pragma once
#include <vector>
#include "kd_io/slam_messages.h"

namespace kd_slam {

  // Pure deserialization: converts a point-cloud message to a typed point vector
  // with azimuth-based timestamp synthesis when scan_duration > 0.
  // No range filtering, no voxelization -- those are handled by Voxelizer_.

  template <typename PointTraits_, typename PointCloudMsgType>
  std::vector<typename PointTraits_::PointType>
  toPointsVector(const PointCloudMsgType& msg, double scan_duration = -1.0);

  template <typename PointTraits_>
  std::vector<typename PointTraits_::PointType>
  toPointsVector(const PointCloudLivox& msg, double scan_duration = -1.0);

} // namespace kd_slam
