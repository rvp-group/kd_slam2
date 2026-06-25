#include "scan_utils_impl_.h"
#include "kd_slam/tree/tree_defs.h"

namespace kd_slam {

  template std::vector<Point3fTraits::PointType>
  toPointsVector<Point3fTraits, PointCloudXYZTData>(const PointCloudXYZTData& msg, double);

  template std::vector<Point2fTraits::PointType>
  toPointsVector<Point2fTraits, PointCloudXYTData>(const PointCloudXYTData& msg, double);

} // namespace kd_slam
