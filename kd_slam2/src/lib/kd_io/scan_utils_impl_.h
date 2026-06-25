#pragma once
#include "scan_utils_.h"
#include <cmath>

namespace kd_slam {

  template <typename PointTraits_,
            typename PointCloudMsgType>
  std::vector<typename PointTraits_::PointType>
  toPointsVector(const PointCloudMsgType& msg, double scan_duration) {
    using PointType = typename PointTraits_::PointType;
    const double msg_t = msg.header.stamp_ns * 1e-9;

    std::vector<PointType> pts;
    pts.reserve(msg.points.size());
    for (const auto& pt : msg.points) {
      PointType p;
      PointTraits_::coordinates(p) = pt.coords;
      if constexpr (PointTraits_::HasTimestamp) {
        double t;
        if (msg.has_per_point_timestamps) {
          t = pt.t;
        } else if (scan_duration > 0.0) {
          double alpha = std::atan2(double(pt.coords.y()), double(pt.coords.x()));
          t = msg_t + (alpha + M_PI) / (2.0 * M_PI) * scan_duration;
        } else {
          t = msg_t;
        }
        PointTraits_::stamp(p) = t;
      }
      pts.push_back(p);
    }
    return pts;
  }

  template <typename PointTraits_>
  std::vector<typename PointTraits_::PointType>
  toPointsVector(const PointCloudLivox& msg, double /*scan_duration*/) {
    using PointType = typename PointTraits_::PointType;
    std::vector<PointType> pts;
    pts.reserve(msg.points.size());
    for (const auto& pt : msg.points) {
      if (! pt.coords.allFinite())
        continue;
      PointType p;
      PointTraits_::coordinates(p) = pt.coords;
      if constexpr (PointTraits_::HasTimestamp)
        PointTraits_::stamp(p) = (msg.timebase + pt.t) * 1e-9;
      pts.push_back(p);
    }
    return pts;
  }
 

} // namespace kd_slam
