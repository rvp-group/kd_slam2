#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <iomanip>

namespace kd_slam {
  template <typename IsometryT>
  static void writeIsometry3(std::ostream& os, const IsometryT& T) {
    os << std::fixed <<  std::setprecision(4);
    if constexpr (IsometryT::Dim == 3) {
      Eigen::Quaternionf q(T.linear());
      const auto& t = T.translation();
      os << t.x() << " " << t.y() << " " << t.z() << " "
         << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
    } else {
      const auto& t = T.translation();
      float theta = std::atan2(T.linear()(1,0), T.linear()(0,0));
      Eigen::Quaternionf q(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
      os << t.x() << " " << t.y() << " 0 "
         << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
    }
  }

  template <typename IsometryT>
  static void writeIsometry(std::ostream& os, const IsometryT& T) {
    os << std::fixed <<  std::setprecision(4);
    if constexpr (IsometryT::Dim == 3) {
      Eigen::Quaternionf q(T.linear());
      const auto& t = T.translation();
      os << t.x() << " " << t.y() << " " << t.z() << " "
         << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
    } else {
      const auto& t = T.translation();
      float theta = std::atan2(T.linear()(1,0), T.linear()(0,0));
      os << t.x() << " " << t.y() << theta;
    }
  }

  
}
