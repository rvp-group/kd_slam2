#pragma once
#include <Eigen/Core>

namespace kd_slam {
  namespace slam {

    struct IGravityPriorHost {
      virtual void addGravityPrior(const Eigen::Vector3f& gravity_dir, float info) = 0;
      virtual ~IGravityPriorHost() = default;
    };

  } // namespace slam
} // namespace kd_slam
