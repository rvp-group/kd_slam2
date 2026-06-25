#pragma once
#include <Eigen/Geometry>
#include "kd_io/slam_message_data.h"

namespace kd_slam {
  namespace slam {

    struct PoseIntegrator {
      using Isometry3d = Eigen::Isometry3d;
      using Matrix6d   = Eigen::Matrix<double, 6, 6>;

      double sigma_nominal = 1e-4;

      void onOdometry(const OdometryData& odom);
      void onUpdate();
      Isometry3d prediction()  const;  // increment since last onUpdate
      Isometry3d priorDelta()  const;  // accumulated from last reset
      const Matrix6d& predictionCovariance() const;
      void onOriginReset();
      void reset();

    private:
      bool       _initialized        = false;
      double     _t_prev             = 0.0;
      Isometry3d _pose_at_reset      = Isometry3d::Identity();
      Isometry3d _pose_at_last_update= Isometry3d::Identity();
      Isometry3d _pose_prev          = Isometry3d::Identity();
      Isometry3d _pose_current       = Isometry3d::Identity();
      Matrix6d   _sigma              = Matrix6d::Zero();
    };

  } // namespace slam
} // namespace kd_slam
