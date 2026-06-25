#include "pose_integrator.h"
#include "kd_slam/utils/geometry_3d_impl_.h"

namespace kd_slam {
  namespace slam {

    using GeometryD = geometry::GeometryTraits_<3, double>;

    void PoseIntegrator::onOdometry(const OdometryData& odom) {
      Isometry3d pose = Isometry3d::Identity();
      pose.linear()      = odom.orientation.toRotationMatrix();
      pose.translation() = odom.position;

      const double t_curr = odom.header.stamp_ns * 1e-9;

      if (!_initialized) {
        _pose_at_reset = pose;
        _pose_prev     = pose;
        _pose_current  = pose;
        _t_prev        = t_curr;
        _sigma         = Matrix6d::Identity() * sigma_nominal;
        _initialized   = true;
        return;
      }

      const double     dt   = t_curr - _t_prev;
      const Isometry3d step = _pose_prev.inverse() * pose;
      const Matrix6d   Ad   = GeometryD::adjoint(step.inverse());
      _sigma = Ad * _sigma * Ad.transpose() + odom.twist_covariance * dt;

      _pose_prev    = pose;
      _pose_current = pose;
      _t_prev       = t_curr;
    }

    void PoseIntegrator::onUpdate() {
      _pose_at_last_update = _pose_current;
    }

    PoseIntegrator::Isometry3d PoseIntegrator::prediction() const {
      return _pose_at_last_update.inverse() * _pose_current;
    }

    PoseIntegrator::Isometry3d PoseIntegrator::priorDelta() const {
      return _pose_at_reset.inverse() * _pose_current;
    }

    const PoseIntegrator::Matrix6d& PoseIntegrator::predictionCovariance() const {
      return _sigma;
    }

    void PoseIntegrator::onOriginReset() {
      _pose_at_reset       = _pose_current;
      _pose_at_last_update = _pose_current;
      _sigma               = Matrix6d::Identity() * sigma_nominal;
    }

    void PoseIntegrator::reset() {
      _initialized         = false;
      _pose_at_last_update = Isometry3d::Identity();
      _sigma               = Matrix6d::Identity() * sigma_nominal;
    }

  } // namespace slam
} // namespace kd_slam
