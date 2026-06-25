#pragma once
#include "imu_motion_model_.h"
#include "kd_slam/slam/gravity_prior_host_.h"
#include <iomanip>

namespace kd_slam {
  namespace slam {

    template <typename TrackerType_>
    typename IMUMotionModel_<TrackerType_>::IsometryType
    IMUMotionModel_<TrackerType_>::prediction() {
      if constexpr (Base::Dim == 3)
        if (_imu_ekf.isValid())
          return _imu_prediction;
      return Base::prediction();
    }

    template <typename TrackerType_>
    void IMUMotionModel_<TrackerType_>::doPredict(const Frame& frame) {
      bool do_reset=this->_last_ts<0;
      Base::doPredict(frame);
      _had_imus       = false;
      _imu_prediction = IsometryType::Identity();

      if constexpr (Base::Dim == 3) {
        if (frame.imus.empty()) return;

        if (do_reset) {
          _imu_ekf.reset();
          _imu_ekf.bg              = param_imu_gyro_bias.value().template cast<float>();
          _imu_ekf.ba              = param_imu_acc_bias.value().template cast<float>();
          _imu_ekf.icp_sigma_scale = param_imu_sigma_scale.value();
          _R_ekf_init_set          = false;
        }

        if (!param_imu_dump_file.value().empty() && !_imu_dump_stream)
          _imu_dump_stream = std::make_unique<std::ofstream>(param_imu_dump_file.value());

        Eigen::Matrix3f R_before = _imu_ekf.R;
        for (auto& [ts, imu] : frame.imus) {
          double dt = ts - _last_imu_ts;
          if (_imu_dump_stream)
            *_imu_dump_stream << "IMU_PREDICT " << std::fixed << std::setprecision(9) << ts
                              << " " << std::setprecision(3)
                              << imu.angular_velocity.transpose() << " "
                              << imu.linear_acceleration.transpose() << "\n";
          _imu_ekf.predict(imu.angular_velocity.template cast<float>(),
                           imu.linear_acceleration.template cast<float>(),
                           static_cast<float>(dt));
          if (_imu_ekf.sampleCount() == 1)
            R_before = _imu_ekf.R;
          _last_imu_ts = ts;
        }

        Eigen::Matrix3f dR = R_before.transpose() * _imu_ekf.R;
        IsometryType dT    = IsometryType::Identity();
        dT.linear()        = dR.template cast<Scalar>();
        if (std::fabs(Eigen::AngleAxisf(dR).angle()) > param_imu_hint_max_rotation.value())
          dT.linear().setIdentity();
        _imu_prediction = dT;
        _had_imus       = true;
      }
    }

    template <typename TrackerType_>
    void IMUMotionModel_<TrackerType_>::doUpdate(const IsometryType& delta,
                                                  const PoseHessianType& sigma) {
      Base::doUpdate(delta, sigma);
      if constexpr (Base::Dim == 3) {
        if (!_had_imus || sigma.isZero()) return;
        if (_imu_dump_stream)
          *_imu_dump_stream << "IMU_UPDATE " << std::fixed << std::setprecision(3)
                            << delta.translation().transpose() << " "
                            << GeometryTraits::logSO3(delta.linear()).transpose() << "\n";
        _imu_ekf.update(delta.linear().template cast<float>(),
                        delta.translation().template cast<float>(),
                        sigma.template cast<float>());
      }
    }

    template <typename TrackerType_>
    void IMUMotionModel_<TrackerType_>::reset() {
      Base::reset();
      _imu_ekf.reset();
      _last_imu_ts    = 0;
      _had_imus       = false;
      _R_ekf_init     = Eigen::Matrix3f::Identity();
      _R_ekf_init_set = false;
      _imu_prediction = IsometryType::Identity();
      _imu_dump_stream.reset();
    }

    template <typename TrackerType_>
    void IMUMotionModel_<TrackerType_>::onKeyframe(const IsometryType&) {
      if constexpr (Base::Dim == 3) {
        float info = param_gravity_prior_info.value();
        if (info < 0 || !_imu_ekf.isValid()) return;
        auto* slam = dynamic_cast<IGravityPriorHost*>(this->_tracker);
        if (!slam) return;
        if (!_R_ekf_init_set) {
          _R_ekf_init     = _imu_ekf.R;
          _R_ekf_init_set = true;
        }
        Eigen::Vector3f gravity_dir = _R_ekf_init.transpose() * _imu_ekf.R.col(2);
        slam->addGravityPrior(gravity_dir, info);
      }
    }

  } // namespace slam
} // namespace kd_slam
