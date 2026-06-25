#include <iomanip>
#pragma once
namespace kd_slam {
  namespace slam {

    template <typename Base_>
    typename TrackerProc_<Base_>::IsometryType
    TrackerProc_<Base_>::imuPredict() {
      if (!param_imu_dump_file.value().empty() && !_imu_dump_file)
        _imu_dump_file.reset(new std::ofstream(param_imu_dump_file.value()));

      if constexpr (NodeType::Dim == 3) {
        if (!_keyframe) {
          _imu_ekf.reset();
          if (_imu_dump_file)
            *_imu_dump_file << "IMU_RESET" << std::endl;
        }
        IsometryType T_before;
        T_before.linear()      = _imu_ekf.R;
        T_before.translation() = _imu_ekf.t;

        for (auto [ts, imu] : _floating_frame->imus) {
          double dt = ts - _last_imu_ts;
          if (_imu_dump_file) {
            *_imu_dump_file << "IMU_PREDICT " << std::fixed
                            << std::setprecision(9) << ts << " "
                            << std::setprecision(3) << imu.angular_velocity.transpose() << " "
                            << imu.linear_acceleration.transpose() << std::endl;
          }
          _imu_ekf.predict(imu.angular_velocity.template cast<float>(),
                           imu.linear_acceleration.template cast<float>(), dt);
          if (_imu_ekf.sampleCount() == 1) {
            T_before.linear()      = _imu_ekf.R;
            T_before.translation() = _imu_ekf.t;
          }
          _last_imu_ts = ts;
        }
        IsometryType T_after;
        T_after.linear()      = _imu_ekf.R;
        T_after.translation() = _imu_ekf.t;
        IsometryType dT       = T_before.inverse() * T_after;
        dT.translation().setZero();
        if (fabs(Eigen::AngleAxisf(dT.linear()).angle()) > _tracker_params.imu_hint_max_rotation)
          dT.linear().setIdentity();
        return dT;
      }
      return IsometryType::Identity();
    }

    template <typename Base_>
    void TrackerProc_<Base_>::imuUpdate(const IsometryType& T_delta) {
      if constexpr (NodeType::Dim == 3) {
        if (!_floating_frame->imus.empty()) {
          auto sigma_local = _odom_aligner->getPoseHessian().inverse().eval()
            + PoseHessianType::Identity() * 1e-2;
          auto T_remap     = T_delta * _pose_in_kf.inverse();
          auto sigma_remap = (GeometryTraits::adjoint(T_remap)
                              * sigma_local
                              * GeometryTraits::adjoint(T_remap).transpose()).eval();
          if (sigma_remap.allFinite()) {
            if (_imu_dump_file) {
              *_imu_dump_file << "IMU_UPDATE " << std::fixed
                              << std::setprecision(9) << _floating_frame->ts << " "
                              << std::setprecision(3) << T_delta.translation().transpose() << " "
                              << GeometryTraits::logSO3(T_delta.linear()).transpose() << " ";
              for (int r = 0; r < sigma_remap.rows(); ++r)
                for (int c = r; c < sigma_remap.cols(); ++c)
                  *_imu_dump_file << std::scientific << sigma_remap(r, c) << " ";
              *_imu_dump_file << std::endl;
            }
            _imu_ekf.update(T_delta.linear(), T_delta.translation(), sigma_remap);
          } else {
            std::cerr << "NOT FINITE EKF UPDATE| " << sigma_local.determinant() << "\n";
          }
        }
      }
    }

  } // namespace slam
} // namespace kd_slam
