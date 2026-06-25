#pragma once
#include "const_vel_motion_model_.h"
#include "kd_slam/utils/imu_ekf_.h"
#include <fstream>

namespace kd_slam {
  namespace slam {

    template <typename TrackerType_>
    struct IMUMotionModel_ : public ConstVelMotionModel_<TrackerType_> {
      using Base             = ConstVelMotionModel_<TrackerType_>;
      using typename Base::TrackerType;
      using typename Base::IsometryType;
      using typename Base::PoseHessianType;
      using typename Base::Frame;
      using Scalar           = typename Base::Scalar;
      using GeometryTraits   = typename TrackerType::GeometryTraits;
      using Vector3          = Eigen::Matrix<Scalar, 3, 1>;
      using PropertyVector3  = srrg2_core::PropertyEigen_<Vector3>;

      PARAM(srrg2_core::PropertyString, imu_dump_file,         "file where to dump imu",                                    "",              nullptr);
      PARAM(PropertyVector3,            imu_gyro_bias,         "initial bias of imu gyro",                                  Vector3::Zero(), nullptr);
      PARAM(PropertyVector3,            imu_acc_bias,          "initial bias of imu accelerometer",                         Vector3::Zero(), nullptr);
      PARAM(srrg2_core::PropertyFloat,  imu_sigma_scale,       "icp covariance scaling in ekf update",                      100.f,           nullptr);
      PARAM(srrg2_core::PropertyFloat,  imu_hint_max_rotation, "max imu gyro rotation for a valid ICP update hint (rad)",   0.1f,            nullptr);
      PARAM(srrg2_core::PropertyFloat,  gravity_prior_info,    "gravity prior information (negative = disabled)",            -1.f,            nullptr);

      IsometryType prediction() override;
      void doPredict(const Frame& frame) override;
      void doUpdate(const IsometryType& delta,
                    const PoseHessianType& sigma = PoseHessianType::Zero()) override;
      void reset() override;
      void onKeyframe(const IsometryType& pose_in_kf = IsometryType::Identity()) override;

    protected:
      ImuEKF       _imu_ekf;
      double       _last_imu_ts      = 0;
      bool         _had_imus         = false;
      Eigen::Matrix3f _R_ekf_init    = Eigen::Matrix3f::Identity();
      bool            _R_ekf_init_set = false;
      IsometryType _imu_prediction   = IsometryType::Identity();
      std::unique_ptr<std::ofstream> _imu_dump_stream;
    };

  } // namespace slam
} // namespace kd_slam
