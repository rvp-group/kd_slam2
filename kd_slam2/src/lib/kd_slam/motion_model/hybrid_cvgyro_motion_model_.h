#pragma once
#include "zero_vel_motion_model_.h"
#include "gyro_filter.h"

namespace kd_slam{
  namespace slam {

    template <typename TrackerType_>
    struct HybridCVGyroMotionModel_: public ZeroVelMotionModel_<TrackerType_> {
      using Base             = ZeroVelMotionModel_<TrackerType_>;
      using typename Base::TrackerType;
      using typename Base::IsometryType;
      using typename Base::PoseHessianType;
      using typename Base::Frame;
      using typename Base::Scalar;
      using typename Base::VelocityVectorType;
      using Base::_frame_duration;
      using Base::_last_ts;
      
      using Base::Dim;
      GyroFilter gyro_filter;

      
      PARAM(srrg2_core::PropertyFloat, motion_model_alpha, "low pass filter for the linear_velocity",  0.9f,  &_params_changed);
      PARAM(srrg2_core::PropertyFloat, cov_gyro_x, "gyro cov on x",  0.00274f,  &_params_changed);
      PARAM(srrg2_core::PropertyFloat, cov_gyro_y, "gyro cov on y",  0.00777f,  &_params_changed);
      PARAM(srrg2_core::PropertyFloat, cov_gyro_z, "gyro cov on z",  0.000293f, &_params_changed);
      PARAM(srrg2_core::PropertyFloat, alpha_gyro_x, "gyro alpha on x",  0.5,  &_params_changed);
      PARAM(srrg2_core::PropertyFloat, alpha_gyro_y, "gyro alpha on y",  0.5,  &_params_changed);
      PARAM(srrg2_core::PropertyFloat, alpha_gyro_z, "gyro alpha on z",  0.05, &_params_changed);
      
      
      IsometryType prediction()  override  {
        handleParams();
        auto result=gyro_filter.getPrediction();
        if (! result.first) {
          return TrackerType::GeometryTraits::expmap(_T_delta_log);
        }
        auto v=_T_delta_log;
        auto dr_prev=_T_delta_log.template tail<3>();
        Eigen::Vector3f dr=result.second*_frame_duration;
        dr=_alpha_r*dr_prev+(Eigen::Matrix3f::Identity()-_alpha_r)*dr;
        v.template tail<3>()=dr;
        return TrackerType::GeometryTraits::expmap(v);
      };

      void doPredict(const Frame& frame) override {
        Base::doPredict(frame);
        for (const auto& [ts,imu_entry]: frame.imus) {
          gyro_filter.predict(ts,imu_entry.angular_velocity.template cast<Scalar>());
        }
      }

      // called by the owner each time a scan is integrated
      // the delta is the motion of floating (before - now). just 1 scan
      void doUpdate(const IsometryType& delta,
                            const PoseHessianType& sigma=PoseHessianType::Zero()) override {
        handleParams();
        Base::doUpdate(delta, sigma);
        float alpha=param_motion_model_alpha.value();
        VelocityVectorType dv=TrackerType::GeometryTraits::logmap(delta);
        bool ok=gyro_filter.update(_last_ts, dv);
        if (! ok) { 
          _T_delta_log=alpha*_T_delta_log+(1.-alpha)*dv;
        } else {
          auto dr_prev=_T_delta_log.template tail<3>();
          _T_delta_log=alpha*_T_delta_log+(1.-alpha)*dv;
          Eigen::Vector3f dr=gyro_filter.estimate*_frame_duration;
          dr=_alpha_r*dr_prev+(Eigen::Matrix3f::Identity()-_alpha_r)*dr;
          _T_delta_log.template tail<3>()=dr;
        }
      }
      // called by the owner when  a new keyframe is created
      void onKeyframe(const IsometryType& pose_in_kf = IsometryType::Identity()) override {} 

      void reset() override {
        _last_ts=-1;
        _T_delta_log.setZero();
        _frame_duration=0;
        gyro_filter.reset();
      }

      VelocityVectorType velocityLog() const override { return _T_delta_log; }
    protected:
      void handleParams(){
        if (!_params_changed)
          return;
        gyro_filter.sigma_meas(0,0)=param_cov_gyro_x.value();
        gyro_filter.sigma_meas(1,1)=param_cov_gyro_y.value();
        gyro_filter.sigma_meas(2,2)=param_cov_gyro_z.value();
        _alpha_r.setIdentity();
        _alpha_r(0,0)=param_alpha_gyro_x.value();
        _alpha_r(1,1)=param_alpha_gyro_y.value();
        _alpha_r(2,2)=param_alpha_gyro_z.value();
        _params_changed=false;
      }
      VelocityVectorType _T_delta_log=VelocityVectorType::Zero();
      bool _params_changed=true;
      Eigen::Matrix3f _alpha_r;
    };
  }
}
