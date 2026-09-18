#pragma once
#include "zero_vel_motion_model_.h"
namespace kd_slam{
  namespace slam {
    template <typename TrackerType_>
    struct ConstVelMotionModel_: public ZeroVelMotionModel_<TrackerType_> {
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

      
      PARAM(srrg2_core::PropertyFloat, motion_model_alpha, "low pass filter for the constant vel model",  0.9f,  nullptr);

      
      IsometryType prediction()  override  {return TrackerType::GeometryTraits::expmap(_T_delta_log);}

      // called by the owner each time a scan is integrated
      // the delta is the motion of floating (before - now). just 1 scan
      void doUpdate(const IsometryType& delta,
                            const PoseHessianType& sigma=PoseHessianType::Zero()) override {
        Base::doUpdate(delta, sigma);
        float alpha=param_motion_model_alpha.value();
        VelocityVectorType dv=TrackerType::GeometryTraits::logmap(delta);
        _T_delta_log=alpha*_T_delta_log+(1.-alpha)*dv;
      }
      // called by the owner when  a new keyframe is created
      void onKeyframe(const IsometryType& pose_in_kf = IsometryType::Identity()) override {} 

      void reset() override {
        _last_ts=-1;
        _T_delta_log.setZero();
        _frame_duration=0;
      }

      VelocityVectorType velocityLog() const override { return _T_delta_log; }
    protected:
      VelocityVectorType _T_delta_log=VelocityVectorType::Zero();
    };
  }
}
