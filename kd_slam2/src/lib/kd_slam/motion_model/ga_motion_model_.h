#pragma once
#include "zero_vel_motion_model_.h"
#include "kd_slam/utils/imu_filter.h"
namespace kd_slam{
  namespace slam {
    template <typename TrackerType_>
    struct GAMotionModel_: public ZeroVelMotionModel_<TrackerType_> {
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

      IMUFilter filter;
      
      PARAM(srrg2_core::PropertyFloat, motion_model_alpha, "low pass filter for the constant vel model",  0.9f,  nullptr);

      
      IsometryType prediction()  override  {
        return TrackerType::GeometryTraits::expmap(filter.predDelta());
      }

      void doPredict(const Frame& frame) override {
        Base::doPredict(frame);
        for (const auto& [ts,imu_entry]: frame.imus) {
          filter.predict(ts,
                         imu_entry.angular_velocity.template cast<float>(),
                         imu_entry.linear_acceleration.template cast<float>());
        }

      }

      // called by the owner each time a scan is integrated
      // the delta is the motion of floating (before - now). just 1 scan
      void doUpdate(const IsometryType& delta,
                    const PoseHessianType& sigma=PoseHessianType::Zero()) override {
        Base::doUpdate(delta, sigma);
        filter.update(_last_ts, TrackerType::GeometryTraits::logmap(delta));
      }
      // called by the owner when  a new keyframe is created
      void onKeyframe(const IsometryType& pose_in_kf = IsometryType::Identity()) override {} 

      void reset() override {
        filter.reset(true);
        _last_ts=-1;
        _frame_duration=0;
      }

      VelocityVectorType velocityLog() const override {
        if (_frame_duration<=0)
          return VelocityVectorType::Zero();
        return filter.predDelta();
      }
    protected:
    };
  }
}
