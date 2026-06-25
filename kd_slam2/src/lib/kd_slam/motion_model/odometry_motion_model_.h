#pragma once
#include "const_vel_motion_model_.h"
#include "pose_integrator.h"

namespace kd_slam {
  namespace slam {

    template <typename TrackerType_>
    struct OdometryMotionModel_ : public ConstVelMotionModel_<TrackerType_> {
      using BaseType        = ConstVelMotionModel_<TrackerType_>;
      using IsometryType    = typename BaseType::IsometryType;
      using PoseHessianType = typename BaseType::PoseHessianType;
      using Scalar          = typename BaseType::Scalar;
      using Frame           = typename BaseType::Frame;
      static_assert(BaseType::Dim == 3, "OdometryMotionModel_ is 3D only");

      PARAM(srrg2_core::PropertyFloat, sigma_nominal, "diagonal covariance at origin reset", 1e-4f, nullptr);

      IsometryType prediction() override {
        const auto delta = _integrator.prediction();
        IsometryType result = IsometryType::Identity();
        result.linear()      = delta.linear().cast<Scalar>();
        result.translation() = delta.translation().cast<Scalar>();
        return result;
      }

      PoseHessianType predictionCovariance() override {
        return _integrator.predictionCovariance().cast<Scalar>();
      }

      IsometryType priorXRef() const override { return _pose_prior_ref_in_kf; }

      IsometryType priorZ() const override {
        const auto delta = _integrator.priorDelta();
        IsometryType result = IsometryType::Identity();
        result.linear()      = delta.linear().cast<Scalar>();
        result.translation() = delta.translation().cast<Scalar>();
        return result;
      }

      PoseHessianType priorOmega() const override {
        return _integrator.predictionCovariance().inverse().cast<Scalar>();
      }

      void doPredict(const Frame& frame) override {
        BaseType::doPredict(frame);
        _integrator.sigma_nominal = double(param_sigma_nominal.value());
        for (const auto& [ts, odom] : frame.odometries)
          _integrator.onOdometry(odom);
      }

      void doUpdate(const IsometryType&, const PoseHessianType&) override {
        _integrator.onUpdate();
      }

      void onKeyframe(const IsometryType& pose_in_kf = IsometryType::Identity()) override {
        onOriginReset(pose_in_kf);
      }

      void onOriginReset(const IsometryType& pose_in_kf = IsometryType::Identity()) override {
        _pose_prior_ref_in_kf = pose_in_kf;
        _integrator.onOriginReset();
      }

      void reset() override {
        BaseType::reset();
        _pose_prior_ref_in_kf = IsometryType::Identity();
        _integrator.sigma_nominal = double(param_sigma_nominal.value());
        _integrator.reset();
      }

    private:
      IsometryType   _pose_prior_ref_in_kf = IsometryType::Identity();
      PoseIntegrator _integrator;
    };

  } // namespace slam
} // namespace kd_slam
