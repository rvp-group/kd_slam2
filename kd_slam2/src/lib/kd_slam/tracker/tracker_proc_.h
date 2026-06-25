#pragma once
#include "srrg_config/property_configurable.h"
#include "srrg_property/property_eigen.h"
#include "kd_slam/map/map_owner_.h"
#include "kd_slam/map/map_align_utils_.h"
#include "kd_slam/utils/runnable.h"
#include "kd_slam/frame/frame_consumer_base.h"
#include "kd_slam/motion_model/const_vel_motion_model_.h"

namespace kd_slam {
  namespace slam {
    using namespace kd_slam::icp;
    using namespace kd_slam::frame;
    using namespace kd_slam::utils;
    using namespace kd_slam::event;
    using namespace kd_slam::map;

    template <typename Base_>
    struct TrackerProc_ : public Base_,
                      public frame::FrameConsumerBase,
                      public Runnable {

      using MapType             = typename Base_::MapType;
      using NodeType            = typename Base_::NodeType;
      using Scalar              = typename Base_::Scalar;
      using IsometryType        = typename Base_::IsometryType;
      using PoseHessianType     = typename Base_::PoseHessianType;
      using ICPStats            = typename Base_::ICPStats;
      using TreeBaseType        = typename Base_::TreeBaseType;
      using AlignerBase         = typename Base_::AlignerBase;
      using FrameTree           = typename Base_::FrameTree;
      using FrameTreePtr        = typename Base_::FrameTreePtr;
      using DescriptorType      = typename Base_::DescriptorType;
      using DescriptorExtractor = typename Base_::DescriptorExtractor;
      using DescriptorMatcher   = descriptor::Matcher_<DescriptorType>;
      using Frame               = typename Base_::Frame;
      using FramePtr            = typename Base_::FramePtr;
      using VelocityVectorType  = typename Base_::VelocityVectorType;
      using GeometryTraits      = typename NodeType::Traits::GeometryTraits;
      using VectorType          = typename NodeType::VectorType;

      using EventFrameAdded     = typename Base_::EventFrameAdded;
      using EventFrameProcessed = typename Base_::EventFrameProcessed;
      using EventOdometry       = typename Base_::EventOdometry;
      using EventVelocity       = typename Base_::EventVelocity;
      using MotionModelType     = ConstVelMotionModel_<TrackerProc_<Base_>>;
      
      using Base_::_map;
      using Base_::pushEvent;
      using Base_::_status;
      using Base_::status;
      using Base_::_param_changed;
      
      TrackerParams _tracker_params;
      PARAM(srrg2_core::PropertyFloat, min_inlier_frac,    "min inlier fraction to keep keyframe",        0.6f,  &_param_changed);
      PARAM(srrg2_core::PropertyFloat, max_kf_trans,       "max translation before new keyframe [m]",     3.0f,  &_param_changed);
      PARAM(srrg2_core::PropertyFloat, max_kf_rot_deg,     "max rotation before new keyframe [deg]",      360.f, &_param_changed);
      PARAM(srrg2_core::PropertyConfigurable_<AlignerBase>,       odom_aligner,       "odometry aligner",   nullptr, &_param_changed);
      PARAM(srrg2_core::PropertyConfigurable_<DescriptorMatcher>, descriptor_matcher, "descriptor matcher", nullptr, &_param_changed);
      PARAM(srrg2_core::PropertyConfigurable_<MotionModelType>, motion_model,  "motion model used in the tracker", std::make_shared<MotionModelType>(), &_param_changed);

      virtual bool isGPU() const;
      virtual bool canCompute() const;
      virtual bool checkCompute() const;

      const IsometryType& poseGlobal() const { return _floating_frame? _floating_frame->pose_in_world : _pose_in_kf;}
      int    frameCount()    const { return _frame_idx; }
      int    keyframeCount() const { return _map->frames().size(); }
      double currentStamp()  const override {
        return _floating_frame ?
          _floating_frame->ts : 0;
      }

      void push(std::shared_ptr<frame::FrameBase> f) override;
      virtual void addFrame(const FrameTreePtr& src);
      void reset() override;
      void syncParams() override;
    protected:
      FramePtr makeFrame(FrameTreePtr src);
      bool shouldSwitchKeyframe(const IsometryType& X, const ICPStats& stats) const;

      std::shared_ptr<AlignerBase>       _odom_aligner       = nullptr;
      std::shared_ptr<DescriptorMatcher> _descriptor_matcher = nullptr;
      std::shared_ptr<MotionModelType>   _motion_model           = nullptr;

      IsometryType _pose_in_kf    = IsometryType::Identity();
      FramePtr     _keyframe;
      FramePtr     _floating_frame;
      int          _frame_idx     = 0;
      double       _t_align       = 0;

    };

  } // namespace slam
} // namespace kd_slam
