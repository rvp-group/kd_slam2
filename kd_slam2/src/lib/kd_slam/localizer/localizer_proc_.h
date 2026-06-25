#pragma once
#include "../tracker/tracker_proc_.h"

namespace kd_slam {
  namespace slam {
    using namespace kd_slam::icp;
    using namespace kd_slam::frame;
    using namespace kd_slam::utils;
    using namespace kd_slam::event;
    using namespace kd_slam::map;

    template <typename Node_>
    struct Localizer_ : public TrackerProc_<MapOwner_<Node_>> {

      using TrackerType  = TrackerProc_<MapOwner_<Node_>>;
      using MapOwnerType = MapOwner_<Node_>;
      using MapType      = Map_<Node_>;

      using typename TrackerType::NodeType;
      using typename TrackerType::Scalar;
      using typename TrackerType::IsometryType;
      using typename TrackerType::AlignerBase;
      using typename TrackerType::FrameTree;
      using typename TrackerType::FrameTreePtr;
      using typename TrackerType::Frame;
      using typename TrackerType::FramePtr;
      using typename TrackerType::VectorType;
      using typename TrackerType::ICPStats;

      using PGOFactor            = typename MapType::PGOFactor;
      using MultiViewICPFactor   = typename MapType::MultiViewICPFactor;
      using MultiViewCTICPFactor = typename MapType::MultiViewCTICPFactor;

      using typename TrackerType::EventFrameAdded;
      using typename TrackerType::EventFrameProcessed;
      using typename TrackerType::EventOdometry;
      using typename TrackerType::EventVelocity;
      using EventRelocalize = EventRelocalize_<Node_>;

      using TrackerType::_map;
      using TrackerType::pushEvent;
      using TrackerType::pushFrameEvents;
      using TrackerType::pushFactorEvents;
      using TrackerType::pushFactorUpdateEvents;
      using TrackerType::applyKFVelocities;
      using TrackerType::_keyframe;
      using TrackerType::_floating_frame;
      using TrackerType::_pose_in_kf;
      using TrackerType::_status;
      using TrackerType::_frame_idx;
      using TrackerType::_param_changed;
      using TrackerType::_odom_aligner;
      using TrackerType::poseGlobal;
      using TrackerType::makeFrame;
      using TrackerType::shouldSwitchKeyframe;
      using TrackerType::frameCount;

      LocalizerParams params;

      PARAM(srrg2_core::PropertyFloat, ba_range,                     "forest search range [m]",               10.f,  &_param_changed);
      PARAM(srrg2_core::PropertyFloat, relocalize_max_chi2,          "max chi2 for relocalization",           100.f, &_param_changed);
      PARAM(srrg2_core::PropertyInt,   relocalize_max_hops,          "max BFS hops (-1=no limit)",             40,   &_param_changed);
      PARAM(srrg2_core::PropertyFloat, relocalize_min_inliers_ratio, "min inlier ratio for relocalization",   0.7f,  &_param_changed);
      PARAM(srrg2_core::PropertyFloat, relocalize_min_score,         "min relocalization score",              40.f,  &_param_changed);
      PARAM(srrg2_core::PropertyFloat, relocalize_slack,             "covariance slack for chi2 test",        1.0f,  &_param_changed);
      PARAM(srrg2_core::PropertyConfigurable_<AlignerBase>, local_aligner, "local aligner for forest search", nullptr, &_param_changed);

      void addFrame(const FrameTreePtr& src) override;
      bool canCompute()   const override;
      bool isGPU()        const override;
      bool checkCompute() const override;
      void reset()        override;
      void syncParams()   override;
      void setMap(std::shared_ptr<MapType> m) override;

    protected:
      std::shared_ptr<AlignerBase> _local_aligner       = nullptr;
      FramePtr                     _last_real_keyframe;
      bool                         _using_dummy_keyframe = false;
      int                          _num_relocalizes      = 0;
    };

  } // namespace slam
} // namespace kd_slam
