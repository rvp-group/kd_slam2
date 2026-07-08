#pragma once
#include "kd_slam/map/frame_match_.h"
#include "../optimizer/optimizer_proc_.h"
#include "../tracker/tracker_proc_.h"
#include "gravity_prior_host_.h"

namespace kd_slam {
  namespace slam {
    using namespace kd_slam::icp;
    using namespace kd_slam::frame;
    using namespace kd_slam::utils;
    using namespace kd_slam::event;
    using namespace kd_slam::map;

    template <typename T_>
    struct SLAMProc_ : public T_, public IGravityPriorHost {

      using BaseType = T_;
      using MapType  = typename BaseType::MapType;

      using typename BaseType::NodeType;
      using typename BaseType::Scalar;
      using typename BaseType::IsometryType;
      using typename BaseType::PoseHessianType;
      using typename BaseType::ICPStats;
      using typename BaseType::AlignerBase;
      using typename BaseType::TreeBaseType;
      using typename BaseType::FrameTree;
      using typename BaseType::FrameTreePtr;
      using typename BaseType::DescriptorType;
      using typename BaseType::DescriptorExtractor;
      using typename BaseType::DescriptorMatcher;
      using typename BaseType::Frame;
      using typename BaseType::FramePtr;
      using typename BaseType::VelocityVectorType;
      using typename BaseType::GeometryTraits;
      using typename BaseType::VectorType;
      using typename BaseType::SolverVariableType;
      using typename BaseType::SolverPGOFactorType;
      using typename BaseType::SolverVelocityVariableType;
      using typename BaseType::SolverVelocityPriorFactorType;
      using typename BaseType::SolverGravityPriorFactorType;
      using typename BaseType::PGOFactor;
      using typename BaseType::PGOFactorPtr;
      using typename BaseType::ICPType;
      using typename BaseType::CTICPType;
      using typename BaseType::MultiViewCTICPFactor;
      using typename BaseType::MultiViewCTICPFactorPtr;
      using typename BaseType::SolverFactorBridge;
      using typename BaseType::SolverFactorBridgePtr;
      using typename BaseType::SolverVelocityPriorFactor;
      using typename BaseType::EventFrameProcessed;
      using typename BaseType::EventFrameAdded;
      using typename BaseType::EventOdometry;
      using typename BaseType::EventVelocity;

      using EventKeyframeAdded = EventKeyframeAdded_<NodeType>;
      using EventFactorAdded   = EventFactorAdded_<NodeType>;
      using EventPGOOptimized  = EventPGOOptimized_<NodeType>;
      using EventLoopSearch    = EventLoopSearch_<NodeType>;
      using EventRelocalize    = EventRelocalize_<NodeType>;

      using BaseType::poseGlobal;
      using BaseType::frameCount;
      using BaseType::makeFrame;
      using BaseType::shouldSwitchKeyframe;
      using BaseType::_map;
      using BaseType::pushEvent;
      using BaseType::factorGraph;
      using BaseType::saveFrames;
      using BaseType::restoreFrames;
      using BaseType::pushFrameEvents;
      using BaseType::pushFactorUpdateEvents;
      using BaseType::pushFactorEvents;
      using BaseType::pushOptimizeEvent;
      using BaseType::pushFrameProcessedEvent;
      using BaseType::applyKFVelocities;
      using BaseType::optimize;
      using BaseType::sync;
      using BaseType::_solver;
      using BaseType::_need_optimize;
      using BaseType::_pending_chi2;
      using BaseType::_keyframe;
      using BaseType::_floating_frame;
      using BaseType::_pose_in_kf;
      using BaseType::_status;
      using BaseType::_frame_idx;
      using BaseType::_param_changed;
      using BaseType::_odom_aligner;
      using BaseType::_descriptor_matcher;
      using BaseType::_motion_model;
      using BaseType::_t_align;
      void addFrame(const FrameTreePtr& src) override;
      void reset()        override;
      void addGravityPrior(const Eigen::Vector3f& gravity_dir, float info) override;
      bool canCompute()   const override;
      bool checkCompute() const override;
      bool isGPU()        const override;
      void syncParams()   override;
      
      SLAMParams _slam_params;

      PARAM(srrg2_core::PropertyFloat, relocalize_max_chi2,          "max chi2 to accept relocalization",             100.f, &_param_changed);
      PARAM(srrg2_core::PropertyInt,   relocalize_max_hops,          "max BFS hops for relocalization (-1=no limit)",  40,   &_param_changed);
      PARAM(srrg2_core::PropertyFloat, relocalize_min_inliers_ratio, "min inlier ratio to accept relocalization",      0.7f, &_param_changed);
      PARAM(srrg2_core::PropertyFloat, relocalize_min_score,         "min relocalization score",                       40.f, &_param_changed);
      PARAM(srrg2_core::PropertyFloat, relocalize_slack,             "covariance slack for chi2 test",                 1.0f, &_param_changed);
      PARAM(srrg2_core::PropertyFloat, loop_max_chi2,                "max chi2 for loop closure candidate",           100.f, &_param_changed);
      PARAM(srrg2_core::PropertyInt,   loop_min_hops,                "min BFS hops to consider loop closure",          40,   &_param_changed);
      PARAM(srrg2_core::PropertyFloat, loop_consensus_max_orientation_deg,     "max orientation in the descriptor consensus [deg]",              10.f, &_param_changed);
      PARAM(srrg2_core::PropertyFloat, loop_consensus_max_translation,     "max translation in the descriptor consensus [m]", 0.5f, &_param_changed);
      PARAM(srrg2_core::PropertyFloat, loop_max_orientation_deg,     "max orientation discrepancy [deg]",              10.f, &_param_changed);
      PARAM(srrg2_core::PropertyFloat, loop_min_inliers_ratio,       "min inlier ratio for loop closure",              0.7f, &_param_changed);
      PARAM(srrg2_core::PropertyFloat, loop_min_score,               "min score for loop closure",                     40.f, &_param_changed);
      PARAM(srrg2_core::PropertyFloat, loop_slack,                   "covariance slack for loop chi2 test",            1.0f, &_param_changed);
      PARAM(srrg2_core::PropertyFloat, factor_max_chi2,              "max chi2 for factor creation",                  100.f, &_param_changed);
      PARAM(srrg2_core::PropertyFloat, factor_min_inliers_ratio,     "min inlier ratio for factor creation",           0.3f, &_param_changed);
      PARAM(srrg2_core::PropertyFloat, factor_min_score,             "min score for factor creation",                  0.1f, &_param_changed);
      PARAM(srrg2_core::PropertyFloat, factor_slack,                 "covariance slack for factor chi2 test",          0.5f, &_param_changed);
      PARAM(srrg2_core::PropertyFloat, covariance_propagate_trans_cond_diag, "translational condition number diagonal", 0.05f,  &_param_changed);
      PARAM(srrg2_core::PropertyFloat, covariance_propagate_rot_cond_diag,   "rotational condition number diagonal",   0.001f, &_param_changed);
      PARAM(srrg2_core::PropertyConfigurable_<AlignerBase>, local_aligner, "local aligner",        nullptr, &_param_changed);
      PARAM(srrg2_core::PropertyConfigurable_<AlignerBase>, loop_aligner,  "loop closure aligner", nullptr, &_param_changed);

      DescriptorMatcher& descriptorMatcher() { return *_descriptor_matcher; }
      void printLoopStats(std::ostream& os);
    protected:
      using Match = FrameMatch_<NodeType>;
      void propagateCovarianceAndDistance();
      bool addFactor(PGOFactorPtr f);
      void addKeyframe();
      void filterSpatially(std::unordered_map<int, Match>& candidates,
                           std::vector<int>& allowed_refs,
                           const MatchThresholds& thresholds);
      bool relocalize();
      PGOFactorPtr seekLoops();
      PGOFactorPtr makeFactor(FramePtr& from, FramePtr& to, KDFactorType ft, bool force = false);
      PGOFactorPtr makeFactor(const Match& match, KDFactorType ft);

      std::shared_ptr<AlignerBase> _local_aligner = nullptr;
      std::shared_ptr<AlignerBase> _loop_aligner  = nullptr;
      // temporaries to show monitor loop closure and descriptor path
      int stat_loop_spatial = 0;
      int stat_loop_desc2floating = 0;
      int stat_loop_desc2kf = 0;
      int stat_loop_ICP_tries = 0;
      int stat_loop_ICP_OK = 0;
    };

    template <typename Node_>
    using SLAM_ = SLAMProc_<OptimizerProc_<TrackerProc_<MapOwner_<Node_>>>>;

  } // namespace slam
} // namespace kd_slam
