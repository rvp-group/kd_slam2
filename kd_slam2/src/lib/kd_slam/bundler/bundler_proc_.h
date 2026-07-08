#pragma once
#include "../optimizer/optimizer_proc_.h"
#include "../map/frame_match_.h"
#include "kd_slam/utils/runnable.h"

namespace kd_slam {
  namespace slam {

    template <typename T_>
    struct BundlerProc_ : public OptimizerProc_<T_>,
                          public utils::Runnable
    {

      using BaseType        = OptimizerProc_<T_>;
      using OptimizerType   = OptimizerProc_<T_>;
      using MapType         = typename T_::MapType;
      using NodeType        = typename T_::NodeType;
      using Scalar          = typename T_::Scalar;
      using IsometryType    = typename T_::IsometryType;
      using PoseHessianType = typename T_::PoseHessianType;
      using ICPStats        = typename T_::ICPStats;
      using Frame           = typename T_::Frame;
      using FramePtr        = typename T_::FramePtr;
      using AlignerBase     = typename T_::AlignerBase;
      using Match           = FrameMatch_<NodeType>;
      using GeometryTraits  = typename NodeType::Traits::GeometryTraits;
      using typename OptimizerType::ICPType;
      using typename OptimizerType::CTICPType;
      using typename OptimizerType::MultiViewICPFactor;
      using typename OptimizerType::MultiViewICPFactorPtr;
      using typename OptimizerType::MultiViewCTICPFactor;
      using typename OptimizerType::MultiViewCTICPFactorPtr;
      using typename OptimizerType::SolverFactorBridge;
      using typename OptimizerType::SolverFactorBridgePtr;
      using typename OptimizerType::PGOFactor;
      using typename OptimizerType::PGOFactorPtr;
      using typename OptimizerType::EventFrameProcessed;
      
      using EventFactorAdded = EventFactorAdded_<NodeType>;


      using OptimizerType::_map;
      using OptimizerType::_status;
      using OptimizerType::_param_changed;
      using OptimizerType::pushEvent;
      using OptimizerType::pushFrameEvents;
      using OptimizerType::factorGraph;
      using OptimizerType::saveFrames;
      using OptimizerType::restoreFrames;
      using OptimizerType::applyKFVelocities;
      using OptimizerType::optimize;

      void syncParams() override;

      BAParams _ba_params;

      PARAM(srrg2_core::PropertyFloat, ba_range,
            "range for multiview icp [m]",           10.f,  &_param_changed);
      PARAM(srrg2_core::PropertyFloat, ba_disable_inlier_ratio,
            "factors disabled if overlap below this", 0.3f,  &_param_changed);
      PARAM(srrg2_core::PropertyConfigurable_<AlignerBase>, multi_icp_aligner,
            "multiview icp aligner",    nullptr, &_param_changed);
      PARAM(srrg2_core::PropertyConfigurable_<AlignerBase>, multi_cticp_aligner,
            "multiview ct-icp aligner", nullptr, &_param_changed);
      PARAM(srrg2_core::PropertyFloat, cure_min_inlier_ratio,
            "min fraction of inliers for cure",           0.3f,  &_param_changed);
      PARAM(srrg2_core::PropertyFloat, cure_min_score,
            "min score for cure to consider a match",     3.f,  &_param_changed);

      std::unordered_map<IndexPair, bool, IndexPairHash>
      seekBundleFactors();

      void propagateDistance(FramePtr root_frame, double max_distance);
      void pruneBundleFactors(std::unordered_map<IndexPair, bool, IndexPairHash>& candidates);
      bool isGPU() const;
      void cure();
      void bundle();
      void bundleCT();
      void bundleCTVelocityOnly();
      void addPoseNoise(float t_sigma, float r_sigma);
      void addVelNoise(float tv_sigma, float rv_sigma);
    protected:
      bool addFactor(MultiViewICPFactorPtr f);
      bool addFactor(MultiViewCTICPFactorPtr f);
      bool addFactor(PGOFactorPtr f);
      
      std::shared_ptr<ICPType>   _multi_icp_aligner   = nullptr;
      std::shared_ptr<CTICPType> _multi_cticp_aligner = nullptr;
    };

    template <typename Node_>
    using Bundler_ = BundlerProc_<MapOwner_<Node_>>;

  } // namespace slam
} // namespace kd_slam
