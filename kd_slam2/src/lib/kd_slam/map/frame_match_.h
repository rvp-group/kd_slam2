#pragma once
#include "kd_slam/map/map_.h"

namespace kd_slam {
  namespace slam {

    template <typename Node_>
    struct FrameMatch_ {
      using MapType     = map::Map_<Node_>;
      using Scalar        = typename MapType::Scalar;
      using IsometryType  = typename MapType::IsometryType;
      using PoseHessianType = typename MapType::PoseHessianType;
      using ICPStats      = typename MapType::ICPStats;
      using AlignerBase   = typename MapType::AlignerBase;
      using FramePtr      = typename MapType::FramePtr;

      using Thresholds = MatchThresholds;

      FrameMatch_(FramePtr fixed  = nullptr,
                 FramePtr moving = nullptr,
                 const IsometryType& p = IsometryType::Identity());

      MatchLabelResult result;
      FramePtr         fixed_frame;
      FramePtr         moving_frame;
      IsometryType     pose          = IsometryType::Identity();
      Scalar           inlier_ratio  = 0;
      Scalar           score         = 0;
      Scalar           chi2_spatial  = std::numeric_limits<Scalar>::max();
      Scalar           desc_distance = -1;
      int              hops          = 0;
      ICPStats         stats;
      PoseHessianType  omega         = PoseHessianType::Zero();

      std::ostream& print(std::ostream& os) const;
      friend std::ostream& operator<<(std::ostream& os, const FrameMatch_& m) { return m.print(os); }

      void initFromGraph();
      void initFromDescriptors(int fixed_canon = -1, int moving_canon = -1);
      IsometryType poseFromGraph() const;
      IsometryType poseFromDescriptors(int fixed_canon = -1, int moving_canon = -1) const;
      void filter(const Thresholds& th, const PoseHessianType& sigma_current = PoseHessianType::Zero());
      void rematch(AlignerBase& aligner,
                   const Thresholds& th,
                   KDFactorType ft);
    };

  } // namespace slam
} // namespace kd_slam
