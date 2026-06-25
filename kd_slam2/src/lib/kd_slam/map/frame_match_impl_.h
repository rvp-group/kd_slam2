#pragma once
#include "frame_match_.h"
#include "map_align_utils_.h"

namespace kd_slam {
  namespace slam {
    using namespace kd_slam::frame;

    template <typename Node_>
    FrameMatch_<Node_>::FrameMatch_(FramePtr fixed,
                                    FramePtr moving,
                                    const IsometryType& p):
      result(MatchUndetermined),
      fixed_frame(fixed),
      moving_frame(moving),
      pose(p),
      hops(moving ? moving->hops_from_root : 0)
    {}

    template <typename Node_>
    typename FrameMatch_<Node_>::IsometryType
    FrameMatch_<Node_>::poseFromDescriptors( int fixed_canon, int moving_canon) const {
      IsometryType p_fixed  = fixed_frame->descriptor.toPose(*fixed_frame->tree, fixed_canon);
      IsometryType p_moving = moving_frame->descriptor.toPose(*moving_frame->tree, moving_canon);
      return p_fixed * p_moving.inverse();  // T_{moving_in_fixed}
    }

    template <typename Node_>
    typename FrameMatch_<Node_>::IsometryType
    FrameMatch_<Node_>::poseFromGraph() const {
      return fixed_frame->pose_in_world.inverse() * moving_frame->pose_in_world;
    }

    template <typename Node_>
    void FrameMatch_<Node_>::initFromDescriptors(int fixed_canon, int moving_canon){
      pose=poseFromDescriptors(fixed_canon, moving_canon);
    }

    template <typename Node_>
    void FrameMatch_<Node_>::initFromGraph(){
      pose = fixed_frame->pose_in_world.inverse() * moving_frame->pose_in_world;
    }

    template <typename Node_>
    void FrameMatch_<Node_>::filter(const Thresholds& th, const PoseHessianType& sigma_current) {
      // hops check
      if (th.min_hops >= 0 && hops < th.min_hops) { result = MatchFailHops; return; }
      if (th.max_hops >= 0 && hops > th.max_hops) { result = MatchFailHops; return; }

      // chi2 spatial check: compound cov of current frame (sigma_current) and candidate (BFS)
      // Sigma = Sigma_current_to_kf + Ad(current_to_candidate) * Sigma_kf_to_candidate * Ad^T
      using VectorType     = typename Node_::VectorType;
      using MatrixType     = PoseHessianType;
      using GeometryTraits = typename Node_::Traits::GeometryTraits;
      const IsometryType delta   = fixed_frame->pose_in_world.inverse() * moving_frame->pose_in_world;
      const VectorType delta_t=delta.translation();
      const VectorType slacked_t = applySlack(delta_t, th.slack);
      static constexpr int TDim = Node_::Dim;
      const MatrixType   Ad      = GeometryTraits::adjoint(delta);
      const MatrixType   Sigma   = sigma_current +
        Ad * moving_frame->marginal_covariance * Ad.transpose();
      const auto Omega_t = Sigma.template block<TDim,TDim>(0,0).inverse();
      chi2_spatial = slacked_t.transpose() * Omega_t * slacked_t;
      if (chi2_spatial > th.max_chi2_spatial) { result = MatchFailChi2Spatial; return; }

      result = MatchOk;
    }

    template <typename Node_>
    void FrameMatch_<Node_>::rematch(AlignerBase& aligner,
                                     const Thresholds& th,
                                     KDFactorType ft) {
      if (result != MatchOk)
        return;

      // ICP: keyframe fixed, root_frame moving, pose as initial guess
      alignFrames(aligner, fixed_frame, moving_frame, pose, ft);

      stats        = aligner.stats;
      int          num_leaves=std::max(moving_frame->tree->_num_leaves, fixed_frame->tree->_num_leaves);
      inlier_ratio = Scalar(stats.num_inliers) / Scalar(num_leaves);
      if (num_leaves)
        score = inlier_ratio / (Scalar(stats.chi2_kernelized) / Scalar(stats.num_inliers) + 1e-6f);

      if (inlier_ratio < th.min_inlier_ratio) {
        result = MatchFailInlierRatio;
      } else if (!std::isfinite(score) || score < th.min_score) {
        result = MatchFailScore;
      } else if(!aligner.getX().matrix().allFinite()) {
        result = MatchFailFinite;
      } else {
        pose  = aligner.getX();
        omega = aligner.getPoseHessian();
        result=MatchOk;
      }
      if (aligner.logStream()) {
        *aligner.logStream() << "MATCH_RESULT| " 
                        << " inlr: " << inlier_ratio
                        << " score: " << score;
        *aligner.logStream() << "| " << MatchLabelResultStr[result] << endl;
      }
    }


    template <typename Node_>
    std::ostream& FrameMatch_<Node_>::print(std::ostream& os) const {
      os << "kf:" << (fixed_frame ? (int)fixed_frame->ref() : -1)
         << " res:" << MatchLabelResultStr[result]
         << " chs:" << std::fixed << std::setprecision(2) << chi2_spatial
         << " hop:" << hops
         << " chn:" << std::setprecision(3) << stats.chi2_kernelized / stats.num_inliers
         << " inl:" << std::setprecision(1) << inlier_ratio * 100.f << "%"
         << " des:" << std::setprecision(3) << desc_distance
         << " scr:" << std::setprecision(3) << score;
      return os;
    }

  } // namespace slam
} // namespace kd_slam
