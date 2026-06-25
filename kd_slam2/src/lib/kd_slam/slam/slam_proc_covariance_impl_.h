#pragma once

namespace kd_slam {
  namespace slam {
    using namespace kd_slam::frame;

    
    // propagates the covariances from current_keyframe
    template <typename T_>
    void SLAMProc_<T_>::propagateCovarianceAndDistance() {

      struct CovariancePropagateActions: public FrameGraph::VisitActions {
        using MatrixType = PoseHessianType;
        using GeometryTraits = typename NodeType::Traits::GeometryTraits;
        double traversalCost(FrameBase& src_, FrameBase& dest_, FactorBase& factor_) override {
          if (! factor_.is_valid)
            return std::numeric_limits<double>::max();
          return 1.0;
        }

        void traverseFactor(FrameBase& src_, FrameBase& dest_, FactorBase& factor_) override {
          auto* pf = dynamic_cast<PGOFactor*>(&factor_);
          if (!pf) return; // skip non-pose-pose factors

          Frame& src         = static_cast<Frame&>(src_);
          Frame& dest        = static_cast<Frame&>(dest_);
          PGOFactor& factor = static_cast<PGOFactor&>(factor_);
          const bool forward = (factor.from_ref == src.ref());
          const IsometryType& Z  = forward ? factor.Z_from_to    : factor.Z_to_from;
          const MatrixType& omega = forward ? factor.omega_from_to : factor.omega_to_from;
          // 1st order error propagation: Sigma_dest = Sigma_src + Ad(T_root_src)*Sigma_z*Ad(T_root_src)^T
          const IsometryType T_root_src = root_pose.inverse() * src.pose_in_world;
          const MatrixType   Ad         = GeometryTraits::adjoint(T_root_src);
          const MatrixType   Sigma_z    = omega.inverse();
          constexpr int TDim = NodeType::Dim;
          constexpr int RDim = AlignerBase::PPDim - TDim;
          MatrixType process_noise = MatrixType::Zero();
          process_noise.template block<TDim,TDim>(0,0).diagonal().setConstant(cond_trans);
          process_noise.template block<RDim,RDim>(TDim,TDim).diagonal().setConstant(cond_rot);
          dest.marginal_covariance = src.marginal_covariance + Ad * Sigma_z * Ad.transpose() + process_noise;
          // metric distance: walking distance on graph in meters
          dest.distance_from_root  = src.distance_from_root + Z.translation().norm();
          dest.hops_from_root       = src.hops_from_root + 1;
        }

        bool stop() override { return false; }

        IsometryType root_pose;
        Scalar cond_trans = 0;
        Scalar cond_rot   = 0;
      };

      if (!_keyframe) return;
      // root starts with the covariance of the last odom ICP (current frame -> keyframe)
      _keyframe->marginal_covariance.setZero();
      CovariancePropagateActions cov_prop;
      cov_prop.root_pose  = _keyframe->pose_in_world;
      cov_prop.cond_trans = _slam_params.covariance_propagate_trans_cond_diag;
      cov_prop.cond_rot   = _slam_params.covariance_propagate_rot_cond_diag;
      _keyframe->distance_from_root = 0;
      _map->visitBFS(_keyframe->ref(), cov_prop);
    }

  }
}
