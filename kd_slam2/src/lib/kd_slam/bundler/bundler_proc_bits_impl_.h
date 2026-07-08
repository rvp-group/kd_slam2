#pragma once
namespace kd_slam {
  namespace slam {

    template <typename T_>
    bool BundlerProc_<T_>::isGPU() const {
      if (_multi_icp_aligner)   return _multi_icp_aligner->isGPU();
      if (_multi_cticp_aligner) return _multi_cticp_aligner->isGPU();
      return false;
    }

    template <typename T_>
    void BundlerProc_<T_>::addPoseNoise(float t_sigma, float r_sigma) {
      static constexpr int Dim=NodeType::Dim;
      using PerturbationPoseType = typename ICPType::PerturbationPoseType;
      for (auto& [_, f_] :_map->frames()) {
        FramePtr f = dynamic_pointer_cast<Frame>(f_);
        PerturbationPoseType n=PerturbationPoseType::Random();
        for (int i=0; i<Dim;++i)
          n(i)*=t_sigma;
        for (int i=Dim; i<PerturbationPoseType::RowsAtCompileTime; ++i)
          n(i)*=r_sigma;
        IsometryType iso_n=NodeType::GeometryTraits::expmap(n);
        if (f->solver_variable->status()!=VariableBase::Fixed) {
          f->pose_in_world=f->pose_in_world*iso_n;
          f->solver_variable->setEstimate(f->pose_in_world);
        } 
      }
      pushFrameEvents();
   }

    template <typename T_>
    void BundlerProc_<T_>::addVelNoise(float tv_sigma, float rv_sigma) {
      static constexpr int Dim=NodeType::Dim;
      using PerturbationPoseType = typename ICPType::PerturbationPoseType;
      for (auto& [_, f_] :_map->frames()) {
        FramePtr f = dynamic_pointer_cast<Frame>(f_);
        PerturbationPoseType n=PerturbationPoseType::Random();
        for (int i=0; i<Dim;++i)
          n(i)*=tv_sigma;
        for (int i=Dim; i<PerturbationPoseType::RowsAtCompileTime; ++i)
          n(i)*=rv_sigma;
        if (f->solver_variable->status()!=VariableBase::Fixed) {
          f->tree->applyVelocitiesInPlace(n);
        }
      }
      pushFrameEvents();
    }

    template <typename T_>
    void BundlerProc_<T_>::syncParams() {
      if (!_param_changed)
        return;
      BaseType::syncParams();
      _ba_params.ba_range               = param_ba_range.value();
      _ba_params.ba_disable_inlier_ratio = param_ba_disable_inlier_ratio.value();
      _multi_icp_aligner   = std::dynamic_pointer_cast<ICPType>(param_multi_icp_aligner.value());
      _multi_cticp_aligner = std::dynamic_pointer_cast<CTICPType>(param_multi_cticp_aligner.value());
      _ba_params.cure_thresholds.min_score=param_cure_min_score.value();
      _ba_params.cure_thresholds.min_inlier_ratio=param_cure_min_inlier_ratio.value();
      
    }

    template <typename T_>
    bool BundlerProc_<T_>::addFactor(MultiViewICPFactorPtr f) {
      bool result = _map->addFactor(f);
      if (!result)
        return result;
      ICPStats empty{};
      pushEvent(std::make_shared<EventFactorAdded>(0,
                                                   f->from_ref,
                                                   f->to_ref,
                                                   IsometryType::Identity(),
                                                   PoseHessianType::Identity(),
                                                   empty,
                                                   f->type,
                                                   (size_t)f.get()));
      return result;
    }

    template <typename T_>
    bool BundlerProc_<T_>::addFactor(MultiViewCTICPFactorPtr f) {
      bool result = _map->addFactor(f);
      if (!result)
        return result;
      ICPStats empty{};
      pushEvent(std::make_shared<EventFactorAdded>(0,
                                                   f->from_ref,
                                                   f->to_ref,
                                                   IsometryType::Identity(),
                                                   PoseHessianType::Identity(),
                                                   empty,
                                                   f->type,
                                                   (size_t)f.get()));
      return result;
    }

    template <typename T_>
    bool BundlerProc_<T_>::addFactor(PGOFactorPtr f) {
      bool result = _map->addFactor(f);
      if (!result)
        return result;
      pushEvent(std::make_shared<EventFactorAdded>(0,
                                                   f->from_ref,
                                                   f->to_ref,
                                                   f->Z_from_to,
                                                   f->omega_from_to,
                                                   f->stats,
                                                   f->type,
                                                   (size_t)f.get()));
      return result;
    }

  } // namespace slam
} // namespace kd_slam
