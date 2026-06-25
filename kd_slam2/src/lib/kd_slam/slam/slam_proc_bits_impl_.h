#pragma once
namespace kd_slam {
  namespace slam {
    using namespace kd_slam::frame;
    using namespace std;

    template <typename T_>
    bool SLAMProc_<T_>::canCompute() const {
      return BaseType::canCompute() && _local_aligner != nullptr;
    }

    template <typename T_>
    bool SLAMProc_<T_>::checkCompute() const {
      bool is_gpu = false, is_cpu = false;
      auto check = [&](const auto& a) {
        if (a) { is_gpu |= a->isGPU(); is_cpu |= !a->isGPU(); }
      };
      check(_odom_aligner);
      check(_local_aligner);
      check(_loop_aligner);
      return !(is_cpu && is_gpu);
    }

    template <typename T_>
    bool SLAMProc_<T_>::isGPU() const {
      if (_local_aligner)
        return _local_aligner->isGPU();
      return BaseType::isGPU();
    }

    template <typename T_>
    void SLAMProc_<T_>::reset() {
      BaseType::reset();
      stat_loop_spatial = 0;
      stat_loop_desc2floating = 0;
      stat_loop_desc2kf = 0;
      stat_loop_ICP_tries = 0;
      stat_loop_ICP_OK = 0;
    }

    template <typename T_>
    void SLAMProc_<T_>::syncParams() {
      if (!_param_changed)
        return;
      BaseType::syncParams();
      _slam_params.relocalize_thresholds = {param_relocalize_max_chi2.value(),
                                            param_relocalize_min_inliers_ratio.value(),
                                            param_relocalize_min_score.value(),
                                            -1,
                                            param_relocalize_max_hops.value(),
                                            param_relocalize_slack.value()};
      _slam_params.loop_thresholds       = {param_loop_max_chi2.value(),
                                            param_loop_min_inliers_ratio.value(),
                                            param_loop_min_score.value(),
                                            param_loop_min_hops.value(),
                                            -1,
                                            param_loop_slack.value()};
      _slam_params.loop_max_orientation_rad             = param_loop_max_orientation_deg.value() * float(M_PI / 180.);
      _slam_params.loop_consensus_max_orientation_rad   = param_loop_consensus_max_orientation_deg.value() * float(M_PI / 180.);
      _slam_params.loop_consensus_max_translation       = param_loop_consensus_max_translation.value();
      _slam_params.covariance_propagate_trans_cond_diag = param_covariance_propagate_trans_cond_diag.value();
      _slam_params.covariance_propagate_rot_cond_diag   = param_covariance_propagate_rot_cond_diag.value();
      _local_aligner = param_local_aligner.value();
      _loop_aligner  = param_loop_aligner.value();

    }

    template <typename T_>
    void SLAMProc_<T_>::filterSpatially(std::unordered_map<int, Match>& candidates,
                                       std::vector<int>& allowed_refs,
                                       const MatchThresholds& thresholds) {
      candidates.clear();
      allowed_refs.reserve(_map->frames().size());
      for (auto& [ref, frame_base] : _map->frames()) {
        auto frame = static_pointer_cast<Frame>(frame_base);
        if (frame == _keyframe)
          continue;
        Match m(_floating_frame, frame);
        m.initFromGraph();
        m.filter(thresholds, _odom_aligner->getPoseHessian().inverse());
        if (m.result != MatchOk)
          continue;
        candidates[ref] = m;
        allowed_refs.push_back(ref);
      }
    }

    template <typename T_>
    void SLAMProc_<T_>::addGravityPrior(const Eigen::Vector3f& gravity_dir, float info) {
      if constexpr (NodeType::Dim == 3) {
        if (!_keyframe) return;
        const Eigen::Matrix<Scalar, 3, 1> g_slam = gravity_dir.template cast<Scalar>();
        auto gprior = std::make_shared<SolverGravityPriorFactorType>();
        gprior->setVariableId(0, _keyframe->ref());
        gprior->setMeasurement(g_slam);
        gprior->setInformationMatrix(
          SolverGravityPriorFactorType::InformationMatrixType::Identity() * info);
        factorGraph()->addFactor(gprior);
        if (info == 0.f) {
          gprior->setEnabled(false);
        } else {
          gprior->compute(true, true);
          _pending_chi2 += gprior->stats().chi;
        }
        _keyframe->solver_gravity_prior = gprior.get();
        _need_optimize = true;
      }
    }
    
    template <typename T_>
    void SLAMProc_<T_>::printLoopStats(std::ostream& os) {
      os << "LOOP_STATS| "
         << " spatial: " << stat_loop_spatial
         << " desc_floating: " << stat_loop_desc2floating
         << " desc_kf: " << stat_loop_desc2kf
         << " desc_ICP: " << stat_loop_ICP_tries
         << " desc_ICP_OK: " << stat_loop_ICP_OK
         << endl;
    }

  } // namespace slam
} // namespace kd_slam
