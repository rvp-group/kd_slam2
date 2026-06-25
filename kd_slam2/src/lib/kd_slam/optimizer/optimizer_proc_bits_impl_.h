#pragma once
namespace kd_slam {
  namespace slam {

    template <typename T_>
    void OptimizerProc_<T_>::reset() {
      T_::reset();
      _need_optimize = false;
      _pending_chi2  = 0;
    }

    template <typename T_>
    void OptimizerProc_<T_>::syncParams() {
      if (!_param_changed)
        return;
      T_::syncParams();
      _optimizer_params.pgo_chi2_threshold     = param_pgo_chi2_threshold.value();
      _optimizer_params.velocity_prior_info    = param_velocity_prior_info.value();
      _optimizer_params.imu_gravity_prior_info = param_imu_gravity_prior_info.value();
      if (param_solver.value() && param_solver.value() != _solver) {
        _solver = param_solver.value();
        _solver->param_actions.pushBack(_solver_action);
        _solver->param_actions.pushBack(_solver_action_predeskew);
      }
    }

    template <typename T_>
    void OptimizerProc_<T_>::sync() {
      for (auto& [ref, base_frame] : _map->frames()) {
        auto frame = std::dynamic_pointer_cast<Frame>(base_frame);
        if (!frame) continue;
        if (frame->solver_variable)
          frame->pose_in_world = frame->solver_variable->estimate();
        if (frame->solver_velocity)
          frame->velocity = frame->solver_velocity->estimate().template cast<Scalar>();
      }
    }

    template <typename T_>
    void OptimizerProc_<T_>::optimize() {
      if (!_solver) return;
      _solver->setGraph(factorGraph());
      _solver->compute();
      sync();
    }

    template <typename T_>
    void OptimizerProc_<T_>::setMap(std::shared_ptr<MapType> m) {
      T_::setMap(m);
      if constexpr (NodeType::Dim == 3) {
        for (auto [_, f_] : _map->frames()) {
          auto f = std::dynamic_pointer_cast<Frame>(f_);
          if (!f) continue;
          if (f->solver_gravity_prior) {
            f->solver_gravity_prior->setInformationMatrix(
              SolverGravityPriorFactorType::InformationMatrixType::Identity()
              * _optimizer_params.imu_gravity_prior_info);
          }
        }
      }
      pushFrameEvents();
      pushFactorEvents();
      pushFactorUpdateEvents();
      _status = Ready;
      pushEvent(std::make_shared<EventFrameProcessed>(Ready, 0.0, 0.0, 0.0, 0.0));
    }

  } // namespace slam
} // namespace kd_slam
