#pragma once
#include <srrg_config/property_configurable.h>

#include <srrg_solver/solver_core/solver.h>
#include <srrg_solver/solver_core/factor_graph.h>
#include "kd_slam/map/map_owner_.h"
#include "optimizer_proc_solver_action_.h"

namespace kd_slam {
  namespace slam {
    using namespace kd_slam::map;
    using namespace kd_slam::event;

    template <typename T_>
    struct OptimizerProc_ : public T_ {

      using TrackerType  = T_;
      using MapType      = typename T_::MapType;
      using NodeType     = typename T_::NodeType;
      using Scalar       = typename T_::Scalar;
      using IsometryType = typename T_::IsometryType;
      using Frame        = typename T_::Frame;
      using FramePtr     = typename T_::FramePtr;

      using typename T_::EventFrameProcessed;

      // solver / factor types from MapType
      using SolverVariableType            = typename MapType::SolverVariableType;
      using SolverPGOFactorType           = typename MapType::SolverPGOFactorType;
      using SolverVelocityVariableType    = typename MapType::SolverVelocityVariableType;
      using SolverVelocityPriorFactorType = typename MapType::SolverVelocityPriorFactorType;
      using SolverGravityPriorFactorType  = typename MapType::SolverGravityPriorFactorType;
      using PGOFactor                     = typename MapType::PGOFactor;
      using PGOFactorPtr                  = typename MapType::PGOFactorPtr;
      using MultiViewICPFactor            = typename MapType::MultiViewICPFactor;
      using MultiViewICPFactorPtr         = typename MapType::MultiViewICPFactorPtr;
      using MultiViewCTICPFactor          = typename MapType::MultiViewCTICPFactor;
      using MultiViewCTICPFactorPtr       = typename MapType::MultiViewCTICPFactorPtr;
      using SolverFactorBridge            = typename MapType::SolverFactorBridge;
      using SolverFactorBridgePtr         = std::shared_ptr<SolverFactorBridge>;
      using SolverVelocityPriorFactor     = VelocityPriorFactor_<SolverVelocityVariableType>;

      // dependent base members
      using T_::_map;
      using T_::pushEvent;
      using T_::pushOptimizeEvent;
      using T_::pushFrameEvents;
      using T_::pushFactorEvents;
      using T_::pushFactorUpdateEvents;
      using T_::pushFrameProcessedEvent;
      using T_::applyKFVelocities;
      using T_::factorGraph;
      using T_::_param_changed;
      using T_::_status;
      using T_::status;
      using T_::currentStamp;

      friend struct OptimizerSolverAction_<OptimizerProc_<T_>>;
      friend struct OptimizerPreDeskewAction_<OptimizerProc_<T_>>;

      OptimizerProc_() {
        _solver_action = std::make_shared<OptimizerSolverAction_<OptimizerProc_<T_>>>();
        _solver_action->optimizer = this;
        _solver_action_predeskew  = std::make_shared<OptimizerPreDeskewAction_<OptimizerProc_<T_>>>();
        _solver_action_predeskew->optimizer = this;
      }

      void optimize();
      void sync();
      void setMap(std::shared_ptr<MapType> m) override;
      void reset()      override;
      void syncParams() override;

      srrg2_solver::Solver* solverPtr() { return _solver.get(); }

      OptimizerParams _optimizer_params;

      PARAM(srrg2_core::PropertyConfigurable_<srrg2_solver::Solver>, solver,
            "PGO solver", nullptr, &_param_changed);
      PARAM(srrg2_core::PropertyFloat, pgo_chi2_threshold,
            "when to trigger pgo",             0.1f,  &_param_changed);
      PARAM(srrg2_core::PropertyFloat, velocity_prior_info,
            "prior for velocity from cticp",   1000,  &_param_changed);
      PARAM(srrg2_core::PropertyFloat, imu_gravity_prior_info,
            "prior for gravity info from imu", 1000,  &_param_changed);

    protected:
      using SolverActionType    = OptimizerSolverAction_<OptimizerProc_<T_>>;
      using PreDeskewActionType = OptimizerPreDeskewAction_<OptimizerProc_<T_>>;

      std::shared_ptr<srrg2_solver::Solver> _solver               = nullptr;
      std::shared_ptr<SolverActionType>     _solver_action;
      std::shared_ptr<PreDeskewActionType>  _solver_action_predeskew;
      bool   _need_optimize = false;
      Scalar _pending_chi2  = 0;
    };

  } // namespace slam
} // namespace kd_slam
