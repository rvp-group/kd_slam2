#pragma once
#include <srrg_solver/solver_core/solver_action_base.h>
#include "kd_slam/map/map_common.h"

namespace kd_slam {
  namespace slam {

    template <typename T_>
    struct OptimizerProc_;

    template <typename OptimizerProc_>
    struct OptimizerSolverAction_ : public srrg2_solver::SolverActionBase {
      OptimizerProc_* optimizer = nullptr;

      OptimizerSolverAction_() {
        param_event.setValue(srrg2_solver::Solver::SolverEvent::IterationEnd);
      }

      void doAction() override {
        if (!optimizer) return;
        std::ostringstream os;
        os << _solver_ptr->lastIterationStats();
        auto status = optimizer->status();
        if (status == ICPBundling || status == CTICPBundling) {
          optimizer->sync();
          optimizer->pushOptimizeEvent(optimizer->currentStamp(), os.str());
          optimizer->pushFactorUpdateEvents();
          optimizer->pushFrameProcessedEvent(optimizer->currentStamp(), status);
        } else {
          optimizer->pushOptimizeEvent(optimizer->currentStamp(), os.str());
        }
        if (status != CTICPBundling)
          return;
        optimizer->pushFrameEvents();
      }
    };

    template <typename OptimizerProc_>
    struct OptimizerPreDeskewAction_ : public srrg2_solver::SolverActionBase {
      OptimizerProc_* optimizer = nullptr;

      OptimizerPreDeskewAction_() {
        param_event.setValue(srrg2_solver::Solver::SolverEvent::IterationStart);
      }

      void doAction() override {
        if (!optimizer) return;
        if (optimizer->status() != CTICPBundling)
          return;
        for (auto [ref, frame_] : optimizer->_map->frames()) {
          auto frame = std::dynamic_pointer_cast<typename OptimizerProc_::Frame>(frame_);
          if (!frame) continue;
          if (!frame->_temp_tree)
            throw std::runtime_error("no frame to restore");
          frame->_temp_tree->copyNodes(*frame->tree);
          frame->tree->applyVelocitiesInPlace(frame->velocity, true);
        }
      }
    };

  } // namespace slam
} // namespace kd_slam
