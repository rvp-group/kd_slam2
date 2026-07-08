#pragma once
#include "kd_slam/map/multiview_icp_factor_.h"
#include "kd_slam/map/multiview_cticp_factor_.h"
#include "kd_slam/tree/count_leaf_policy_.h"
#include "kd_slam/tree/node_impl_.h"
#include "kd_slam/tree/tree_impl_.h"
#include "kd_slam/tree/tree_cpu_impl_.h"
#include "srrg_solver/solver_core/factor_base.h"
#include <unordered_map>
namespace kd_slam {
  namespace slam {
    using namespace kd_slam::frame;

    template <typename T_>
    void BundlerProc_<T_>::bundle() {
      if (_status != Ready) {
        std::cerr << "no bundling, not ready" << std::endl;
        return;
      }
      _status = ICPBundling;
      saveFrames();
      applyKFVelocities();
      pushFrameEvents();

      for (auto [ref, fr_]: _map->frames()) {
        auto fr = std::dynamic_pointer_cast<Frame>(fr_);
        if (!fr) continue;
        if (fr->solver_velocity_prior)
          fr->solver_velocity_prior->setEnabled(false);
        if (fr->solver_velocity)
          fr->solver_velocity->setStatus(srrg2_solver::VariableBase::Fixed);
      }

      auto ba_factors = seekBundleFactors();

      std::cerr << "adding kd_factors" << std::endl;
      int num_added = 0;
      for (auto [ip, force_valid]: ba_factors) {
        auto from_frame = std::dynamic_pointer_cast<Frame>(_map->frame(ip.from_ref));
        auto to_frame   = std::dynamic_pointer_cast<Frame>(_map->frame(ip.to_ref));
        MultiViewICPFactor fac;
        fac.from_ref             = ip.from_ref;
        fac.to_ref               = ip.to_ref;
        fac.force_valid          = force_valid;
        fac.is_enabled           = true;
        fac.icp                  = _multi_icp_aligner.get();
        fac.tree_fixed           = from_frame->tree.get();
        fac.tree_moving          = to_frame->tree.get();
        fac.disable_inlier_ratio = force_valid ? 0 : _ba_params.ba_disable_inlier_ratio;
        addFactor(std::make_shared<MultiViewICPFactor>(fac));
        ++num_added;
      }
      std::cerr << "added " << num_added << " ba factors" << std::endl;
      optimize();
      restoreFrames();
      _status = Ready;
      pushEvent(std::make_shared<EventFrameProcessed>(Ready, 0.0, 0.0, 0.0, 0.0));
    }


  } // namespace slam
} // namespace kd_slam
