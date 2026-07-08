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
    void BundlerProc_<T_>::bundleCT() {
      static constexpr int MAX_FRAMES = MapType::MAX_FRAMES;

      if (_status != Ready) {
        std::cerr << "no bundling, not ready" << std::endl;
        return;
      }
      _status = CTICPBundling;
      saveFrames();

      for (auto [ref, fr_]: _map->frames()) {
        auto frame = std::dynamic_pointer_cast<Frame>(fr_);
        if (!frame) continue;
        if (frame->solver_variable->status() == srrg2_solver::VariableBase::Fixed) {
          frame->solver_velocity_prior->setEnabled(false);
          frame->solver_velocity->setStatus(srrg2_solver::VariableBase::Fixed);
        } else {
          frame->solver_velocity_prior->setEnabled(true);
          auto new_info = frame->solver_velocity_prior->informationMatrix().eval();
          new_info.setIdentity();
          new_info *= this->_optimizer_params.velocity_prior_info;
          frame->solver_velocity_prior->setInformationMatrix(new_info);
          frame->solver_velocity->setStatus(srrg2_solver::VariableBase::Active);
        }
      }

      auto ba_factors = seekBundleFactors();

      std::cerr << "adding kd_factors" << std::endl;
      int num_added = 0;
      for (auto [ip, force_valid]: ba_factors) {
        auto from_frame = std::dynamic_pointer_cast<Frame>(_map->frame(ip.from_ref));
        auto to_frame   = std::dynamic_pointer_cast<Frame>(_map->frame(ip.to_ref));
        MultiViewCTICPFactor fac;
        fac.from_ref             = ip.from_ref;
        fac.to_ref               = ip.to_ref;
        fac.force_valid          = force_valid;
        fac.is_enabled           = true;
        fac.cticp                = _multi_cticp_aligner.get();
        fac.vel_from_ref         = MAX_FRAMES + fac.from_ref;
        fac.vel_to_ref           = MAX_FRAMES + fac.to_ref;
        fac.tree_fixed           = from_frame->tree.get();
        fac.tree_moving          = to_frame->tree.get();
        fac.disable_inlier_ratio = force_valid ? 0 : _ba_params.ba_disable_inlier_ratio;
        addFactor(std::make_shared<MultiViewCTICPFactor>(fac));
        ++num_added;
      }
      std::cerr << "added " << num_added << " ba factors" << std::endl;
      optimize();

      for (auto [ref, frame_]: _map->frames()) {
        auto frame = std::dynamic_pointer_cast<Frame>(frame_);
        if (!frame || !frame->_temp_tree) continue;
        frame->_temp_tree->copyNodes(*frame->tree);
        frame->_temp_tree.reset();
        frame->solver_velocity_prior->setEnabled(false);
        frame->solver_velocity->setStatus(srrg2_solver::VariableBase::Fixed);
      }

      _status = Ready;
      pushEvent(std::make_shared<EventFrameProcessed>(Ready, 0.0, 0.0, 0.0, 0.0));
    }


    template <typename T_>
    void BundlerProc_<T_>::bundleCTVelocityOnly() {
      static constexpr int MAX_FRAMES = MapType::MAX_FRAMES;

      if (_status != Ready) {
        std::cerr << "no bundling, not ready" << std::endl;
        return;
      }
      _status = CTICPBundling;
      saveFrames();

      // fix all poses; activate all velocities regardless of pose status
      for (auto [ref, fr_]: _map->frames()) {
        auto frame = std::dynamic_pointer_cast<Frame>(fr_);
        if (!frame) continue;
        if (frame->solver_variable->status() != srrg2_solver::VariableBase::Fixed)
          frame->solver_variable->setStatus(srrg2_solver::VariableBase::Fixed);
        frame->solver_velocity_prior->setEnabled(true);
        auto new_info = frame->solver_velocity_prior->informationMatrix().eval();
        new_info.setIdentity();
        new_info *= this->_optimizer_params.velocity_prior_info;
        frame->solver_velocity_prior->setInformationMatrix(new_info);
        frame->solver_velocity->setStatus(srrg2_solver::VariableBase::Active);
      }

      auto ba_factors = seekBundleFactors();
      
      std::cerr << "adding kd_factors (velocity only)" << std::endl;
      int num_added = 0;
      for (auto [ip, force_valid]: ba_factors) {
        auto from_frame = std::dynamic_pointer_cast<Frame>(_map->frame(ip.from_ref));
        auto to_frame   = std::dynamic_pointer_cast<Frame>(_map->frame(ip.to_ref));
        MultiViewCTICPFactor fac;
        fac.from_ref             = ip.from_ref;
        fac.to_ref               = ip.to_ref;
        fac.force_valid          = force_valid;
        fac.is_enabled           = true;
        fac.cticp                = _multi_cticp_aligner.get();
        fac.vel_from_ref         = MAX_FRAMES + fac.from_ref;
        fac.vel_to_ref           = MAX_FRAMES + fac.to_ref;
        fac.tree_fixed           = from_frame->tree.get();
        fac.tree_moving          = to_frame->tree.get();
        fac.disable_inlier_ratio = force_valid ? 0 : _ba_params.ba_disable_inlier_ratio;
        addFactor(std::make_shared<MultiViewCTICPFactor>(fac));
        ++num_added;
      }
      std::cerr << "added " << num_added << " ba factors" << std::endl;
      optimize();

      for (auto [ref, frame_]: _map->frames()) {
        auto frame = std::dynamic_pointer_cast<Frame>(frame_);
        if (!frame || !frame->_temp_tree) continue;
        frame->_temp_tree->copyNodes(*frame->tree);
        frame->_temp_tree.reset();
        frame->solver_velocity_prior->setEnabled(false);
        frame->solver_velocity->setStatus(srrg2_solver::VariableBase::Fixed);
      }

      _status = Ready;
      pushEvent(std::make_shared<EventFrameProcessed>(Ready, 0.0, 0.0, 0.0, 0.0));
    }

  } // namespace slam
} // namespace kd_slam
