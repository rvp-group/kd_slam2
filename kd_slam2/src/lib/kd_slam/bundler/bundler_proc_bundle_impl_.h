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
    std::unordered_map<IndexPair, bool, IndexPairHash>
    BundlerProc_<T_>::seekBundleFactors() {
      struct TrajectoryPoint:
                public PointWithEigenMember_<typename NodeType::VectorType> {
        int ref=-1;
        int count=-1;
        int degree=0;
      };
      using TrajectoryPointTraits = PointTraits_<TrajectoryPoint>;
      using TrajectoryNode        = kd_slam::Node_<TrajectoryPointTraits>;
      using TrajectoryTree        = TreeCPU_<Tree_<TrajectoryNode>>;

      std::cerr << "cleaning graph" << std::endl;
      std::unordered_map<IndexPair, bool, IndexPairHash> ba_factors;
      std::list<PGOFactorPtr> pose_factors;
      std::list<SolverFactorBridgePtr> erased_factors;
      for (auto fac_: _map->factors()) {
        auto pose_fac = std::dynamic_pointer_cast<PGOFactor>(fac_);
        if (pose_fac) {
          pose_fac->is_enabled = false;
          pose_fac->solver_factor->setEnabled(false);
          pose_factors.push_back(pose_fac);
          IndexPair ip{pose_fac->from_ref, pose_fac->to_ref};
          if (ip.from_ref > ip.to_ref)
            std::swap(ip.from_ref, ip.to_ref);
          ba_factors[ip] = true;
          continue;
        }
        auto micp_fac = std::dynamic_pointer_cast<MultiViewICPFactor>(fac_);
        if (micp_fac) {
          erased_factors.push_back(micp_fac);
        }
        auto cticp_fac = std::dynamic_pointer_cast<MultiViewCTICPFactor>(fac_);
        if (cticp_fac) {
          erased_factors.push_back(cticp_fac);
        }
      }
      for (auto fac: erased_factors) {
        factorGraph()->removeFactor(fac->solver_factor);
        _map->removeFactor(fac);
      }
      std::cerr << "removed " << erased_factors.size() << " factors" << std::endl;

      std::cerr << "populating trajectory tree" << std::endl;
      std::vector<TrajectoryPoint> traj_points;
      traj_points.reserve(_map->frames().size());
      for (auto [ref, fr_]: _map->frames()) {
        auto fr = std::dynamic_pointer_cast<Frame>(fr_);
        if (!fr) continue;
        TrajectoryPoint p;
        p.coordinates = fr->pose_in_world.translation();
        p.ref   = ref;
        p.count = fr->frame_count;
        traj_points.push_back(p);
      }

      CountLeafPolicy_<TrajectoryNode> policy;
      policy.max_points = 20;
      TrajectoryTree traj_tree(traj_points, policy);

      std::cerr << "building neighbors" << std::endl;
      for (auto [ref, fr_]: _map->frames()) {
        auto fr = std::dynamic_pointer_cast<Frame>(fr_);
        if (!fr) continue;
        std::vector<size_t> neighbors;
        traj_tree.searchFull(neighbors, fr->pose_in_world.translation(), _ba_params.ba_range);
        for (auto idx: neighbors) {
          const TrajectoryPoint& p = traj_points[idx];
          if (p.ref >= ref) continue;
          IndexPair ip{ref, p.ref};
          if (ba_factors.count(ip)) continue;
          ba_factors[ip] = false;
        }
      }
      return ba_factors;
    }

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
