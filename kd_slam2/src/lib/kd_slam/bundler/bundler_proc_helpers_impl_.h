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

        // propagates the covariances from current_keyframe
    template <typename T_>
    void BundlerProc_<T_>::propagateDistance(FramePtr root_frame, double max_distance) {
      struct DistancePropagateActions: public FrameGraph::VisitActions {
        bool _stop_now=false;
        DistancePropagateActions(double md):
          max_distance(md){}
        
        double traversalCost(FrameBase& src_, FrameBase& dest_, FactorBase& factor_) override {
          if (! factor_.is_valid)
            return std::numeric_limits<double>::max();
          // skip non-pose-pose factors
          auto* pf = dynamic_cast<PGOFactor*>(&factor_);
          if (!pf) return std::numeric_limits<double>::max();
          
          Frame& src         = static_cast<Frame&>(src_);
          Frame& dest        = static_cast<Frame&>(dest_);
          // metric distance: walking distance on graph in meters
          IsometryType delta = src.pose_in_world.inverse()*dest.pose_in_world;
          return delta.translation().norm();
        }

        void traverseFactor(FrameBase& src_, FrameBase& dest_, FactorBase& factor_) override {
          auto* pf = dynamic_cast<PGOFactor*>(&factor_);
          if (!pf) return; // skip non-pose-pose factors
          Frame& src         = static_cast<Frame&>(src_);
          if (src.distance_from_root>max_distance) {
            _stop_now=true;
          }
          Frame& dest        = static_cast<Frame&>(dest_);
          // metric distance: walking distance on graph in meters
          IsometryType delta = src.pose_in_world.inverse()*dest.pose_in_world;
          dest.distance_from_root  = src.distance_from_root + delta.translation().norm()+1e-3;
          dest.hops_from_root       = src.hops_from_root + 1;
        }

        bool stop() override {
          return _stop_now;
        }

        double max_distance;
      };

      if (!root_frame) return;
      for (auto [ref, fr_]: _map->frames()) {
        auto frame = std::dynamic_pointer_cast<Frame>(fr_);
        if (!frame) continue;
        frame->distance_from_root=std::numeric_limits<float>::max();
        frame->hops_from_root=0;
      }
      
      DistancePropagateActions dist_prop(max_distance);
      root_frame->distance_from_root = 0;
      _map->visitBFS(root_frame->ref(), dist_prop);
    }

    template <typename T_>
    void BundlerProc_<T_>::pruneBundleFactors(std::unordered_map<IndexPair, bool, IndexPairHash>& candidates){
      using OrderedMap=std::map<IndexPair, bool>;
      OrderedMap omap(candidates.begin(), candidates.end());
      candidates.clear();
      int prev_ref=-1;
      for (auto [ip,keep]: omap) {
        if (prev_ref!=ip.from_ref) {
          auto root = std::dynamic_pointer_cast<Frame>(_map->frame(ip.from_ref));
          if (!root)
            continue;
          propagateDistance(root, _ba_params.ba_range);
        }
        prev_ref=ip.from_ref;
        auto f_=_map->frame(ip.to_ref);
        auto f = std::dynamic_pointer_cast<Frame>(f_);
        if (!f)
          continue;
        if (keep || f->distance_from_root > _ba_params.ba_range) {
          candidates[ip]=keep;
        }
      }
    }
    
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


  } // namespace slam
} // namespace kd_slam
