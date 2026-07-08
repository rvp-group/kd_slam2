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
    void BundlerProc_<T_>::cure() {
      if (_status != Ready) {
        std::cerr << "no doctor, not ready" << std::endl;
        return;
      }
      _status = MapDoctor;
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
      pruneBundleFactors(ba_factors);
      // enable all pgo factors
      for (auto fac_: _map->factors()) {
        auto pose_fac = std::dynamic_pointer_cast<PGOFactor>(fac_);
        if (pose_fac) {
          pose_fac->is_enabled = true;
          pose_fac->solver_factor->setEnabled(true);
        }
      }
      int num_factors_old=this->factorGraph()->factors().size();
      std::list<Match> matches;
      for (auto [ip, keep]: ba_factors) {
        if (keep)
          continue;
        auto from_frame = std::dynamic_pointer_cast<Frame>(_map->frame(ip.from_ref));
        if (! from_frame)
          continue;
        
        auto to_frame   = std::dynamic_pointer_cast<Frame>(_map->frame(ip.to_ref));
        if (! to_frame)
          continue;
        
        Match m(from_frame, to_frame);
        m.pose.setIdentity();
        m.pose.linear()=from_frame->pose_in_world.linear().transpose()
          *to_frame->pose_in_world.linear();
        m.rematch(*_multi_icp_aligner, _ba_params.cure_thresholds, Loop);
        if (m.result==MatchOk) {
          //cerr << "MatchOk!" << ip.from_ref << "->" << ip.to_ref << endl;
          matches.push_back(m);
        } else  {
          //cerr << MatchLabelResultStr[m.result] << ip.from_ref << "->" << ip.to_ref << endl;
        }
      }
      
      for (auto& match: matches) {
        match.rematch(*_multi_icp_aligner, _ba_params.cure_thresholds, Loop, true);
        PGOFactor f;
        f.from_ref      = match.fixed_frame->ref();
        f.to_ref        = match.moving_frame->ref();
        f.Z_from_to     = match.pose;
        f.Z_to_from     = f.Z_from_to.inverse();
        f.omega_from_to = match.omega;
        f.omega_to_from = GeometryTraits::transformOmega(f.omega_from_to, f.Z_from_to);
        f.det_from_to   = f.omega_from_to.determinant();
        f.det_to_from   = f.omega_to_from.determinant();
        f.stats         = match.stats;
        f.type          = Loop;
        PGOFactorPtr f_ptr= std::make_shared<PGOFactor>(f);
        bool result = addFactor(f_ptr);
        if (! result) {
          cerr << "error, factor already in map" << endl;
        } else {
          cerr << "add ok| " 
               << " from: " << f_ptr->from_ref 
               << " to: " << f_ptr->to_ref
               << " det: " << f_ptr->det_from_to
               << " Z: "  << GeometryTraits::logmap(f_ptr->Z_from_to).transpose() << endl;
          //cerr << "Omega: " << endl << f_ptr->omega_from_to << endl;
        }
        this->_need_optimize = true;
      }
      cerr<< "added : "<< matches.size() << " new factors" << endl;
      
      cerr << "press a key to continue" << endl;
      int num_factors_new = this->factorGraph()->factors().size();

      cerr << "factors before: " << num_factors_old << " after: " << num_factors_new << endl;
      optimize();
      this->pushOptimizeEvent(0.f);
      //store the chi2 of the graph, and compute the average chi2 in the factors nearby the connected nodes. 
      // for each factor try a match zeroing the translation
      //    if match bad,
      //       drop and continue  
      //    else
      //       backup the map (the graph lets you do that with push)
      //       optimize
      //       check the chi2 distribution.
      //       If the new entry added some excessive tension
      //          disable the factor, and restore the estimates
      //       else
      //          leave the factor and keep the estimate
      restoreFrames();
      _status = Ready;
      pushEvent(std::make_shared<EventFrameProcessed>(Ready, 0.0, 0.0, 0.0, 0.0));
    }
  }
}
