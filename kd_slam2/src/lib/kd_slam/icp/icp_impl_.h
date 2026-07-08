#pragma once
#include <Eigen/Cholesky>
#include <iterator>
#include <iostream>
#include <sstream>
#include "icp_.h"
#include "utils/geometry_.h"

namespace kd_slam {
  namespace icp {

    template <typename Traits_>
    ICP_<Traits_>::~ICP_(){}

    template <typename Traits_>
    void ICP_<Traits_>::updateCache() {
      _cache.Xm=moving_state.X;
      _cache.Xf=fixed_state.X;
      _cache.iXf=_cache.Xf.inverse();
      _cache.Xmf=_cache.iXf*_cache.Xm;
    }

    template <typename Traits_>
    void ICP_<Traits_>::buildQuadraticForm(bool stats_mode) {
      updateCache();
      _buildQuadraticForm(stats_mode);
    }

    
    template <typename Base_>
    void ICP_<Base_>::buildQuadraticForm(FixedEntryBase& fixed_, bool stats_mode) {
      FixedEntry& fixed=static_cast<FixedEntry&>(fixed_);
      this->_fixed=fixed.fixed_tree;
      fixed_state=fixed.fixed_state;
      updateCache();
      _buildQuadraticForm(stats_mode);
      fixed.stats=this->stats;
      fixed.H=H;
      fixed.b=b;
    }

    
    template <typename Traits_>
    void ICP_<Traits_>::buildQuadraticFormDual(bool stats_mode) {
      buildQuadraticForm(stats_mode);
      // J_B = -J_A  =>  H_BB = H_AA, b_B = -b_A, J_A^T J_B = -H_AA
      H_fixed =  H;
      b_fixed = -b;
      H_cross = -H;
    }

    template <typename Traits_>
    bool ICP_<Traits_>::oneRound(bool stats_mode) {
      using namespace std;
      if (_fixed_forest.empty()) {
        buildQuadraticForm(stats_mode);
      } else {
        HessianType temp_H=HessianType::Zero();
        CoefficientType temp_b=CoefficientType::Zero();
        StatsType temp_stats=StatsType::Zero();
        for(auto [ref, e]: _fixed_forest) {
          FixedEntry& entry = static_cast<FixedEntry&>(*e);
          buildQuadraticForm(entry, stats_mode);
          temp_H+=entry.H;
          temp_b+=entry.b;
          temp_stats+=entry.stats;
        }
        H=temp_H;
        b=temp_b;
        this->stats=temp_stats;
      }
      AlignerBaseType::posePriorBuildQuadraticForm();
      H+=AlignerBaseType::_pose_prior_H;
      b+=AlignerBaseType::_pose_prior_b;
      // if not enough mesaurements complain
      int good_measurements=this->stats.num_outliers+this->stats.num_inliers;
      if (good_measurements<this->params.min_good_measurements) {
        //cerr << "good: " << good_measurements << endl;
        return false;
      }
      
      dx=(H+HessianType::Identity()*this->params.damping).ldlt().solve(- b);
      this->stats.pert_pose_norm=dx.norm();
      if (stats_mode)
        return true;

      // apply perturbation
      moving_state.X=Traits::expmap(dx)*moving_state.X;
      return true;
    }

    template <typename Traits_>
    std::ostream& ICP_<Traits_>::printStatus(std::ostream& os) {
      using namespace std;
      os << this->stats << endl;
      os << "H: " << endl << H << endl;
      os << "b: " << b.transpose() << endl;
      os << "dx: " << dx.transpose() << endl;
      return os;
    }

    template <typename Traits_>
    void ICP_<Traits_>::quadraticTerm(typename ICP_<Traits_>::DestView& dest,
                                      int tid,
                                      const ICP_<Traits_>::State& fixed_state,
                                      const ICP_<Traits_>::State& moving_state,
                                      const ICP_<Traits_>::QuadraticTermCache& cache,
                                      const ICP_<Traits_>::NodeType* fixed_nodes_ptr,
                                      const ICP_<Traits_>::NodeType& moving_leaf,
                                      const ICP_<Traits_>::ParamsType& params,
                                      bool stats_mode) {
      dest.clear(tid);
      const auto p_mif=applyIsometry_(cache.Xmf, moving_leaf._mean);
      //const auto n_mif=applyRotation_ (cache.Xmf, moving_leaf._direction);

      int aux_idx=0;
      while(aux_idx>=0 && !fixed_nodes_ptr[aux_idx].isLeaf()) {
        const auto& aux=fixed_nodes_ptr[aux_idx];
        aux_idx = aux.whichSide(p_mif) ? aux._idx_left : aux._idx_right;
      }
      if (aux_idx<0) { dest.stats[tid].num_bad=1; return; }

      const auto& pf=fixed_nodes_ptr[aux_idx]._mean;
      const auto& nf=fixed_nodes_ptr[aux_idx]._direction;

      const auto p_moving=applyIsometry_(cache.Xm, moving_leaf._mean);
      const auto n_moving=applyRotation_ (cache.Xm, moving_leaf._direction);

      const auto p_fixed=applyIsometry_(cache.Xf, pf);
      const auto n_fixed=applyRotation_ (cache.Xf, nf);

      typename Traits_::Scalar e;
      typename Traits_::JacobianPoseType J;
      int result=Traits_::errorAndJacobian(e, J, p_fixed, n_fixed, p_moving, n_moving,
                                           params.min_normal_cos, params.max_error);
      switch(result) {
      case -1: dest.stats[tid].num_err_normal=1; return;
      case -2: dest.stats[tid].num_err_fn=1;     return;
      default:;
      }

      Scalar chi2=e*e;
      Scalar gamma = 1;
      if (chi2>params.kernel_threshold) {
        dest.stats[tid].num_outliers=1;
        gamma = stats_mode ? 0.f : sqrt(params.kernel_threshold/chi2);
      } else {
        dest.stats[tid].num_inliers=1;
      }
      dest.stats[tid].pack(chi2, gamma);
      dest.diag_pose[tid].pack(e, J, gamma);
    }

    template <typename Traits_>
    std::string  ICP_<Traits_>::iterationLogHeader() const {
      using namespace std;
      std::ostringstream os;
      os <<"ICP" << Traits::Dim << "D| START "
         << " fp: " << Traits_::GeometryTraits::logmap(_saved_start_fixed_state.X).transpose()
         << " fv: none"
         << " mp: " << Traits_::GeometryTraits::logmap(_saved_start_moving_state.X).transpose()
         << " mv: none"
         << " | END "
         << " fp: " << Traits_::GeometryTraits::logmap(fixed_state.X).transpose()
         << " fv: none"
         << " mp: " << Traits_::GeometryTraits::logmap(moving_state.X).transpose()
         << " mv: none";
        return os.str();
    }

  }
} // namespace kd_slam::icp
