#pragma once
#include "cuda/cuda_common.h"
#include <iostream>
#include "srrg_config/configurable.h"

namespace kd_slam {
  namespace icp {

    template <typename Scalar_>
    struct ICPStats_ {
      using Scalar = Scalar_;
      int    num_inliers    = 0;
      int    num_outliers   = 0;
      int    num_bad       = 0;
      int    num_err_normal= 0;
      int    num_err_fn    = 0;
      Scalar chi2             = 0;
      Scalar chi2_kernelized  = 0;
      Scalar pert_pose_norm        = 0;
      Scalar pert_vel_norm        = 0;

      __CUDA_EXPORT_INLINE__
      int  numLeaves() const {
        return num_inliers+num_outliers+num_bad+num_err_normal+num_err_fn;
      }

      __CUDA_EXPORT_INLINE__
      Scalar inlierRatio() const {
        int num_leaves=numLeaves();
        return num_leaves?(Scalar)num_inliers/(Scalar)num_leaves:0;
      }
      __CUDA_EXPORT_INLINE__
      Scalar score() const {
        return inlierRatio() / (chi2_kernelized / num_inliers + 1e-6f);
      }
      
      __CUDA_EXPORT_INLINE__ void clear() {
        num_inliers=0; num_outliers=0; num_bad=0; num_err_normal=0; num_err_fn=0;
        chi2=0; chi2_kernelized=0;
      }

      __CUDA_EXPORT_INLINE__
      ICPStats_& operator+=(const ICPStats_& o) {
        num_inliers+=o.num_inliers; num_outliers+=o.num_outliers; num_bad+=o.num_bad;
        num_err_normal+=o.num_err_normal; num_err_fn+=o.num_err_fn;
        chi2+=o.chi2; chi2_kernelized+=o.chi2_kernelized;
        return *this;
      }

      __CUDA_EXPORT_INLINE__
      ICPStats_ operator+(const ICPStats_& o) const {
        auto r=*this; r+=o; return r;
      }

      static ICPStats_ Zero() { return ICPStats_(); }

      __CUDA_EXPORT_INLINE__ void pack(const Scalar& chi2_, const Scalar& gamma) {
        chi2=chi2_; chi2_kernelized=chi2_*gamma;
      }
    };

    template <typename Scalar_>
    std::ostream& operator << (std::ostream& os, const ICPStats_<Scalar_>& s) {
      using namespace std;
      os << "k2: " << s.chi2 
         << " kk: " << s.chi2_kernelized 
         << " ni: " << s.num_inliers
         << " no: " << s.num_outliers
         << " nb: " << s.num_bad
         << " nn: " << s.num_err_normal
         << " nf: " << s.num_err_fn
         << " pp: " << s.pert_pose_norm
         << " pv: " << s.pert_vel_norm;
      return os;
    }

    template <typename Scalar_>
    struct ICPTerminationCriteriaBase_: public srrg2_core::Configurable {
      using Scalar= Scalar_;
      using Stats = ICPStats_<Scalar_>;
      virtual bool hasToStop(Stats& stats, int iteration) = 0;
    };

    template <typename Scalar_>
    struct ICPTerminationCriteriaScore_: public ICPTerminationCriteriaBase_<Scalar_> {
      using Scalar= Scalar_;
      using Stats = ICPStats_<Scalar_>;
      Scalar prev_score=0;
      PARAM(srrg2_core::PropertyFloat, d_pert, "perutbation_change, smaller than that terminates",       1e-3,   nullptr);
      PARAM(srrg2_core::PropertyFloat, d_score, "score_change between iterations, smaller than that terminates",       1e-3,   nullptr);
      virtual bool hasToStop(Stats& s, int it) override {
        if (it>0) {
          bool pert_ok=s.pert_pose_norm < param_d_pert.value();
          Scalar d_score= s.score()-prev_score;
          bool score_ok = (d_score>0) && (d_score)/(prev_score + 1e-6) < param_d_score.value();
          if (pert_ok && score_ok)
            return true;
        }
        prev_score=s.score();
        return false;
      }
    };
  }
}
