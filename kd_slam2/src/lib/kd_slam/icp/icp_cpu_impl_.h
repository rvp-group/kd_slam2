#pragma once
#include <Eigen/Cholesky>
#include <iterator>
#include <iostream>
#include <future>
#include "icp_cpu_.h"
#include "utils/stable_adder_.h"
#include "icp_impl_.h"

namespace kd_slam {
  namespace icp {

    using namespace std;
    using namespace kd_slam::utils;

    template <typename Base_>
    void ICP_CPU_<Base_>::setMoving(const TreeBaseType& moving) {
      this->_moving = &moving;
    }

    template <typename Base_>
    void ICP_CPU_<Base_>::setFixed(const TreeBaseType& fixed) {
      this->_fixed = &fixed;
    }


    template <typename Base_>
    void ICP_CPU_<Base_>::_buildQuadraticForm(bool stats_mode) {
      static constexpr int MAX_THREADS=8;
      struct BQFReturn {
        HessianType H;
        CoefficientType b;
        StatsType stats;
      };

      auto bqf =[&] (size_t i_min, size_t i_max) -> BQFReturn {
        using DestView     = typename Base_::DestView;
        using StatsType    = typename Base_::StatsType;
        using DiagPoseType = typename Base_::DiagPoseType;

        StatsType    stats_buf;
        DiagPoseType diag_pose_buf;

        StableAdder_<StatsType>    stats_adder;
        StableAdder_<DiagPoseType> diag_pose_adder;
        stats_adder.reset();
        diag_pose_adder.reset();

        DestView view{&stats_buf, &diag_pose_buf};

        i_min=std::max(size_t(0),i_min);
        i_max=std::min(this->_moving->_num_leaves, i_max);
        
        for (size_t i=i_min; i<i_max; ++i) {
          const auto& moving_leaf=this->_moving->_nodes_ptr[this->_moving->_leaves_indices_ptr[i]];
          Base::quadraticTerm(view, 0, this->fixed_state, this->moving_state, this->_cache,
                              this->_fixed->_nodes_ptr, moving_leaf, this->params, stats_mode);
          stats_adder.add(stats_buf);
          diag_pose_adder.add(diag_pose_buf);
        }

        BQFReturn ret;
        diag_pose_buf=diag_pose_adder.sum();
        diag_pose_buf.unpack(ret.H, ret.b);
        ret.stats = stats_adder.sum();
        return ret;
      };

      int nt=std::min(param_num_threads.value(), MAX_THREADS);
      
      if (nt<=1) {
        BQFReturn ret=bqf(0, this->_moving->_num_leaves);
        this->stats=ret.stats;
        this->H=ret.H;
        this->b=ret.b;
      } else  {
        std::future<BQFReturn> thr[MAX_THREADS];

        size_t n = this->_moving->_num_leaves;
        size_t chunk = n / nt;
        size_t rem   = n % nt;
        size_t start = 0;
        for (int i = 0; i < nt; ++i) {
          size_t end = start + chunk + (i < (int)rem ? 1 : 0);
          thr[i] = std::async(std::launch::async, bqf, start, end);
          start = end;
        }
        this->H.setZero();
        this->b.setZero();
        this->stats.clear();
        for (int i=0; i<nt; ++i) {
          auto ret=thr[i].get();
          this->H+=ret.H;
          this->b+=ret.b;
          this->stats+=ret.stats;
        }
      }
    }

  }
} // namespace kd_slam::icp
