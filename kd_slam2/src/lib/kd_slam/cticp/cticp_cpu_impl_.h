#include <Eigen/Cholesky>
#include <iterator>
#include <iostream>
#include <future>
#include "cticp_cpu_.h"
#include "utils/stable_adder_.h"


namespace kd_slam {
  namespace cticp {
    using namespace kd_slam::icp;
    using namespace kd_slam::utils;
    
    using namespace std;

    template <typename Base_>
    void CTICP_CPU_<Base_>::setMoving(const TreeBaseType& moving) {
      this->_moving = &moving;
    }

    template <typename Base_>
    void CTICP_CPU_<Base_>::setFixed(const TreeBaseType& fixed) {
      this->_fixed = &fixed;
    }

    template <typename Base_>
    void CTICP_CPU_<Base_>::_buildQuadraticForm() {
      static constexpr int MAX_THREADS=8;

      using DestView     = typename Base_::DestView;
      using StatsType    = typename Base_::StatsType;
      using DiagPoseType = typename Base_::DiagPoseType;
      using DiagVelType  = typename Base_::DiagVelType;
      using CrossType    = typename Base_::CrossType;
      static constexpr int PDim = Base_::PerturbationPoseDim;
      static constexpr int VDim = Base_::VelocityDim;

      struct BQFReturn {
        StatsType stats;
        HessianType H;
        CoefficientType b;
        
      };

      auto bqf = [&](size_t imin, size_t imax) ->BQFReturn{
        StatsType    stats_buf;
        DiagPoseType diag_pose_buf;
        DiagVelType  diag_vel_buf;
        CrossType    cross_buf;

        StableAdder_<StatsType>    stats_adder;
        StableAdder_<DiagPoseType> diag_pose_adder;
        StableAdder_<DiagVelType>  diag_vel_adder;
        StableAdder_<CrossType>    cross_adder;
        stats_adder.reset();
        diag_pose_adder.reset();
        diag_vel_adder.reset();
        cross_adder.reset();

        DestView view{&stats_buf, &diag_pose_buf, &diag_vel_buf, &cross_buf};
        imax=std::min(imax, this->_moving->_num_leaves);
        for (size_t i=imin; i<imax; ++i) {
          const auto& moving_leaf=this->_moving->_nodes_ptr[this->_moving->_leaves_indices_ptr[i]];
          Base_::quadraticTerm(view, 0,
                               this->fixed_state, this->moving_state, this->_cache,
                               this->_fixed->_nodes_ptr, moving_leaf, this->params);
          stats_adder.add(stats_buf);
          diag_pose_adder.add(diag_pose_buf);
          diag_vel_adder.add(diag_vel_buf);
          cross_adder.add(cross_buf);
        }

        BQFReturn ret;

        auto diag_pose = diag_pose_adder.sum();
        auto diag_vel  = diag_vel_adder.sum();
        auto cross     = cross_adder.sum();
        ret.stats     = stats_adder.sum();
  
        typename DiagPoseType::HessianType     H_pose; typename DiagPoseType::CoefficientType b_pose;
        typename DiagVelType::HessianType      H_vel;  typename DiagVelType::CoefficientType  b_vel;
        typename CrossType::HessianType        H_cross;

        diag_pose.unpack(H_pose, b_pose);
        diag_vel.unpack(H_vel,   b_vel);
        cross.unpack(H_cross);

        ret.H.template block<PDim, PDim>(0,    0)    = H_pose;
        ret.H.template block<PDim, VDim>(0,    PDim) = H_cross;
        ret.H.template block<VDim, PDim>(PDim, 0)    = H_cross.transpose();
        ret.H.template block<VDim, VDim>(PDim, PDim) = H_vel;
        ret.b.template head<PDim>() = b_pose;
        ret.b.template tail<VDim>() = b_vel;
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

    template <typename Base_>
    void CTICP_CPU_<Base_>::_buildQuadraticFormDual() {
      using DestViewDual      = typename Base_::DestViewDual;
      using StatsType         = typename Base_::StatsType;
      using DiagPoseType      = typename Base_::DiagPoseType;
      using DiagVelType       = typename Base_::DiagVelType;
      using CrossType         = typename Base_::CrossType;
      using CrossVelAVelBType = typename Base_::CrossVelAVelBType;
      static constexpr int PDim = Base_::PerturbationPoseDim;
      static constexpr int VDim = Base_::VelocityDim;

      StatsType         stats_buf;
      DiagPoseType      diag_pose_buf;
      DiagVelType       diag_vel_buf;
      CrossType         cross_buf;
      DiagVelType       diag_vel_B_buf;
      CrossType         cross_pose_vel_B_buf;
      CrossVelAVelBType cross_vel_AB_buf;

      StableAdder_<StatsType>         stats_adder;
      StableAdder_<DiagPoseType>      diag_pose_adder;
      StableAdder_<DiagVelType>       diag_vel_adder;
      StableAdder_<CrossType>         cross_adder;
      StableAdder_<DiagVelType>       diag_vel_B_adder;
      StableAdder_<CrossType>         cross_pose_vel_B_adder;
      StableAdder_<CrossVelAVelBType> cross_vel_AB_adder;
      stats_adder.reset();
      diag_pose_adder.reset();
      diag_vel_adder.reset();
      cross_adder.reset();
      diag_vel_B_adder.reset();
      cross_pose_vel_B_adder.reset();
      cross_vel_AB_adder.reset();

      DestViewDual view{&stats_buf, &diag_pose_buf, &diag_vel_buf, &cross_buf,
                        &diag_vel_B_buf, &cross_pose_vel_B_buf, &cross_vel_AB_buf};

      for (size_t i=0; i<this->_moving->_num_leaves; ++i) {
        const auto& moving_leaf=this->_moving->_nodes_ptr[this->_moving->_leaves_indices_ptr[i]];
        Base_::quadraticTermDual(view, 0,
                                 this->fixed_state, this->moving_state, this->_cache,
                                 this->_fixed->_nodes_ptr, moving_leaf, this->params);
        stats_adder.add(stats_buf);
        diag_pose_adder.add(diag_pose_buf);
        diag_vel_adder.add(diag_vel_buf);
        cross_adder.add(cross_buf);
        diag_vel_B_adder.add(diag_vel_B_buf);
        cross_pose_vel_B_adder.add(cross_pose_vel_B_buf);
        cross_vel_AB_adder.add(cross_vel_AB_buf);
      }

      auto diag_pose          = diag_pose_adder.sum();
      auto diag_vel           = diag_vel_adder.sum();
      auto cross              = cross_adder.sum();
      auto diag_vel_B         = diag_vel_B_adder.sum();
      auto cross_pose_vel_B   = cross_pose_vel_B_adder.sum();
      auto cross_vel_AB       = cross_vel_AB_adder.sum();
      this->stats     = stats_adder.sum();
  
      typename DiagPoseType::HessianType  H_pose; typename DiagPoseType::CoefficientType b_pose;
      typename DiagVelType::HessianType   H_vel;  typename DiagVelType::CoefficientType  b_vel;
      typename CrossType::HessianType     H_cross_pv;

      diag_pose.unpack(H_pose, b_pose);
      diag_vel.unpack(H_vel, b_vel);
      cross.unpack(H_cross_pv);

      this->H.template block<PDim, PDim>(0,    0)    = H_pose;
      this->H.template block<PDim, VDim>(0,    PDim) = H_cross_pv;
      this->H.template block<VDim, PDim>(PDim, 0)    = H_cross_pv.transpose();
      this->H.template block<VDim, VDim>(PDim, PDim) = H_vel;
      this->b.template head<PDim>() = b_pose;
      this->b.template tail<VDim>() = b_vel;

      diag_vel_B.unpack(this->H_dual_vel_B, this->b_dual_vel_B);
      cross_pose_vel_B.unpack(this->H_dual_cross_pose_vel_B);
      cross_vel_AB.unpack(this->H_dual_cross_vel_AB);
    }

    template <typename Base_>
    void CTICP_CPU_<Base_>::applyVelocitiesInPlace(typename Base_::TreeBaseType& target,
                                                   const State& s, bool skip_leaves) {
      target.applyVelocitiesInPlace(s.velocities, skip_leaves);
    }


  }
} // namespace kd_slam::cticp
