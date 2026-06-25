#pragma once
#include "cuda/cuda_common.h"
#include "icp_stats_.h"

namespace kd_slam {
  namespace icp {

    template <typename TraitsType_>
    using ICPWsItemStats_ = ICPStats_<typename TraitsType_::Scalar>;
    
    // ---- Diagonal Hessian block ------------------------------------------------
    // Accumulates J^T J (upper triangular, packed) and J^T e for one Jacobian.
    template <typename JacobianType_>
    struct ICPWsItemDiag_ {
      using Scalar = typename JacobianType_::Scalar;
      static constexpr int BDim = JacobianType_::ColsAtCompileTime;
      static constexpr int HDim = (BDim+1)*BDim/2;
      static constexpr int AccumulatorDim = HDim+BDim;
      using AccumulatorType  = Eigen::Matrix<Scalar, AccumulatorDim, 1>;
      using HessianType      = Eigen::Matrix<Scalar, BDim, BDim>;
      using CoefficientType  = Eigen::Matrix<Scalar, BDim, 1>;
      AccumulatorType accumulator;

      __CUDA_EXPORT_INLINE__ void clear() { accumulator.setZero(); }

      __CUDA_EXPORT_INLINE__
      void pack(const Scalar& error, const JacobianType_& J, const Scalar& gamma) {
        int k=0;
        for (int r=0; r<BDim; ++r)
          for (int c=r; c<BDim; ++c, ++k)
            accumulator[k]=J(0,r)*J(0,c)*gamma;
        for (int r=0; r<BDim; ++r, ++k)
          accumulator[k]=J(0,r)*error*gamma;
      }

      __CUDA_EXPORT_INLINE__
      void unpack(HessianType& H, CoefficientType& b) const {
        int k=0;
        for (int r=0; r<BDim; ++r)
          for (int c=r; c<BDim; ++c, ++k)
            H(r,c)=H(c,r)=accumulator[k];
        for (int r=0; r<BDim; ++r, ++k)
          b(r)=accumulator[k];
      }

      __CUDA_EXPORT_INLINE__
      ICPWsItemDiag_& operator+=(const ICPWsItemDiag_& o) {
        accumulator+=o.accumulator; return *this;
      }

      __CUDA_EXPORT_INLINE__
      ICPWsItemDiag_ operator+(const ICPWsItemDiag_& o) const {
        auto r=*this; r+=o; return r;
      }

      static ICPWsItemDiag_ Zero() { return ICPWsItemDiag_(); }
    };

    // ---- Off-diagonal (cross) Hessian block ------------------------------------
    // Accumulates J_A^T J_B for two Jacobian blocks.
    template <typename JacobianTypeA_, typename JacobianTypeB_>
    struct ICPWsItemCross_ {
      using Scalar = typename JacobianTypeA_::Scalar;
      static constexpr int Rows = JacobianTypeA_::ColsAtCompileTime;
      static constexpr int Cols = JacobianTypeB_::ColsAtCompileTime;
      using AccumulatorType = Eigen::Matrix<Scalar, Rows, Cols>;
      using HessianType     = AccumulatorType;
      AccumulatorType accumulator;

      __CUDA_EXPORT_INLINE__ void clear() { accumulator.setZero(); }

      __CUDA_EXPORT_INLINE__
      void pack(const JacobianTypeA_& ja, const JacobianTypeB_& jb, const Scalar& gamma) {
        accumulator=ja.transpose()*jb*gamma;
      }

      __CUDA_EXPORT_INLINE__
      void unpack(HessianType& H) const { H=accumulator; }

      __CUDA_EXPORT_INLINE__
      ICPWsItemCross_& operator+=(const ICPWsItemCross_& o) {
        accumulator+=o.accumulator; return *this;
      }

      __CUDA_EXPORT_INLINE__
      ICPWsItemCross_ operator+(const ICPWsItemCross_& o) const {
        auto r=*this; r+=o; return r;
      }

      static ICPWsItemCross_ Zero() { return ICPWsItemCross_(); }
    };

  }
} // namespace kd_slam::icp
