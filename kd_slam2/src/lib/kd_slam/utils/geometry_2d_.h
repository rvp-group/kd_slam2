#pragma once
#include "cuda/cuda_common.h"
#include <Eigen/Geometry>
#include "geometry_.h"
namespace kd_slam {
  namespace geometry {

    template <typename Scalar_>
    struct GeometryTraits_<2, Scalar_>{
      using Scalar     = Scalar_;
      static constexpr int Dim=2;
      using VectorType = Eigen::Matrix<Scalar, Dim, 1>;
      using MatrixType = Eigen::Matrix<Scalar, Dim, Dim>;
      using IsometryType = Eigen::Transform<Scalar_, Dim, Eigen::Isometry>;

      static constexpr int PerturbationPoseDim = 3;
      using PerturbationPoseType = Eigen::Matrix<Scalar, PerturbationPoseDim, 1>;
      using HessianType = Eigen::Matrix<Scalar, PerturbationPoseDim, PerturbationPoseDim>;

      static constexpr int VelocityVectorDim = 3;
      using VelocityVectorType   = Eigen::Matrix<Scalar, VelocityVectorDim, 1>;

      __CUDA_EXPORT_INLINE__
      static MatrixType rot2d(Scalar_ theta);

      __CUDA_EXPORT_INLINE__
      static IsometryType expmap(const PerturbationPoseType& v);

      __CUDA_EXPORT_INLINE__
      static PerturbationPoseType logmap(const IsometryType& T);

      // Jacobian of logmap w.r.t. isometry coefficients [tx,ty, R_col_major], 3x6
      __CUDA_EXPORT_INLINE__
      static Eigen::Matrix<Scalar, 3, 6> dlogmap(const IsometryType& T);

      // e = logmap(ZXi * X_moving), ZXi = Z^{-1} * X_fixed^{-1}
      __CUDA_EXPORT_INLINE__
      static PerturbationPoseType poseToPoseError(const IsometryType& ZXi, const IsometryType& X_moving);

      // Jacobian of e=logmap(ZXi*X_moving) w.r.t. left perturbation on X_moving, 3x3
      __CUDA_EXPORT_INLINE__
      static HessianType dPoseToPoseError(const IsometryType& ZXi, const IsometryType& X_moving);

      __CUDA_EXPORT_INLINE__
      static HessianType adjoint(const IsometryType& T);

      __CUDA_EXPORT_INLINE__
      static HessianType transformOmega(const HessianType& Omega, const IsometryType& T_AB);

      __CUDA_EXPORT_INLINE__
      static Scalar angleFromRotation(const MatrixType& dR);

      __CUDA_EXPORT_INLINE__
      static void applyVelocities(const VelocityVectorType& v,
                                  VectorType& mean,
                                  VectorType& direction,
                                  double dts_mean);
      
      __CUDA_EXPORT_INLINE__
      static Scalar getAngle(const MatrixType& R);
    };

  }
}
