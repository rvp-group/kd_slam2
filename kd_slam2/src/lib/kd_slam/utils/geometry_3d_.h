#pragma once
#include "geometry_.h"

namespace kd_slam{
  namespace geometry {
    template <typename Scalar_>
    struct GeometryTraits_<3, Scalar_>{
      using Scalar     = Scalar_;
      static constexpr int Dim=3;
      using VectorType = Eigen::Matrix<Scalar, Dim, 1>;
      using MatrixType = Eigen::Matrix<Scalar, Dim, Dim>;
      using IsometryType = Eigen::Transform<Scalar_, Dim, Eigen::Isometry>;

      static constexpr int PerturbationPoseDim = 6;
      using PerturbationPoseType = Eigen::Matrix<Scalar, PerturbationPoseDim, 1>;
      using HessianType = Eigen::Matrix<Scalar, PerturbationPoseDim, PerturbationPoseDim>;

      static constexpr int VelocityVectorDim = 6;
      using VelocityVectorType   = Eigen::Matrix<Scalar, VelocityVectorDim, 1>;


      __CUDA_EXPORT_INLINE__
      static MatrixType skew(const VectorType& v);

      // R = I + sin(a)*S + (1-cos(a))*S2
      __CUDA_EXPORT_INLINE__
      static MatrixType rodrigues(Scalar_ angle, const MatrixType& S, const MatrixType& S2);

      // Rotation from angle and unit axis
      __CUDA_EXPORT_INLINE__
      static MatrixType rodrigues(Scalar_ angle, const VectorType& n);

      // Rotation by axis-angle vector w (angle = ||w||)
      __CUDA_EXPORT_INLINE__
      static MatrixType rodrigues(const VectorType& w);

      // SO(3) logarithm: returns axis-angle vector w s.t. rodrigues(w) = R
      __CUDA_EXPORT_INLINE__
      static VectorType logSO3(const MatrixType& R);

      // Jacobian of logSO3 w.r.t. R coefficients in column-major order, 3x9
      __CUDA_EXPORT_INLINE__
      static Eigen::Matrix<Scalar, 3, 9> dlogSO3(const MatrixType& R);

      // Jacobian of logmap w.r.t. isometry coefficients [tx,ty,tz, R_col_major], 6x12
      __CUDA_EXPORT_INLINE__
      static Eigen::Matrix<Scalar, 6, 12> dlogmap(const IsometryType& T);

      // e = logmap(ZXi * X_moving), ZXi = Z^{-1} * X_fixed^{-1}
      __CUDA_EXPORT_INLINE__
      static PerturbationPoseType poseToPoseError(const IsometryType& ZXi, const IsometryType& X_moving);

      // Jacobian of e=logmap(ZXi*X_moving) w.r.t. left perturbation on X_moving, 6x6
      __CUDA_EXPORT_INLINE__
      static HessianType dPoseToPoseError(const IsometryType& ZXi, const IsometryType& X_moving);


      // Returns 3x3 M s.t. M.col(i) = d(R(w)*u)/dw_i  (Gallego & Yezzi 2014)
      __CUDA_EXPORT_INLINE__
      static MatrixType dRxp(Scalar_ angle,
                                    const VectorType& n,
                                    const MatrixType& R,
                                    const MatrixType& S,
                                    const VectorType& u);

      // Computes ur = R(w)*u and dR = d(R(w)*u)/dw simultaneously
      __CUDA_EXPORT_INLINE__
      static void dRxp(VectorType& ur, MatrixType& dR,
                                              const VectorType& w, const VectorType& u);

      // SO(E) logarithm: returns translation + axis-angle vector of rotation
      __CUDA_EXPORT_INLINE__
      static PerturbationPoseType logmap(const IsometryType& T);

      __CUDA_EXPORT_INLINE__
      static IsometryType expmap(const Eigen::Matrix<Scalar_, 6, 1>& v);

      __CUDA_EXPORT_INLINE__
      static HessianType adjoint(const IsometryType& T);
      
      __CUDA_EXPORT_INLINE__
      static HessianType transformOmega(const HessianType& Omega, const IsometryType& T_AB);

      static Scalar angleFromRotation(const MatrixType& dR) {
        return Eigen::AngleAxis<Scalar>(dR).angle();
      }

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
