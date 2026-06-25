#pragma once
#include "geometry_2d_.h"

namespace kd_slam {
  namespace geometry {

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<2, Scalar_>::MatrixType
    GeometryTraits_<2, Scalar_>::rot2d(Scalar_ theta) {
      const Scalar_ s = std::sin(theta), c = std::cos(theta);
      MatrixType R;
      R << c, -s, s, c;
      return R;
    }

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<2, Scalar_>::IsometryType
    GeometryTraits_<2, Scalar_>::expmap(const PerturbationPoseType& v) {
      IsometryType iso = IsometryType::Identity();
      iso.linear() = rot2d(v(2));
      iso.translation() = v.template head<2>();
      return iso;
    }

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<2, Scalar_>::PerturbationPoseType
    GeometryTraits_<2, Scalar_>::logmap(const typename GeometryTraits_<2, Scalar_>::IsometryType& T ) {
      PerturbationPoseType v;
      v.template block<2,1>(0,0)=T.translation();
      v(2,0)=angleFromRotation(T.linear());
      return v;
    }


    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    Eigen::Matrix<Scalar_, 3, 6>
    GeometryTraits_<2, Scalar_>::dlogmap(const IsometryType& T) {
      Eigen::Matrix<Scalar_, 3, 6> J = Eigen::Matrix<Scalar_, 3, 6>::Zero();
      J(0,0) = Scalar_(1);
      J(1,1) = Scalar_(1);
      // theta = atan2(R(1,0), R(0,0)), col-major: coeff 2=R(0,0), 3=R(1,0)
      J(2,2) = -T.linear()(1,0);
      J(2,3) =  T.linear()(0,0);
      return J;
    }

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<2, Scalar_>::PerturbationPoseType
    GeometryTraits_<2, Scalar_>::poseToPoseError(const IsometryType& ZXi,
                                                  const IsometryType& X_moving) {
      return logmap(ZXi * X_moving);
    }

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<2, Scalar_>::HessianType
    GeometryTraits_<2, Scalar_>::dPoseToPoseError(const IsometryType& ZXi,
                                                   const IsometryType& X_moving) {
      const IsometryType    P  = ZXi * X_moving;
      const MatrixType&     R  = ZXi.linear();
      const VectorType&     t  = X_moving.translation();
      const MatrixType&     Rm = X_moving.linear();
      Eigen::Matrix<Scalar_, 6, 3> B = Eigen::Matrix<Scalar_, 6, 3>::Zero();
      B.template block<2,2>(0,0) = R;
      B.template block<2,1>(0,2) = R * VectorType(-t.y(), t.x());
      for (int i = 0; i < 2; ++i) {
        const VectorType ci = Rm.col(i);
        B.template block<2,1>(2 + 2*i, 2) = R * VectorType(-ci.y(), ci.x());
      }
      return dlogmap(P) * B;
    }

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<2, Scalar_>::HessianType
    GeometryTraits_<2, Scalar_>::adjoint(const IsometryType& T) {
      HessianType Ad = HessianType::Zero();
      Ad.template block<2,2>(0,0) = T.linear();
      Ad(0,2) = -T.translation().y();
      Ad(1,2) =  T.translation().x();
      Ad(2,2) = Scalar_(1);
      return Ad;
    }

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<2, Scalar_>::HessianType
    GeometryTraits_<2, Scalar_>::transformOmega(const HessianType& Omega, const IsometryType& T_AB) {
      const HessianType Ad_BA = adjoint(T_AB.inverse());
      return Ad_BA.transpose() * Omega * Ad_BA;
    }

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<2, Scalar_>::Scalar
    GeometryTraits_<2, Scalar_>::angleFromRotation(const MatrixType& dR) {
      return std::atan2(dR(1,0), dR(0,0));
    }

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    void GeometryTraits_<2, Scalar_>::applyVelocities(const VelocityVectorType& v,
                                                      VectorType& mean,
                                                      VectorType& direction,
                                                      double dts_mean) {
      const auto M = expmap(v * Scalar_(dts_mean));
      mean      = M * mean;
      direction = M.linear() * direction;
    }


    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    Scalar_ GeometryTraits_<2, Scalar_>::getAngle(const MatrixType& R) {
      return atan2(R(1,0),R(0,0));
    }

  }
}
