#pragma once
#include "geometry_3d_.h"

namespace kd_slam {
  namespace geometry {

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<3, Scalar_>::MatrixType
    GeometryTraits_<3, Scalar_>::skew(const VectorType& v) {
      MatrixType m;
      m <<
        0,      -v.z(),  v.y(),
        v.z(),   0,     -v.x(),
        -v.y(),  v.x(),  0;
      return m;
    }

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<3, Scalar_>::MatrixType
    GeometryTraits_<3, Scalar_>::rodrigues(Scalar_ angle,
              const typename GeometryTraits_<3, Scalar_>::MatrixType& S,
              const typename GeometryTraits_<3, Scalar_>::MatrixType& S2) {
      using MatrixType=typename GeometryTraits_<3, Scalar_>::MatrixType;
      return MatrixType::Identity() + sin(angle)*S + (Scalar_(1.) - cos(angle))*S2;
    }

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<3, Scalar_>::MatrixType
    GeometryTraits_<3, Scalar_>::rodrigues(Scalar_ angle, const VectorType& n) {
      const auto S = skew(n);
      return rodrigues(angle, S, S*S);
    }

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<3, Scalar_>::MatrixType
    GeometryTraits_<3, Scalar_>::rodrigues(const typename GeometryTraits_<3, Scalar_>::VectorType& w) {
      using MatrixType=typename GeometryTraits_<3, Scalar_>::MatrixType;
      const Scalar_ angle = w.norm();
      if (angle < Scalar_(1e-9))
        return MatrixType::Identity();
      return rodrigues(angle, w * (Scalar_(1.) / angle));
    }

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<3, Scalar_>::VectorType
    GeometryTraits_<3, Scalar_>::logSO3(const MatrixType& R) {
      Scalar_ cos_a = Scalar_(0.5) * (R.trace() - Scalar_(1));
      cos_a = std::max(Scalar_(-1), std::min(Scalar_(1), cos_a));
      const Scalar_ angle = std::acos(cos_a);
      const VectorType ax{R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1)};
      if (angle < Scalar_(1e-7))
        return ax * Scalar_(0.5);
      return ax * (angle / (Scalar_(2) * std::sin(angle)));
    }
    
    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    void GeometryTraits_<3, Scalar_>::applyVelocities(const VelocityVectorType& v,
                                                      VectorType& mean,
                                                      VectorType& direction,
                                                      double dts_mean) {
      const VectorType t_delta = v.template head<3>() * Scalar_(dts_mean);
      const VectorType r_delta = v.template tail<3>() * Scalar_(dts_mean);
      const MatrixType R_delta = rodrigues(r_delta);
      mean      = R_delta * mean + t_delta;
      direction = R_delta * direction;
    }


    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    Eigen::Matrix<Scalar_, 3, 9>
    GeometryTraits_<3, Scalar_>::dlogSO3(const MatrixType& R) {
      using JType = Eigen::Matrix<Scalar_, 3, 9>;
      const VectorType ax{R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1)};
      Scalar_ cos_a = Scalar_(0.5) * (R.trace() - Scalar_(1));
      cos_a = std::max(Scalar_(-1), std::min(Scalar_(1), cos_a));
      const Scalar_ angle = std::acos(cos_a);
      const bool small   = angle < Scalar_(1e-7);
      const Scalar_ f    = small ? Scalar_(0.5) : angle / (Scalar_(2) * std::sin(angle));
      JType J = JType::Zero();
      // f * M_ax: d(ax)/dR in column-major layout
      // ax_0=R(2,1)-R(1,2): col5=+f, col7=-f
      // ax_1=R(0,2)-R(2,0): col6=+f, col2=-f
      // ax_2=R(1,0)-R(0,1): col1=+f, col3=-f
      J(0,5) =  f;  J(0,7) = -f;
      J(1,6) =  f;  J(1,2) = -f;
      J(2,1) =  f;  J(2,3) = -f;
      if (!small) {
        const Scalar_ s = std::sin(angle);
        const Scalar_ c = (s - angle * std::cos(angle)) / (Scalar_(4) * s * s * s);
        // -c * ax * trace_mask^T: diagonal cols 0, 4, 8
        for (int k = 0; k < 3; ++k) {
          J(k,0) -= c * ax(k);
          J(k,4) -= c * ax(k);
          J(k,8) -= c * ax(k);
        }
      }
      return J;
    }

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    Eigen::Matrix<Scalar_, 6, 12>
    GeometryTraits_<3, Scalar_>::dlogmap(const IsometryType& T) {
      Eigen::Matrix<Scalar_, 6, 12> J = Eigen::Matrix<Scalar_, 6, 12>::Zero();
      J.template block<3,3>(0,0).setIdentity();
      J.template block<3,9>(3,3) = dlogSO3(T.linear());
      return J;
    }

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<3, Scalar_>::PerturbationPoseType
    GeometryTraits_<3, Scalar_>::poseToPoseError(const IsometryType& ZXi,
                                                  const IsometryType& X_moving) {
      return logmap(ZXi * X_moving);
    }

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<3, Scalar_>::HessianType
    GeometryTraits_<3, Scalar_>::dPoseToPoseError(const IsometryType& ZXi,
                                                   const IsometryType& X_moving) {
      const IsometryType    P  = ZXi * X_moving;
      const MatrixType&     R  = ZXi.linear();
      const VectorType&     t  = X_moving.translation();
      const MatrixType&     Rm = X_moving.linear();
      Eigen::Matrix<Scalar_, 12, 6> B = Eigen::Matrix<Scalar_, 12, 6>::Zero();
      B.template block<3,3>(0,0) = R;
      B.template block<3,3>(0,3) = -R * skew(t);
      for (int i = 0; i < 3; ++i)
        B.template block<3,3>(3 + 3*i, 3) = -R * skew(VectorType(Rm.col(i)));
      return dlogmap(P) * B;
    }

    // derivative of R(w)*u -- Gallego & Yezzi, arXiv:1312.0788v2
    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<3, Scalar_>::MatrixType
    GeometryTraits_<3, Scalar_>::dRxp(Scalar_ angle,
         const typename GeometryTraits_<3, Scalar_>::VectorType& n,
         const typename GeometryTraits_<3, Scalar_>::MatrixType& R,
         const typename GeometryTraits_<3, Scalar_>::MatrixType& S,
         const typename GeometryTraits_<3, Scalar_>::VectorType& u) {
      using MatrixType=typename GeometryTraits_<3, Scalar_>::MatrixType;

      if (angle < Scalar_(1e-9))
        return -skew(u);
      return -R * skew(u) * (n * n.transpose() + (R.transpose() - MatrixType::Identity()) * S * (Scalar_(1.) / angle));
    }

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    void GeometryTraits_<3, Scalar_>::dRxp(VectorType& ur, MatrixType& dR,
              const VectorType& w, const VectorType& u) {
      const Scalar_ angle = w.norm();
      if (angle < Scalar_(1e-9)) {
        ur = u;
        dR = -skew(u);
        return;
      }
      const VectorType n  = w * (Scalar_(1.) / angle);
      const MatrixType S  = skew(n);
      const MatrixType S2 = S * S;
      const MatrixType R  = rodrigues(angle, S, S2);
      ur = R * u;
      dR = dRxp(angle, n, R, S, u);
    }

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<3, Scalar_>::IsometryType
    GeometryTraits_<3, Scalar_>::expmap(const typename GeometryTraits_<3, Scalar_>::PerturbationPoseType&v ) {
      IsometryType iso = IsometryType::Identity();
      iso.translation() = v.template head<3>();
      iso.linear() = rodrigues(v.template tail<3>());
      return iso;
    }

    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<3, Scalar_>::PerturbationPoseType
    GeometryTraits_<3, Scalar_>::logmap(const typename GeometryTraits_<3, Scalar_>::IsometryType& T ) {
      PerturbationPoseType v;
      v.template block<3,1>(0,0)=T.translation();
      v.template block<3,1>(3,0)=logSO3(T.linear());
      return v;
    }

    // Adjoint of T in SE(3), perturbation order: [t(3), w(3)].
    // Ad(T) = [ R    skew(t)*R ]
    //         [ 0       R      ]
    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<3, Scalar_>::HessianType
    GeometryTraits_<3, Scalar_>::adjoint(const IsometryType& T) {
      HessianType Ad = HessianType::Zero();
      const auto& R = T.linear();
      const auto& t = T.translation();
      Ad.template block<3,3>(0,0) = R;
      Ad.template block<3,3>(3,3) = R;
      Ad.template block<3,3>(0,3) = skew(VectorType(t)) * R;
      return Ad;
    }

    // Omega_BA = Ad(T_BA)^T * Omega_AB * Ad(T_BA)
    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    typename GeometryTraits_<3, Scalar_>::HessianType
    GeometryTraits_<3, Scalar_>::transformOmega(const HessianType& Omega, const IsometryType& T_AB) {
      const HessianType Ad_BA = adjoint(T_AB.inverse());
      return Ad_BA.transpose() * Omega * Ad_BA;
    }


    template <typename Scalar_>
    __CUDA_EXPORT_INLINE__
    Scalar_ GeometryTraits_<3, Scalar_>::getAngle(const MatrixType& R) {
      return angleFromRotation(R);
    }

  }
}
