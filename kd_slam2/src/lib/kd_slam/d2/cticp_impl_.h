#pragma once
#include "cticp_.h"

namespace kd_slam::d2 {
  using namespace kd_slam::icp;
  using namespace kd_slam::cticp;
  using namespace geometry;

  __CUDA_EXPORT_INLINE__ CTICP_2D_Traits::VectorType
  CTICP_2D_Traits::perp(const VectorType& v) {
    return VectorType(-v.y(), v.x());
  }

  // ---- errorAndJacobianVelocityA -----------------------------------------

  __CUDA_EXPORT_INLINE__ void
  CTICP_2D_Traits::errorAndJacobianVelocityA(
                                             Scalar& e, JacobianVelocityType& J,
                                             const IsometryType& Ta,
                                             const VectorType& pa_local, const VectorType& na_local,
                                             const VectorType& pb_global, const VectorType& nb_global,
                                             const VelocityType& va, Scalar dt) {
    const VelocityType ma       = va * dt;
    const VectorType   t_motion = ma.head<2>();
    const Scalar       dtheta   = ma(2);

    const MatrixType R_a    = rot2d(dtheta);
    const VectorType pa_rot = R_a * pa_local;
    const VectorType na_rot = R_a * na_local;

    const MatrixType& Ra        = Ta.linear();
    const VectorType  pa_global = Ta * (pa_rot + t_motion);
    const VectorType  na_global = Ra * na_rot;

    const VectorType n  = na_global + nb_global;
    const VectorType dp = pa_global - pb_global;

    e = dp.dot(n);

    J.head<2>() = n.transpose() * Ra;
    J(2) = n.dot(perp(Ra * pa_rot)) + dp.dot(perp(na_global));
    J *= dt;
  }

  // ---- errorAndJacobianVelocityB -----------------------------------------

  __CUDA_EXPORT_INLINE__ void
  CTICP_2D_Traits::errorAndJacobianVelocityB(
                                             Scalar& e, JacobianVelocityType& J,
                                             const IsometryType& Tb,
                                             const VectorType& pa_global, const VectorType& na_global,
                                             const VectorType& pb_local, const VectorType& nb_local,
                                             const VelocityType& vb, Scalar dt) {
    const VelocityType mb       = vb * dt;
    const VectorType   t_motion = mb.head<2>();
    const Scalar       dtheta   = mb(2);

    const MatrixType R_b    = rot2d(dtheta);
    const VectorType pb_rot = R_b * pb_local;
    const VectorType nb_rot = R_b * nb_local;

    const MatrixType& Rb        = Tb.linear();
    const VectorType  pb_global = Tb * (pb_rot + t_motion);
    const VectorType  nb_global = Rb * nb_rot;

    const VectorType n  = na_global + nb_global;
    const VectorType dp = pa_global - pb_global;

    e = dp.dot(n);

    J.head<2>() = -n.transpose() * Rb;
    J(2) = -n.dot(perp(Rb * pb_rot)) + dp.dot(perp(nb_global));
    J *= dt;
  }

  // ---- applyVelocities ---------------------------------------------------

  __CUDA_EXPORT_INLINE__ CTICP_2D_Traits::NodeType
  CTICP_2D_Traits::applyVelocities(const VelocityType& v, const NodeType& src) {
    NodeType dest           = src;
    const auto M = expmap(v*src._dts_mean);
    dest._mean      = applyIsometry_(M, src._mean);
    dest._direction = applyRotation_(M, src._direction);
    return dest;
  }

  __CUDA_EXPORT_INLINE__ CTICP_2D_Traits::PointType
  CTICP_2D_Traits::applyVelocities(const VelocityType& v, const PointType& src, const double t_start) {
    PointType dest = src;
    if constexpr(PointTraits::HasTimestamp) {
      double dts=PointTraits::stamp(dest)-t_start;
      const auto M = expmap(v*dts);
      PointTraits::coordinates(dest) = applyIsometry_(M, PointTraits::coordinates(src));
    }
    return dest;
  }

} // namespace kd_slam::d2
