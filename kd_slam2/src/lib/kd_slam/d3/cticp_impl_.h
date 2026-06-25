#pragma once
#include "cticp_.h"


namespace kd_slam::d3 {
  using namespace kd_slam::icp;
  using namespace kd_slam::cticp;
  // ---- errorAndJacobianVelocityA -----------------------------------------

  __CUDA_EXPORT_INLINE__ void
  CTICP_3D_Traits::errorAndJacobianVelocityA(
                                             Scalar& e, JacobianVelocityType& J,
                                             const IsometryType& Ta,
                                             const VectorType& pa_local, const VectorType& na_local,
                                             const VectorType& pb_global, const VectorType& nb_global,
                                             const VelocityType& va, Scalar dt) {
    const VelocityType ma       = va * dt;
    const VectorType   t_motion = ma.head<3>();
    const VectorType   wa       = ma.tail<3>();

    VectorType pa_moved, na_moved;
    MatrixType dR_pa, dR_na;
    GeometryTraits::dRxp(pa_moved, dR_pa, wa, pa_local);
    GeometryTraits::dRxp(na_moved, dR_na, wa, na_local);
    pa_moved += t_motion;

    const MatrixType& Ra        = Ta.linear();
    const VectorType  pa_global = Ta * pa_moved;
    const VectorType  na_global = Ra * na_moved;

    const VectorType n  = na_global + nb_global;
    const VectorType dp = pa_global - pb_global;

    e = dp.dot(n);

    J.head<3>() = n.transpose() * Ra;
    J.tail<3>() = n.transpose() * (Ra * dR_pa) + dp.transpose() * (Ra * dR_na);
    J *= dt;
  }

  // ---- errorAndJacobianVelocityB -----------------------------------------

  __CUDA_EXPORT_INLINE__ void
  CTICP_3D_Traits::errorAndJacobianVelocityB(
                                             Scalar& e, JacobianVelocityType& J,
                                             const IsometryType& Tb,
                                             const VectorType& pa_global, const VectorType& na_global,
                                             const VectorType& pb_local, const VectorType& nb_local,
                                             const VelocityType& vb, Scalar dt) {

    const VelocityType mb       = vb * dt;
    const VectorType   t_motion = mb.head<3>();
    const VectorType   wb       = mb.tail<3>();

    VectorType pb_moved, nb_moved;
    MatrixType dR_pb, dR_nb;
    GeometryTraits::dRxp(pb_moved, dR_pb, wb, pb_local);
    GeometryTraits::dRxp(nb_moved, dR_nb, wb, nb_local);
    pb_moved += t_motion;

    const MatrixType& Rb        = Tb.linear();
    const VectorType  pb_global = Tb * pb_moved;
    const VectorType  nb_global = Rb * nb_moved;

    const VectorType n  = na_global + nb_global;
    const VectorType dp = pa_global - pb_global;

    e = dp.dot(n);

    J.head<3>() = -n.transpose() * Rb;
    J.tail<3>() = -n.transpose() * (Rb * dR_pb) + dp.transpose() * (Rb * dR_nb);
    J *= dt;
  }

  __CUDA_EXPORT_INLINE__ CTICP_3D_Traits::NodeType
  CTICP_3D_Traits::applyVelocities(const VelocityType& v, const NodeType& src) {

    NodeType dest=src;
    const VectorType t_delta=v.head<3>()*src._dts_mean;
    const VectorType r_delta=v.tail<3>()*src._dts_mean;
    auto R_delta=GeometryTraits::rodrigues(r_delta);
    dest._mean=R_delta*src._mean+t_delta;
    dest._direction=R_delta*src._direction;
    return dest;
  }

  __CUDA_EXPORT_INLINE__ CTICP_3D_Traits::PointType
  CTICP_3D_Traits::applyVelocities(const VelocityType& v, const PointType& src, double t_start) {
    PointType dest = src;
    if constexpr(PointTraits::HasTimestamp) {
      double dts=PointTraits::stamp(dest)-t_start;
      const auto M = GeometryTraits::expmap(v*dts);
      PointTraits::coordinates(dest) = applyIsometry_(M, PointTraits::coordinates(src));
    }
    return dest;
  }

} // namespace kd_slam::d3
