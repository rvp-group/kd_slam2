#pragma once
#include "kd_slam/d2/icp_.h"
#include "kd_slam/cticp/cticp_.h"
#include "kd_slam/cticp/cticp_cpu_.h"
#include "utils/geometry_2d_.h"
#include "utils/geometry_2d_impl_.h"


namespace kd_slam::d2 {
  using namespace kd_slam::icp;
  using namespace kd_slam::cticp;
  using namespace geometry;
  
  struct CTICP_2D_Traits : public ICP_2D_Traits {
    using MatrixType           = Eigen::Matrix<Scalar, Dim, Dim>;
    using TransformType        = IsometryType;
    static constexpr int VelocityDim = PerturbationPoseDim;
    using VelocityType         = Eigen::Matrix<Scalar, VelocityDim, 1>;
    using JacobianVelocityType = Eigen::Matrix<Scalar, 1, VelocityDim>;

    // override optimization dimension to pose + velocity
    static constexpr int OptimizationDim = PerturbationPoseDim + VelocityDim;
    using JacobianType    = Eigen::Matrix<Scalar, 1, OptimizationDim>;
    using PerturbationType = Eigen::Matrix<Scalar, OptimizationDim, 1>;
    using HessianType     = Eigen::Matrix<Scalar, OptimizationDim, OptimizationDim>;
    using CoefficientType = Eigen::Matrix<Scalar, OptimizationDim, 1>;

    // ---- CUDA thread-count limits per kernel type --------------------------
    static constexpr int JacobianMaxThreadsPerBlock       = 1024;
    static constexpr int StatsReduceMaxThreadsPerBlock    = 1024;
    static constexpr int DiagPoseReduceMaxThreadsPerBlock = 1024;
    static constexpr int DiagVelReduceMaxThreadsPerBlock  = 1024;
    static constexpr int CrossReduceMaxThreadsPerBlock    = 1024;

    using GeometryTraits  = kd_slam::geometry::GeometryTraits_<2,Scalar>;
    using GeometryTraits::expmap;
    using GeometryTraits::rot2d;

    // Perpendicular vector: perp([x,y]) = [-y, x]
    // Key identity: d(R(theta)*v)/dtheta = perp(R(theta)*v)
    //               Ra * perp(v) = perp(Ra * v)  for any 2D rotation Ra
    __CUDA_EXPORT_INLINE__ static VectorType perp(const VectorType& v);

    // ---------- error + Jacobian w.r.t. velocity ----------
    //
    // Model: cloud A undergoes rigid motion va over dt  (ma = va*dt),
    //   with translation t = ma.head<2>() and rotation dtheta = ma(2).
    //   The moved point is expressed in global frame via Ta.
    //
    // Symmetric point-to-plane error:
    //   dp = Ta*(R(dtheta)*pa_local + t) - pb_global
    //   n  = Ta.linear()*R(dtheta)*na_local + nb_global
    //   e  = dp . n
    //
    // Jacobian J = de/d(va) (1 x VelocityDim), scaled by dt (chain rule):
    //   J_t     = n^T * Ra                                    (w.r.t. t, 1x2)
    //   J_omega = n . perp(Ra*R(dtheta)*pa_local)
    //           + dp . perp(Ra*R(dtheta)*na_local)             (w.r.t. dtheta, scalar)

    __CUDA_EXPORT_INLINE__ static void errorAndJacobianVelocityA(
                                                                 Scalar& e, JacobianVelocityType& J,
                                                                 const IsometryType& Ta,
                                                                 const VectorType& pa_local, const VectorType& na_local,
                                                                 const VectorType& pb_global, const VectorType& nb_global,
                                                                 const VelocityType& va, Scalar dt);

    // Symmetric: cloud B moves with velocity vb over dt.
    //
    // Jacobian J = de/d(vb):
    //   J_t     = -n^T * Rb
    //   J_omega = -n . perp(Rb*R(dtheta)*pb_local)
    //             + dp . perp(Rb*R(dtheta)*nb_local)

    __CUDA_EXPORT_INLINE__ static void errorAndJacobianVelocityB(
                                                                 Scalar& e, JacobianVelocityType& J,
                                                                 const IsometryType& Tb,
                                                                 const VectorType& pa_global, const VectorType& na_global,
                                                                 const VectorType& pb_local, const VectorType& nb_local,
                                                                 const VelocityType& vb, Scalar dt);

    // motion-compensate a NodeType using per-node timestamp _dts_mean
    __CUDA_EXPORT_INLINE__ static NodeType applyVelocities(const VelocityType& v, const NodeType& src);

    __CUDA_EXPORT_INLINE__ static PointType applyVelocities(const VelocityType& v, const PointType& src, const double t_start=0);

  };

  using CTICP_2D_CPU  = CTICP_CPU_<CTICP_<CTICP_2D_Traits>>;

} // namespace kd_slam::d2

namespace kd_slam {
  namespace cticp {
    template <>
    struct CTICPType_<Node_<kd_slam::Point2fTraits>> {
      using ICP=CTICP_<kd_slam::d2::CTICP_2D_Traits>;
    };
  }
}
