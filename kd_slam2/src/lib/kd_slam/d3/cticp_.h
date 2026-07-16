#pragma once
#include "kd_slam/d3/icp_.h"
#include "kd_slam/cticp/cticp_.h"
#include "kd_slam/cticp/cticp_cpu_.h"


namespace kd_slam::d3 {
  using namespace kd_slam::icp;
  using namespace kd_slam::cticp;

  struct CTICP_3D_Traits : public ICP_3D_Traits {
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
    using GeometryTraits         = geometry::GeometryTraits_<Dim, Scalar>;
   
    __CUDA_EXPORT_INLINE__ static IsometryType expmap(const PerturbationPoseType& v) {
      return GeometryTraits::expmap(v);
    }

    // ---------- error + Jacobian w.r.t. velocity ----------
    //
    // Model: cloud A undergoes rigid motion v_a over dt  (ma = va*dt),
    //   with translation t = ma.head<3>() and rotation w = ma.tail<3>().
    //   The moved point is expressed in global frame via Ta.
    //
    // Symmetric point-to-plane error:
    //   dp = Ta*(R(w)*pa_local + t) - pb_global
    //   n  = Ta.linear()*R(w)*na_local + nb_global
    //   e  = dp . n
    //
    // Jacobian J = de/d(va) (1 x VelocityDim), scaled by dt (chain rule).

    __CUDA_EXPORT_INLINE__ static void errorAndJacobianVelocityA(Scalar& e, JacobianVelocityType& J,
                                                                 const IsometryType& Ta,
                                                                 const VectorType& pa_local, const VectorType& na_local,
                                                                 const VectorType& pb_global, const VectorType& nb_global,
                                                                 const VelocityType& va, Scalar dt);

    __CUDA_EXPORT_INLINE__ static void errorAndJacobianVelocityB(Scalar& e, JacobianVelocityType& J,
                                                                 const IsometryType& Tb,
                                                                 const VectorType& pa_global, const VectorType& na_global,
                                                                 const VectorType& pb_local, const VectorType& nb_local,
                                                                 const VelocityType& vb, Scalar dt);

    __CUDA_EXPORT_INLINE__ static NodeType applyVelocities(const VelocityType& v, const NodeType& src);

    __CUDA_EXPORT_INLINE__ static PointType applyVelocities(const VelocityType& v, const PointType& src, double t_start=0);

  };


  using CTICP_3D_CPU  = CTICP_CPU_<CTICP_<CTICP_3D_Traits>>;
} // namespace kd_slam::d3


namespace kd_slam {
  namespace cticp {
    template <>
    struct CTICPType_<Node_<kd_slam::Point3fTraits>> {
      using ICP=CTICP_<kd_slam::d3::CTICP_3D_Traits>;
    };
  }
}
