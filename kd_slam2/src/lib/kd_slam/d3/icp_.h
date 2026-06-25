#pragma once
#include "cuda/cuda_common.h"
#include <Eigen/Geometry>
#include <iostream>
#include "kd_slam/tree/tree_defs.h"
#include "kd_slam/icp/icp_.h"
#include "kd_slam/icp/icp_cpu_.h"
#include "kd_slam/icp/icp_cuda_.h"
#include "utils/geometry_3d_.h"
#include "utils/geometry_3d_impl_.h"


namespace kd_slam::d3 {
  using namespace kd_slam::icp;

  struct ICP_3D_Traits:
    public kd_slam::geometry::GeometryTraits_<3, typename Point2fTraits::Scalar>  {
    using NodeType                   = Node_<Point3fTraits>;
    using PointTraits            = typename NodeType::Traits;
    using PointType              = typename NodeType::PointType;
    using VectorType             = typename NodeType::VectorType;
    using Scalar                 = typename NodeType::Scalar;
    static constexpr int Dim     = NodeType::Dim;
    using IsometryType           = Eigen::Transform<Scalar, Dim, Eigen::Isometry>;
  
    static constexpr int PerturbationPoseDim = 6;
    using PerturbationPoseType       = Eigen::Matrix<Scalar, PerturbationPoseDim, 1>;
    using JacobianPoseType           = Eigen::Matrix<Scalar, 1, PerturbationPoseDim>;

    static constexpr int OptimizationDim = PerturbationPoseDim;
    using JacobianType           = Eigen::Matrix<Scalar, 1, OptimizationDim>;
    using PerturbationType       = Eigen::Matrix<Scalar, OptimizationDim, 1>;
    using HessianType            = Eigen::Matrix<Scalar, OptimizationDim, OptimizationDim>;
    using CoefficientType        = Eigen::Matrix<Scalar, OptimizationDim, 1>;
    using GeometryTraits         = geometry::GeometryTraits_<Dim, Scalar>;
    
    using GeometryTraits::adjoint;
    using GeometryTraits::expmap;
    using GeometryTraits::transformOmega;

    __CUDA_EXPORT_INLINE__
    static int errorAndJacobian(Scalar& error,
                                JacobianPoseType& J,
                                const VectorType& p_fixed,
                                const VectorType& n_fixed,
                                const VectorType& p_moving,
                                const VectorType& n_moving,
                                Scalar min_normal_cos,
                                Scalar max_error){

      if (n_fixed.dot(n_moving) < min_normal_cos)
        return -1;

      error=(p_moving-p_fixed).dot(n_moving+n_fixed);
      if (fabs(error)>max_error)
        return -2;

      J.block<1,3>(0,0)=(n_moving+n_fixed).transpose();
      J.block<1,3>(0,3)=(p_moving.cross(n_fixed)+p_fixed.cross(n_moving)).transpose();
      return 0;
    }
    // ---- CUDA thread-count limits per kernel type ----------------------------
    static constexpr int JacobianMaxThreadsPerBlock  = 1024;
    static constexpr int StatsReduceMaxThreadsPerBlock    = 1024;
    static constexpr int DiagPoseReduceMaxThreadsPerBlock = 1024;

  };

  using ICP_3D_CPU=ICP_CPU_<ICP_<ICP_3D_Traits>>;
  using ICP_3D_CUDA=ICP_CUDA_<ICP_<ICP_3D_Traits>>;
} // namespace kd_slam::d3


namespace kd_slam {
  namespace icp {
    template <>
    struct ICPType_<Node_<kd_slam::Point3fTraits>> {
      using ICP=ICP_<kd_slam::d3::ICP_3D_Traits>;
    };
  }
}
