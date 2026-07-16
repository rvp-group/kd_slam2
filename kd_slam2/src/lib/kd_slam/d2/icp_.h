#pragma once
#include "cuda/cuda_common.h"
#include <Eigen/Geometry>
#include <iostream>
#include "kd_slam/tree/tree_defs.h"
#include "utils/geometry_2d_.h"
#include "kd_slam/icp/icp_.h"
#include "kd_slam/icp/icp_cpu_.h"
#include "kd_slam/utils/geometry_2d_.h"
#include "kd_slam/utils/geometry_2d_impl_.h"

namespace kd_slam::d2 {
  using namespace kd_slam::icp;

  struct ICP_2D_Traits:
    public kd_slam::geometry::GeometryTraits_<2, typename Point2fTraits::Scalar> {
    using NodeType                   = Node_<Point2fTraits>;
    using PointTraits            = typename NodeType::Traits;
    using PointType              = typename NodeType::PointType;
    using VectorType             = typename NodeType::VectorType;
    using Scalar                 = typename NodeType::Scalar;
    static constexpr int Dim     = NodeType::Dim;
    using IsometryType           = Eigen::Transform<Scalar, Dim, Eigen::Isometry>;
    
    static constexpr int PerturbationPoseDim = 3;
    using PerturbationPoseType       = Eigen::Matrix<Scalar, PerturbationPoseDim, 1>;
    using JacobianPoseType           = Eigen::Matrix<Scalar, 1, PerturbationPoseDim>;

    static constexpr int OptimizationDim = PerturbationPoseDim;
    using JacobianType           = Eigen::Matrix<Scalar, 1, OptimizationDim>;
    using PerturbationType       = Eigen::Matrix<Scalar, OptimizationDim, 1>;
    using HessianType            = Eigen::Matrix<Scalar, OptimizationDim, OptimizationDim>;
    using CoefficientType        = Eigen::Matrix<Scalar, OptimizationDim, 1>;
    // parameters

    // ---- CUDA thread-count limits per kernel type --------------------------
    static constexpr int JacobianMaxThreadsPerBlock       = 1024;
    static constexpr int DiagPoseReduceMaxThreadsPerBlock = 1024;
    static constexpr int StatsReduceMaxThreadsPerBlock    = 1024;
    using GeometryTraits  = kd_slam::geometry::GeometryTraits_<2,Scalar>;
    using GeometryTraits::expmap;
    using GeometryTraits::rot2d;
    
    __CUDA_EXPORT_INLINE__
    static int errorAndJacobian(Scalar& error,
                                JacobianPoseType& J,
                                const VectorType& p_fixed,
                                const VectorType& n_fixed,
                                const VectorType& p_moving,
                                const VectorType& n_moving,
                                Scalar min_normal_cos,
                                Scalar max_error) {
      Scalar ne=n_fixed.dot(n_moving);
      if (ne < min_normal_cos) {
        return -1;
      }
      error=(p_moving-p_fixed).dot(n_moving+n_fixed);
      if (fabs(error) > max_error) {
        return -2;
      }
      J.block<1,2>(0,0)=(n_moving+n_fixed).transpose();
      J(0,2) =
        -n_fixed.x()*p_moving.y()
        +n_fixed.y()*p_moving.x()
        +p_fixed.x()*n_moving.y()
        -p_fixed.y()*n_moving.x();
      return 0;
    }
  };

  using ICP_2D_CPU=ICP_CPU_<ICP_<ICP_2D_Traits>>;

} // namespace kd_slam::d2

namespace kd_slam {
  namespace icp {
    template <>
    struct ICPType_<Node_<kd_slam::Point2fTraits>> {
      using ICP=ICP_<kd_slam::d2::ICP_2D_Traits>;
    };
  }
  
  using ICPTerminationCriteriaScore=icp::ICPTerminationCriteriaScore_<kd_slam::d2::ICP_2D_Traits::Scalar>;

}
