#pragma once
#include "cticp_.h"
#include "kd_slam/icp/icp_cpu_.h"
#include "kd_slam/cticp/cticp_cpu_.h"
#include "kd_slam/tree/flat_leaf_policy_.h"

#ifdef HAVE_CUDA
#include "kd_slam/icp/icp_cuda_.h"
#include "kd_slam/cticp/cticp_cuda_.h"
#endif

namespace kd_slam::d2 {

  using ICPTraits           = ICP_2D_Traits;
  using PointType            = ICPTraits::PointType;
  using PointTraits          = ICPTraits::PointTraits;
  using VectorType           = ICPTraits::VectorType;
  using NodeType           = ICPTraits::NodeType;
  using Scalar               = ICPTraits::Scalar;
  using IsometryType         = ICPTraits::IsometryType;
  using PerturbationPoseType = ICPTraits::PerturbationPoseType;
  using ICPBaseType          = ICP_<ICPTraits>;
  using ICPCPUType           = ICP_CPU_<ICPBaseType>;
  using TreeCPUType        = ICPCPUType::KDTreeType;
  using PointsVectorType     = std::vector<PointType>;
  using NodesVectorType      = std::vector<NodeType>;
  
  using CTICP_Traits         = CTICP_2D_Traits;
  using CTICPBaseType        = CTICP_<CTICP_Traits>;
  using CTICPCPUType         = CTICP_CPU_<CTICPBaseType>;
  using VelocityType         = CTICP_2D_Traits::VelocityType;

  
#ifdef HAVE_CUDA
  using ICPCUDAType          = ICP_CUDA_<ICPBaseType>;
  using TreeCUDAType       = ICPCUDAType::KDTreeType;
  using CTICPCUDAType        = CTICP_CUDA_<CTICPBaseType>;
#endif

  constexpr const char* DUMP_STRING =
    "set_size_ratio -1\n plot '-' with vectors title \"fixed\", '-' with vectors title \"moving\"";

  void generateScene(PointsVectorType& dest, Scalar range, Scalar resolution, int num_items);
  void generateScene(std::vector<VectorType>& dest, Scalar range, Scalar resolution, int num_items);

  void linearDeskew(PointsVectorType& dest,
                    const PointsVectorType& src,
                    const PerturbationPoseType& velocities,
                    double t_start = -1);

  TreeCPUType linearDeskew(const TreeCPUType& src,
                           const PerturbationPoseType& velocities);

  template <typename PredicateType>
  void dumpGnuplot(const TreeCPUType& fixed_tree,
                   const TreeCPUType& moving_tree,
                   const PredicateType& predicate,
                   const IsometryType& X = IsometryType::Identity()) {
    using namespace std;
    cout << DUMP_STRING << endl;
    fixed_tree.dumpSelectedNodes(cout, predicate);
    cout << "e" << endl;
    moving_tree.dumpSelectedNodes(cout, predicate, X);
    cout << "e" << endl;
  }

  template <typename PointTraits_>
  inline void assignTimeStamps(PointsVectorType& points) {
    if constexpr (PointTraits_::HasTimestamp) {
      for (auto& p : points) {
        const auto& c = PointTraits_::coordinates(p);
        PointTraits_::stamp(p) = atan2(c.y(), c.x());
      }
    }
  }

} // namespace kd_slam::d2

#include "kd_slam/descriptor/descriptor_defs.h"
#include "kd_slam/tree/node_traits_.h"

namespace kd_slam::d2 {
  using namespace kd_slam::descriptor;
  using ExtractorPoint2f  = ExtractorPoint2f_<NodeTraits_<NodePoint2f>::ExtractorLevel>;
  using DescriptorPoint2f = DescriptorPoint2f_<NodeTraits_<NodePoint2f>::ExtractorLevel>;
  using MatcherPoint2f    = MatcherPoint2f_<NodeTraits_<NodePoint2f>::ExtractorLevel>;
}

#ifndef __CUDACC__
#include "kd_slam/map/map_traits_.h"
#include "srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h"
#include "srrg_solver/variables_and_factors/types_2d/se2_pose_pose_left_error_factor_ad.h"
#include "srrg_solver/variables_and_factors/types_common/all_types.h"
#include "kd_slam/map/velocity_prior_factor_.h"

namespace kd_slam {
  namespace map {
    template <>
    struct MapTraits_<d2::NodeType> {
      using SolverVariableType             = srrg2_solver::VariableSE2LeftAD;
      using SolverPGOFactorType            = srrg2_solver::SE2PosePoseLeftErrorFactorAD;
      using SolverVelocityVariableType     = srrg2_solver::VariableVector3;
      using SolverVelocityPriorFactorType  = VelocityPriorFactor_<srrg2_solver::VariableVector3>;
      using SolverGravityPriorFactorType   = void;
    };
  }
}
#endif // __CUDACC__
