#pragma once
#include "icp_.h"
#include "kd_slam/icp/icp_cpu_.h"
#include "cticp_.h"
#include "kd_slam/cticp/cticp_cpu_.h"
#include "kd_slam/tree/flat_leaf_policy_.h"

namespace kd_slam::d3 {

  using ICPTraits           = ICP_3D_Traits;
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

  using CTICP_Traits         = CTICP_3D_Traits;
  using CTICPBaseType        = CTICP_<CTICP_Traits>;
  using CTICPCPUType         = CTICP_CPU_<CTICPBaseType>;
  using VelocityType         = CTICP_3D_Traits::VelocityType;

  constexpr const char* DUMP_STRING =
    "set view equal xyz\n splot '-' with vectors title \"fixed\", '-' with vectors title \"moving\"";

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

} // namespace kd_slam::d3

#include "kd_slam/descriptor/descriptor_defs.h"
#include "kd_slam/tree/node_traits_.h"

namespace kd_slam::d3 {
  using namespace kd_slam::descriptor;
  using ExtractorPoint3f  = ExtractorPoint3f_<NodeTraits_<NodePoint3f>::ExtractorLevel>;
  using DescriptorPoint3f = DescriptorPoint3f_<NodeTraits_<NodePoint3f>::ExtractorLevel>;
  using MatcherPoint3f    = MatcherPoint3f_<NodeTraits_<NodePoint3f>::ExtractorLevel>;
}

#ifndef __CUDACC__
#include "kd_slam/map/map_traits_.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_pose_expmap_left_error_factor_ad.h"
#include "srrg_solver/variables_and_factors/types_common/all_types.h"
#include "kd_slam/map/velocity_prior_factor_.h"
#include "slam_gravity_prior_factor.h"

namespace kd_slam {
  namespace map {
    template <>
    struct MapTraits_<d3::NodeType> {
      using SolverVariableType             = srrg2_solver::VariableSE3ExpmapLeftAD;
      using SolverPGOFactorType            = srrg2_solver::SE3PosePoseExpmapLeftErrorFactorAD;
      using SolverVelocityVariableType     = srrg2_solver::VariableVector6;
      using SolverVelocityPriorFactorType  = VelocityPriorFactor_<srrg2_solver::VariableVector6>;
      using SolverGravityPriorFactorType   = GravityPriorFactor;
    };
  }
}
#endif // __CUDACC__
