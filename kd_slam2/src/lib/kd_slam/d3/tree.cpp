#include "srrg_boss/serializable.h"
#include "typedefs.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "cuda/cuda_common.h"
#include "utils/geometry_.h"
#include "utils/geometry_3d_.h"
#include "utils/geometry_3d_impl_.h"
#include "kd_slam/tree/tree_defs.h"
#include "kd_slam/tree/tree_defs.h"
#include "kd_slam/tree/tree_impl_.h"
#include "kd_slam/tree/tree_cpu_impl_.h"
#include "kd_slam/tree/tree_generator_impl_.h"
#include "kd_slam/utils/voxelizer_.h"
#include "kd_slam/utils/voxelizer_impl_.h"

#ifdef HAVE_CUDA
#include "kd_slam/tree/tree_cuda_.h"
#endif

namespace kd_slam {

  template struct  Node_<Point3fTraits>;
  template struct  Tree_<NodePoint3f>;
  template struct  TreeCPU_<TreePoint3f>;
  template struct  LabelPolicyBase_<NodePoint3f>;
  template struct  CountLeafPolicy_<NodePoint3f>;
  template struct  FlatLeafPolicy_<NodePoint3f>;
  template struct  TreeGenerator_<NodePoint3f>;
  namespace utils {
    template struct Voxelizer_<kd_slam::d3::PointTraits>;
  }

  template TreeCPUPoint3f::TreeCPU_<CountLeafPolicyPoint3f>
  (TreeCPUPoint3f::PointsVectorType&, const CountLeafPolicyPoint3f&);

  template TreeCPUPoint3f::TreeCPU_<FlatLeafPolicyPoint3f>
  (TreeCPUPoint3f::PointsVectorType&, const FlatLeafPolicyPoint3f&);

  template std::ostream& operator << (std::ostream& os, const NodePoint3f& n);

  
  using TreeGenerator3D = TreeGenerator_<d3::NodeType>;
  using Voxelizer3D     = utils::Voxelizer_<d3::PointTraits>;

  void kd_slam_registerTreeTypes3D() {
    BOSS_REGISTER_CLASS(TreeGenerator3D);
    BOSS_REGISTER_CLASS(Voxelizer3D);
  }

} // namespace kd_slam
