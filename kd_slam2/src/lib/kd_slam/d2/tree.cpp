#include "srrg_boss/serializable.h"
#include "typedefs.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "cuda/cuda_common.h"
#include "utils/geometry_.h"
#include "utils/geometry_2d_.h"
#include "utils/geometry_2d_impl_.h"
#include "kd_slam/tree/tree_defs.h"
#include "kd_slam/tree/tree_defs.h"
#include "kd_slam/tree/tree_impl_.h"
#include "kd_slam/tree/tree_cpu_impl_.h"
#include "kd_slam/tree/tree_generator_impl_.h"
#include "kd_slam/utils/voxelizer_.h"
#include "kd_slam/utils/voxelizer_impl_.h"

namespace kd_slam {


  template struct  Node_<Point2fTraits>;
  template struct  Tree_<NodePoint2f>;
  template struct  TreeCPU_<TreePoint2f>;
  template struct  LabelPolicyBase_<NodePoint2f>;
  template struct  CountLeafPolicy_<NodePoint2f>;
  template struct  FlatLeafPolicy_<NodePoint2f>;
  template struct  TreeGenerator_<NodePoint2f>;

  namespace utils {
    template struct Voxelizer_<kd_slam::d2::PointTraits>;
  }

  ToComputeFn_<TreePoint2f> g_tree_toCompute2d =
    [](std::shared_ptr<TreePoint2f>& t, bool gpu) {
      return Tree_toCompute_<TreePoint2f>(t, gpu);
    };
  
  template<>
  std::shared_ptr<TreePoint2f> 
  Tree_toCompute<TreePoint2f>(std::shared_ptr<TreePoint2f>& t, bool is_gpu) {
    return g_tree_toCompute2d(t, is_gpu);
  }

  CopyToFn_<TreePoint2f> g_tree_copyTo2d =
    [](TreePoint2f& dest, const TreePoint2f& src){
      Tree_copyTo_<TreePoint2f>(dest, src);
    };

  template<>
  void 
  Tree_copyTo<TreePoint2f>(TreePoint2f& dest, const TreePoint2f& src) {
    g_tree_copyTo2d(dest, src);
  }


  template TreeCPUPoint2f::TreeCPU_<CountLeafPolicyPoint2f>
  (TreeCPUPoint2f::PointsVectorType&, const CountLeafPolicyPoint2f&);

  template TreeCPUPoint2f::TreeCPU_<FlatLeafPolicyPoint2f>
  (TreeCPUPoint2f::PointsVectorType&, const FlatLeafPolicyPoint2f&);

  template std::ostream& operator << (std::ostream& os, const NodePoint2f& n);

  
  using TreeGenerator2D = TreeGenerator_<d2::NodeType>;
  using Voxelizer2D     = utils::Voxelizer_<d2::PointTraits>;

  void registerTreeTypes2D() {
    BOSS_REGISTER_CLASS(TreeGenerator2D);
    BOSS_REGISTER_CLASS(Voxelizer2D);
  }

} // namespace kd_slam
