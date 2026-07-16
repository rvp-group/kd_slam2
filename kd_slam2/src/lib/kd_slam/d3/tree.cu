#include <Eigen/Core>
#include <Eigen/Geometry>
#include "cuda/cuda_common.h"
#include "utils/geometry_.h"
#include "utils/geometry_3d_.h"
#include "utils/geometry_3d_impl_.h"

#include "kd_slam/tree/tree_defs.h"
#include "kd_slam/tree/tree_impl_.h"
#include "kd_slam/tree/tree_cuda_.h"
#include "kd_slam/tree/tree_cuda_impl_.h"
#include "kd_slam/frame/frame_tree_.h"

namespace kd_slam {
  extern template struct Node_<Point3fTraits>;
  extern template struct Tree_<NodePoint3f>;
  template struct  TreeCUDA_<TreePoint3f>;
 
  extern ToComputeFn_<TreePoint3f> g_tree_toCompute3d;
  extern CopyToFn_<TreePoint3f>    g_tree_copyTo3d;

  void __attribute__((constructor)) initializeTree3d() {
    g_tree_toCompute3d = [](std::shared_ptr<TreePoint3f>& t, bool gpu) {
      return Tree_toCompute_<TreePoint3f>(t, gpu);
    };
    g_tree_copyTo3d = [](TreePoint3f& dest, const TreePoint3f& src) {
      Tree_copyTo_<TreePoint3f>(dest, src);
    };

  }

} // namespace kd_slam


