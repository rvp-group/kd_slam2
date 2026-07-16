#include <Eigen/Core>
#include <Eigen/Geometry>
#include "cuda/cuda_common.h"
#include "utils/geometry_.h"
#include "utils/geometry_2d_.h"
#include "utils/geometry_2d_impl_.h"

#include "kd_slam/tree/tree_defs.h"
#include "kd_slam/tree/tree_impl_.h"
#include "kd_slam/tree/tree_cuda_.h"
#include "kd_slam/tree/tree_cuda_impl_.h"
#include "kd_slam/frame/frame_tree_.h"

namespace kd_slam {
  extern template struct Node_<Point2fTraits>;
  extern template struct Tree_<NodePoint2f>;
  template struct  TreeCUDA_<TreePoint2f>;

  extern ToComputeFn_<TreePoint2f> g_tree_toCompute2d;
  extern CopyToFn_<TreePoint2f>    g_tree_copyTo2d;

  void __attribute__((constructor)) initializeTree2d()  {
    g_tree_toCompute2d = [](std::shared_ptr<TreePoint2f>& t, bool gpu) {
      return Tree_toCompute_<TreePoint2f>(t, gpu);
    };
    g_tree_copyTo2d = [](TreePoint2f& dest, const TreePoint2f& src) {
      Tree_copyTo_<TreePoint2f>(dest, src);
    };
  }

}



 // namespace kd_slam
