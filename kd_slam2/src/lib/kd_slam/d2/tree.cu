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

namespace kd_slam {

  template struct  TreeCUDA_<TreePoint2f>;

} // namespace kd_slam
