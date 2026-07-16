#pragma once
#include "icp_.h"
#include "kd_slam/icp/icp_cuda_.h"
namespace kd_slam::d2 {
  using namespace kd_slam::icp;

  using ICP_2D_CUDA=ICP_CUDA_<ICP_<ICP_2D_Traits>>;

} // namespace kd_slam::d2
