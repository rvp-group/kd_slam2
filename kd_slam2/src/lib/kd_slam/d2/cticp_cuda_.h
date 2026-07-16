#pragma once
#include "cticp_.h"
#include "kd_slam/cticp/cticp_cuda_.h"
namespace kd_slam::d2 {
  using namespace kd_slam::icp;

  using CTICP_2D_CUDA=CTICP_CUDA_<CTICP_<CTICP_2D_Traits>>;

} // namespace kd_slam::d2
