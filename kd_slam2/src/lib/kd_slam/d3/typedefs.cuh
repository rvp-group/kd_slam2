#pragma once
#include "typedefs.h"
#include "kd_slam/icp/icp_cuda_.h"
#include "kd_slam/cticp/cticp_cuda_.h"

namespace kd_slam::d3 {
  using ICPCUDAType          = ICP_CUDA_<ICPBaseType>;
  using TreeCUDAType       = ICPCUDAType::KDTreeType;
  using CTICPCUDAType        = CTICP_CUDA_<CTICPBaseType>;
}
