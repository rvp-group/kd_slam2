#include "srrg_boss/serializable.h"
#include "icp_.h"
#include "kd_slam/icp/icp_impl_.h"
#include "kd_slam/icp/icp_cuda_impl_.cuh"
#include "kd_slam/cuda/cuda_common.h"
#include "typedefs.h"

namespace kd_slam {
  namespace icp {
    template struct ICP_CUDA_<ICP_<kd_slam::d2::ICP_2D_Traits>>;
  }

  using ICPCUDA2D = d2::ICPCUDAType;
  
  void kd_slam_registerICPCUDATypes2D() {
    BOSS_REGISTER_CLASS(ICPCUDA2D);
  }

}
