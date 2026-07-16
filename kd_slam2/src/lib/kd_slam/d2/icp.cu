#include "srrg_boss/serializable.h"
#include "icp_cuda_.h"
#include "kd_slam/icp/icp_impl_.h"
#include "kd_slam/icp/icp_cuda_impl_.cuh"
#include "kd_slam/cuda/cuda_common.h"
#include "typedefs.cuh"

namespace kd_slam {
  namespace icp {
    extern template struct ICP_<kd_slam::d2::ICP_2D_Traits>;
    template struct ICP_CUDA_<ICP_<kd_slam::d2::ICP_2D_Traits>>;
  }

  using ICPCUDA2D = d2::ICPCUDAType;
  
  void registerICPCUDATypes2D() {
    BOSS_REGISTER_CLASS(ICPCUDA2D);
  }

}
