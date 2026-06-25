#include "srrg_boss/serializable.h"
#include "icp_.h"
#include "kd_slam/icp/icp_impl_.h"
#include "kd_slam/icp/icp_cuda_impl_.cuh"
#include "kd_slam/cuda/cuda_common.h"
#include "typedefs.h"
namespace kd_slam {
  namespace icp {
    template struct ICP_CUDA_<ICP_<kd_slam::d3::ICP_3D_Traits>>;
  }

  using ICPCUDA3D = d3::ICPCUDAType;
  
  void kd_slam_registerICPCUDATypes3D() {
    BOSS_REGISTER_CLASS(ICPCUDA3D);
  }

}
