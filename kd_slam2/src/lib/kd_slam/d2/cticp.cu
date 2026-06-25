#include "srrg_boss/serializable.h"
#include "kd_slam/d2/cticp_.h"
#include "kd_slam/d2/cticp_impl_.h"
#include "kd_slam/cticp/cticp_impl_.h"
#include "kd_slam/cticp/cticp_cuda_impl_.cuh"
#include "typedefs.h"

namespace kd_slam {
  namespace cticp {
    template struct CTICP_CUDA_<CTICP_<kd_slam::d2::CTICP_2D_Traits>>;
  }

  using CTICPCUDA2D = d2::CTICPCUDAType;
  void __attribute__((constructor)) kd_slam_registerCTICPCUDATypes2D() {
    BOSS_REGISTER_CLASS(CTICPCUDA2D);
  }

}
