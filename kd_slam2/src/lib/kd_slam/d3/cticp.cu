#include "srrg_boss/serializable.h"
#include "kd_slam/d3/cticp_.h"
#include "kd_slam/d3/cticp_impl_.h"
#include "kd_slam/cticp/cticp_impl_.h"
#include "kd_slam/cticp/cticp_cuda_impl_.cuh"
#include "typedefs.cuh"

namespace kd_slam {
  namespace cticp {
    extern template struct CTICP_<kd_slam::d3::CTICP_3D_Traits>;
    template struct CTICP_CUDA_<CTICP_<kd_slam::d3::CTICP_3D_Traits>>;
  }

  using CTICPCUDA3D = d3::CTICPCUDAType;
  
  void registerCTICPCUDATypes3D() {
    BOSS_REGISTER_CLASS(CTICPCUDA3D);
  }

}
