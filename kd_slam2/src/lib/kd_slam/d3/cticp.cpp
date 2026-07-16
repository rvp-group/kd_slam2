#include "srrg_boss/serializable.h"
#include "kd_slam/d3/cticp_.h"
#include "kd_slam/d3/cticp_impl_.h"
#include "kd_slam/cticp/cticp_impl_.h"
#include "kd_slam/cticp/cticp_cpu_impl_.h"
#include "typedefs.h"

// Explicit instantiations must be in the namespace where the template is defined
namespace kd_slam {
  namespace cticp {
    template struct CTICP_<kd_slam::d3::CTICP_3D_Traits>;
    template struct CTICP_CPU_<CTICP_<kd_slam::d3::CTICP_3D_Traits>>;
  }

  using CTICPCPU3D   = d3::CTICPCPUType;

  void registerCTICPCPUTypes3D() {
    BOSS_REGISTER_CLASS(CTICPCPU3D);
  }

}
