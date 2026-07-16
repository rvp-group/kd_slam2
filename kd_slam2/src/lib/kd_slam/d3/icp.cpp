#include "icp_.h"
#include "kd_slam/icp/icp_impl_.h"
#include "kd_slam/icp/icp_cpu_impl_.h"
#include "utils/geometry_3d_impl_.h"
#include "typedefs.h"

namespace kd_slam {
  namespace icp {
    template struct ICP_<kd_slam::d3::ICP_3D_Traits>;
    template struct ICP_CPU_<ICP_<kd_slam::d3::ICP_3D_Traits>>;
  }

  using ICPCPU3D   = d3::ICPCPUType;

  void registerICPCPUTypes3D() {
    BOSS_REGISTER_CLASS(ICPCPU3D);
  }

}
