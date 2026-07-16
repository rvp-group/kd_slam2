#include "srrg_boss/serializable.h"
#include "utils/geometry_2d_.h"
#include "icp_.h"
#include "kd_slam/icp/icp_impl_.h"
#include "kd_slam/icp/icp_cpu_impl_.h"
#include "typedefs.h"

// Explicit instantiations must be in the namespace where the template is defined
namespace kd_slam {
  namespace icp {
    using namespace geometry;
    template struct ICP_<kd_slam::d2::ICP_2D_Traits>;
    template struct ICP_CPU_<ICP_<kd_slam::d2::ICP_2D_Traits>>;
    template struct ICPTerminationCriteriaScore_<kd_slam::d2::ICP_2D_Traits::Scalar>;
  }

  using ICPCPU2D   = d2::ICPCPUType;
  
  void registerICPCPUTypes2D() {
    BOSS_REGISTER_CLASS(ICPCPU2D);
    BOSS_REGISTER_CLASS(ICPTerminationCriteriaScore);
  }
}
