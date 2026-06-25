//#include "srrg_boss/serializable.h"
#include "kd_slam/descriptor/descriptor_defs.h"
#include "kd_slam/descriptor/matcher_cuda_impl_.cuh"
#include "kd_slam/d3/typedefs.h"

namespace kd_slam {
  namespace descriptor {


    template struct MatcherCUDA_<MatcherPoint3f_<3>>;
    template struct MatcherCUDA_<MatcherPoint3f_<4>>;
    template struct MatcherCUDA_<MatcherPoint3f_<5>>;

    //using DescriptorMatcher3D = descriptor::MatcherCPU_<d3::MatcherPoint3f>;
  }
  
  // void kd_slam_registerDescriptorTypes3D() {
  //   BOSS_REGISTER_CLASS(DescriptorMatcher3D);
  // }

} // namespace kd_slam
