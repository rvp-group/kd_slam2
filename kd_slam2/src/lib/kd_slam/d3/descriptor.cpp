#include "srrg_boss/serializable.h"
#include "kd_slam/descriptor/descriptor_defs.h"
#include "kd_slam/descriptor/descriptor_impl_.h"
#include "kd_slam/descriptor/matcher_cpu_impl_.h"
#include "kd_slam/descriptor/extractor_impl_.h"
#include "kd_slam/d3/typedefs.h"

namespace kd_slam {
    namespace descriptor {


    // 3D - levels 3, 4, 5
    template struct Descriptor_<NodePoint3f, (1<<3)-2>;
    template struct Descriptor_<NodePoint3f, (1<<4)-2>;
    template struct Descriptor_<NodePoint3f, (1<<5)-2>;

    template struct Extractor_<TreeCPUPoint3f, 3>;
    template struct Extractor_<TreeCPUPoint3f, 4>;
    template struct Extractor_<TreeCPUPoint3f, 5>;

    template struct MatcherCPU_<MatcherPoint3f_<3>>;
    template struct MatcherCPU_<MatcherPoint3f_<4>>;
    template struct MatcherCPU_<MatcherPoint3f_<5>>;

  }
  
  using DescriptorMatcher3D = descriptor::MatcherCPU_<d3::MatcherPoint3f>;

  void registerDescriptorTypes3D() {
    using namespace std;
    BOSS_REGISTER_CLASS(DescriptorMatcher3D);
  }

} // namespace kd_slam
