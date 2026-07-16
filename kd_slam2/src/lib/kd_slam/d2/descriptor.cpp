#include "srrg_boss/serializable.h"
#include "kd_slam/descriptor/descriptor_defs.h"
#include "kd_slam/descriptor/descriptor_impl_.h"
#include "kd_slam/descriptor/matcher_cpu_impl_.h"
#include "kd_slam/descriptor/extractor_impl_.h"
#include "kd_slam/d2/typedefs.h"

namespace kd_slam {
  namespace descriptor {

    // 2D - levels 3, 4, 5
    template struct Descriptor_<NodePoint2f, (1<<3)-2>;
    template struct Descriptor_<NodePoint2f, (1<<4)-2>;
    template struct Descriptor_<NodePoint2f, (1<<5)-2>;

    template struct Extractor_<TreeCPUPoint2f, 3>;
    template struct Extractor_<TreeCPUPoint2f, 4>;
    template struct Extractor_<TreeCPUPoint2f, 5>;

    template struct MatcherCPU_<MatcherPoint2f_<3>>;
    template struct MatcherCPU_<MatcherPoint2f_<4>>;
    template struct MatcherCPU_<MatcherPoint2f_<5>>;

  }

  using DescriptorMatcher2D = descriptor::MatcherCPU_<d2::MatcherPoint2f>;
  void registerDescriptorTypes2D() {
    BOSS_REGISTER_CLASS(DescriptorMatcher2D);
  }

} // namespace kd_slam
