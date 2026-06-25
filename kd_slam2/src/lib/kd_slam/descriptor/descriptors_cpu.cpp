#include "descriptor_defs.h"
#include "descriptor_impl_.h"
#include "matcher_cpu_impl_.h"
#include "extractor_impl_.h"
#include "utils/geometry_2d_.h"
#include "utils/geometry_3d_.h"


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
} // namespace kd_slam::descriptor
