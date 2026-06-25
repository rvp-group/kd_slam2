#include "descriptor_defs.h"
#include "matcher_cuda_impl_.cuh"
#include "utils/geometry_2d_.h"
#include "utils/geometry_3d_.h"


namespace kd_slam {
  namespace descriptor {

    template struct MatcherCUDA_<MatcherPoint3f_<3>>;
    template struct MatcherCUDA_<MatcherPoint3f_<4>>;
    template struct MatcherCUDA_<MatcherPoint3f_<5>>;

  }
} // namespace kd_slam::descriptor
