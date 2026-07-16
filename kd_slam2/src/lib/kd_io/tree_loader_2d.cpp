#include "srrg_boss/serializable.h"
#include "kd_io/tree_loader_.h"
#include "kd_io/tree_loader_impl_.h"
#include "kd_slam/d2/typedefs.h"
namespace kd_slam {

  template struct TreeLoader_<d2::NodeType>;
  using TreeLoader2D    = TreeLoader_<d2::NodeType>;

} // namespace kd_slam

namespace kd_io {
  void __attribute__((constructor)) registerTypes2D() {
    using namespace kd_slam;
    BOSS_REGISTER_CLASS(TreeLoader2D);
  }
}
