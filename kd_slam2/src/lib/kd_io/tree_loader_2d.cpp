#include "srrg_boss/serializable.h"
#include "kd_io/tree_loader_.h"
#include "kd_io/tree_loader_impl_.h"
#include "kd_slam/d2/typedefs.h"
namespace kd_slam {

  template struct TreeLoader_<d2::NodeType>;
  using TreeLoader2D    = TreeLoader_<d2::NodeType>;

  void __attribute__((constructor)) kd_io_registerTypes2D() {
    BOSS_REGISTER_CLASS(TreeLoader2D);
  }

} // namespace kd_slam
