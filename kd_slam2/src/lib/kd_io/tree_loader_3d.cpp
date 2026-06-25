#include "srrg_boss/serializable.h"
#include "kd_io/tree_loader_.h"
#include "kd_io/tree_loader_impl_.h"
#include "kd_slam/d3/typedefs.h"
namespace kd_slam {

  template struct TreeLoader_<d3::NodeType>;
  using TreeLoader3D    = TreeLoader_<d3::NodeType>;

  void __attribute__((constructor)) kd_io_registerTypes3D() {
    BOSS_REGISTER_CLASS(TreeLoader3D);
  }

} // namespace kd_slam
