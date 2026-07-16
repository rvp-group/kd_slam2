#include "srrg_boss/serializable.h"
#include "kd_io/tree_loader_.h"
#include "kd_slam/d2/typedefs.h"

namespace kd_slam {

  using TreeGenerator3D = TreeGenerator_<d3::NodeType>;
  using Voxelizer3D     = utils::Voxelizer_<d3::PointTraits>;
  using TreeLoader3D    = TreeLoader_<d3::NodeType>;

  void registerTreeTypes3D() {
    BOSS_REGISTER_CLASS(TreeGenerator3D);
    BOSS_REGISTER_CLASS(Voxelizer3D);
    BOSS_REGISTER_CLASS(TreeLoader3D);
  }

} // namespace kd_slam
