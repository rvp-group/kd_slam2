#include "instances_3d.h"
#include "srrg_boss/serializable.h"

namespace kd_slam {
  void registerCPUTypes3D();
  void registerTypes3D();
}

namespace kd_io{
  void registerTypes3D();
}
  
void kd_slam_registerTypes3D() {
  kd_slam::registerCPUTypes3D();
  kd_io::registerTypes3D();
}
