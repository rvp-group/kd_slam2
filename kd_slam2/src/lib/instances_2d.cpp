#include "instances_2d.h"
#include "srrg_boss/serializable.h"
namespace kd_slam {
  
  void registerCPUTypes2D();
  void registerTypes2D();
}

namespace kd_io {
  void registerTypes2D();
}
  
void kd_slam_registerTypes2D() {
  kd_slam::registerCPUTypes2D();
  kd_io::registerTypes2D();
}
