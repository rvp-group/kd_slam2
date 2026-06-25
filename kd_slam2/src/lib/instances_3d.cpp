#include "instances_3d.h"
#include "srrg_boss/serializable.h"
namespace kd_slam {

  
  void kd_slam_registerCPUTypes3D();
  void kd_io_registerTypes3D();

#ifdef HAVE_CUDA
  void kd_slam_registerCUDATypes3D();
#endif

  
  void kd_slam_registerTypes3D() {
    kd_slam_registerCPUTypes3D();
    kd_io_registerTypes3D();
#ifdef HAVE_CUDA
    kd_slam_registerCUDATypes3D();
#endif
  }

} // namespace kd_slam
