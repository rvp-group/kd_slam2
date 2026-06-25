#include "instances_2d.h"
#include "srrg_boss/serializable.h"
namespace kd_slam {

  
  void kd_slam_registerCPUTypes2D();
  void kd_io_registerTypes2D();

#ifdef HAVE_CUDA
  void kd_slam_registerCUDATypes2D();
#endif

  
  void kd_slam_registerTypes2D() {
    kd_slam_registerCPUTypes2D();
    kd_io_registerTypes2D();
#ifdef HAVE_CUDA
    kd_slam_registerCUDATypes2D();
#endif
  }

} // namespace kd_slam
