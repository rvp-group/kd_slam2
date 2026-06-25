namespace kd_slam {
  
  void kd_slam_registerICPCUDATypes2D();
  void kd_slam_registerCTICPCUDATypes2D();
  void kd_slam_registerCPUTypes2D();
  
  void  __attribute__((constructor)) kd_slam_registerCUDATypes2D() {
    kd_slam_registerCPUTypes2D();
    kd_slam_registerICPCUDATypes2D();
    kd_slam_registerCTICPCUDATypes2D();
  }
}
