namespace kd_slam {
  
  void kd_slam_registerCPUTypes3D();
  void kd_slam_registerICPCUDATypes3D();
  void kd_slam_registerCTICPCUDATypes3D();

  void __attribute__((constructor)) kd_slam_registerCUDATypes3D() {
    kd_slam_registerCPUTypes3D();
    kd_slam_registerICPCUDATypes3D();
    kd_slam_registerCTICPCUDATypes3D();
  }
}
