namespace kd_slam {
  
  void kd_slam_registerTreeTypes2D();
  void kd_slam_registerDescriptorTypes2D();
  void kd_slam_registerICPCPUTypes2D();
  void kd_slam_registerCTICPCPUTypes2D();
  void kd_slam_registerSLAMTypes2D();

  void  __attribute__((constructor)) kd_slam_registerCPUTypes2D() {
    kd_slam_registerTreeTypes2D();
    kd_slam_registerDescriptorTypes2D();
    kd_slam_registerICPCPUTypes2D();
    kd_slam_registerCTICPCPUTypes2D();
    kd_slam_registerSLAMTypes2D();
  }
}
