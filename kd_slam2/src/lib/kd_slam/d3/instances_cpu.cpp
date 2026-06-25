namespace kd_slam {
  
  void kd_slam_registerTreeTypes3D();
  void kd_slam_registerDescriptorTypes3D();
  void kd_slam_registerICPCPUTypes3D();
  void kd_slam_registerCTICPCPUTypes3D();
  void kd_slam_registerSLAMTypes3D();

  
  void __attribute__((constructor)) kd_slam_registerCPUTypes3D() {
    kd_slam_registerTreeTypes3D();
    kd_slam_registerDescriptorTypes3D();
    kd_slam_registerICPCPUTypes3D();
    kd_slam_registerCTICPCPUTypes3D();
    kd_slam_registerSLAMTypes3D();
  }
}
