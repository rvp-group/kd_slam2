namespace kd_slam {
  
  void registerTreeTypes2D();
  void registerDescriptorTypes2D();
  void registerICPCPUTypes2D();
  void registerCTICPCPUTypes2D();
  void registerSLAMTypes2D();

  void  __attribute__((constructor)) registerCPUTypes2D() {
    kd_slam::registerTreeTypes2D();
    kd_slam::registerDescriptorTypes2D();
    kd_slam::registerICPCPUTypes2D();
    kd_slam::registerCTICPCPUTypes2D();
    kd_slam::registerSLAMTypes2D();
  }

}

