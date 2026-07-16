namespace kd_slam {
  
  void registerTreeTypes3D();
  void registerDescriptorTypes3D();
  void registerICPCPUTypes3D();
  void registerCTICPCPUTypes3D();
  void registerSLAMTypes3D();
  
  void __attribute__((constructor)) registerCPUTypes3D() {
    registerTreeTypes3D();
    registerDescriptorTypes3D();
    registerICPCPUTypes3D();
    registerCTICPCPUTypes3D();
    registerSLAMTypes3D();
  }

}

  
