namespace kd_slam {
  
  void registerCPUTypes3D();
  void registerICPCUDATypes3D();
  void registerCTICPCUDATypes3D();

  void __attribute__((constructor)) registerCUDATypes3D() {
    kd_slam::registerCPUTypes3D();
    kd_slam::registerICPCUDATypes3D();
    kd_slam::registerCTICPCUDATypes3D();
  }

}

