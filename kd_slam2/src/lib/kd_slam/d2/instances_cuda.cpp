namespace kd_slam {
  
  void registerICPCUDATypes2D();
  void registerCTICPCUDATypes2D();
  void registerCPUTypes2D();
  
  void  __attribute__((constructor)) registerCUDATypes2D() {
    kd_slam::registerCPUTypes2D();
    kd_slam::registerICPCUDATypes2D();
    kd_slam::registerCTICPCUDATypes2D();
  }
  
}

