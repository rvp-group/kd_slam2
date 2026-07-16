#include "kd_slam/cuda/cuda_common.h"

void __attribute__((constructor)) kd_slam_cuda_init() {
  using namespace kd_slam::cuda;
  CUDA_CHECK(cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync));
}
