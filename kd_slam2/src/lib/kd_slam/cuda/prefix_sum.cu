#include "cuda_common.h"
namespace kd_slam::cuda {
  
  __global__ void  CUDA_computeCompactionVector (int* dest, const int* src, int num_nodes) {
    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    if (tid>=num_nodes)
      return;
    int pos=src[tid]-1;
    if (pos<0)
      return;
    if (tid==0 || src[tid]>src[tid-1]) {
      dest[pos]=tid;
    }
  }
}
