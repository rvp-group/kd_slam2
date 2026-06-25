#pragma once
#ifdef HAVE_CUDA
#include <cuda_runtime.h>
#endif

#include <cstdio>
#include <cstdlib>

#ifdef __CUDACC__
#  pragma diag_suppress 20013
#  pragma diag_suppress 20015
#  define __CUDA_EXPORT_INLINE__ __host__ __device__ __forceinline__
#else
#  define __CUDA_EXPORT_INLINE__ inline
#endif

namespace kd_slam::cuda{

  __CUDA_EXPORT_INLINE__ size_t roundUp(int idx, int block_size) {
    return idx/block_size + ((idx%block_size)?1:0);
  };
}

#ifdef HAVE_CUDA

#define CUDA_CHECK(err) (HandleError(err, __FILE__, __LINE__))

namespace kd_slam::cuda{

  inline void HandleError(cudaError_t err, const char* file, int line) {
    if (err != cudaSuccess) {
      printf("%s in %s at line %d\n", cudaGetErrorString(err), file, line);
      exit(EXIT_FAILURE);
    }
  }

  inline bool isGPUPtr(const void* v) {
    cudaPointerAttributes attributes;
    cudaError_t error = cudaPointerGetAttributes(&attributes, v);
    return error==cudaSuccess;
  }

}
#endif
