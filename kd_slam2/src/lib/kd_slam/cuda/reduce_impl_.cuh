#pragma once
#include "reduce_.h"
#include <iostream>

namespace  kd_slam::cuda {
  using namespace std;


  template <typename T, typename Op, int ChunkSize=16, int MaxThreads=1024>
  __global__ __launch_bounds__(MaxThreads) void _reduce1(T* dest, T* src, int size) {
    const Op op;
    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    int start_idx=tid*ChunkSize;
    int end_idx=(tid+1)*ChunkSize < size ? (tid+1)*ChunkSize:size;
    if (start_idx>=size)
      return;
    T acc=src[start_idx];
    for (int i=start_idx+1; i<end_idx; ++i) {
      op(acc,src[i]);
    }
    dest[tid]=acc;
    // printf("reduce1 dest: %d src_start: %d src_end: %d",
    //        tid, start_idx, end_idx);
  }

  template <typename T, int ChunkSize, int MaxThreads>
  template <typename Op>
  void Reduce_<T,ChunkSize,MaxThreads>::reduceDeferred(T* input, int num_items) {
    using namespace std;
    init(num_items);
    _deferred_buffer=0;
    int size=num_items;
    T* src=input;
    T* dest=ws;
    while (size>1) {
      int n_size=roundUp(size, ChunkSize);
      int n_blocks  = (n_size + MaxThreads - 1) / MaxThreads;
      _reduce1<T, Op, ChunkSize, MaxThreads><<<n_blocks, MaxThreads>>>(dest, src,size);
      std::swap(src, dest);
      size=n_size;
    }
    _deferred_buffer=src;
  }

  template <typename T, int ChunkSize, int MaxThreads>
  template <typename Op>
  T Reduce_<T,ChunkSize,MaxThreads>::reduce(T* input, int num_items) {
    reduceDeferred<Op>(input, num_items);
    return getDeferredResult();
  }


  template <typename T, int ChunkSize, int MaxThreads>
  void Reduce_<T,ChunkSize,MaxThreads>::init(int num_items) {
    int new_ws_size=roundUp(num_items, ChunkSize);
    if (ws_capacity<new_ws_size) {
      if (ws) {
        CUDA_CHECK(cudaFree(ws));
      }
      CUDA_CHECK(cudaMalloc((void**)&ws, sizeof(T)*new_ws_size));
      ws_capacity=new_ws_size;
    }
  }
  
  template <typename T, int ChunkSize, int MaxThreads>
  Reduce_<T,ChunkSize,MaxThreads>::~Reduce_() {
    if (ws)
      CUDA_CHECK(cudaFree(ws));
    ws=0;
    ws_capacity=0;
  }

  template <typename T, int ChunkSize, int MaxThreads>
  T Reduce_<T,ChunkSize,MaxThreads>::getDeferredResult() {
    T out;
    CUDA_CHECK(cudaMemcpy(&out, _deferred_buffer, sizeof(T), cudaMemcpyDeviceToHost));
    return out;
  }
  
}
