#pragma once
#include "prefix_sum_.h"
//#include "cuda_utils.h"
namespace kd_slam::cuda {  
  __global__
  void  CUDA_computeCompactionVector(int* dest, const int* src, int num_nodes);

  template <typename ValueType, int ChunkSize>
  __global__ void PrefixSum_forward(ValueType* ws,
                                    //int level,
                                    int lower_offset,
                                    int lower_size,
                                    int upper_offset,
                                    int upper_size){
    //int upper_size=context->wsSize(level+1);
    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    if (tid>=upper_size)
      return;
  
    ValueType* lower=ws + lower_offset;
    ValueType* upper=ws + upper_offset+tid;

    ValueType acc=lower[tid*ChunkSize];
    const int start_idx=tid*ChunkSize+1;
    const int end_idx=min((tid+1)*ChunkSize, lower_size);
    for (int i=start_idx; i<end_idx; ++i) {
      acc += lower[i];
      lower [i] = acc;
    }
  
    *upper=acc;
  }


  template <typename ValueType, int ChunkSize>
  __global__ void PrefixSum_backward(ValueType* ws,
                                     int lower_offset,
                                     int lower_size,
                                     int upper_offset,
                                     int upper_size){
    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    if (tid==0 || tid>= upper_size)
      return;
    ValueType* lower=ws+lower_offset;
    const ValueType base=ws[upper_offset+tid-1];
    const int end_idx=min((tid+1)*ChunkSize, lower_size);
    for (int i=tid*ChunkSize; i<end_idx; ++i)
      lower[i]+=base;
  }

  template <typename ValueType_, int ChunkSize>
  void PrefixSum_<ValueType_, ChunkSize>::prepareWorkspace(int num_elements) {
    if (_num_elements==num_elements)
      return;

    memset(_h_offsets, 0, sizeof(int)*MAX_LEVELS);
    _num_elements=num_elements;
    
    // compute the size of the workspace
    int level=0;
    int ws_size=_num_elements;
    int level_size=_num_elements;
    while (level_size>1) {
      _h_offsets[level]=ws_size;
      level_size=roundUp(level_size, ChunkSize);
      ws_size+=level_size;
      ++level;
    }
    _h_offsets[level]=ws_size;
    _num_levels=level;
    _ws_num_elements=ws_size;
  }
  
  template <typename ValueType_, int ChunkSize>
  PrefixSum_<ValueType_, ChunkSize>::PrefixSum_(cudaStream_t stream_){
    _stream=stream_;
    CUDA_CHECK(cudaMalloc((void**) &device_self, sizeof(ThisType)));
  }
  
  template <typename ValueType_, int ChunkSize>
  void PrefixSum_<ValueType_, ChunkSize>::initDevice(int src_size){
    prepareWorkspace(src_size);
    if (_ws_capacity<_ws_num_elements) {
      if (ws)
        CUDA_CHECK(cudaFree(ws));
      _ws_capacity=_ws_num_elements;
      CUDA_CHECK(cudaMalloc((void**) &ws, sizeof(ValueType)*_ws_capacity));
      //cerr << __PRETTY_FUNCTION__ << "| ws_num_elements: " << _ws_num_elements << " num_levels: " << _num_levels << endl;
    }

    //cerr << "ws_size: " << _ws_num_elements << " capacity: " << _ws_capacity << endl;
    CUDA_CHECK(cudaMemcpyAsync(device_self,
                               this,
                               sizeof(ThisType),
                               cudaMemcpyHostToDevice, _stream));
  }

  template <typename ValueType_, int ChunkSize>
  void PrefixSum_<ValueType_, ChunkSize>::setSource(const ValueType* src, int src_size) {
    if (src_size!=_num_elements)
      throw std::runtime_error("mismatch in number of elements");
    CUDA_CHECK(cudaMemcpyAsync(ws, src,sizeof(ValueType)*_num_elements, cudaMemcpyHostToDevice, _stream));
  };
  
  template <typename ValueType_, int ChunkSize>
  void PrefixSum_<ValueType_, ChunkSize>::getResult(ValueType* dest) {
    CUDA_CHECK(cudaMemcpyAsync(dest, ws,sizeof(ValueType)*_num_elements, cudaMemcpyDeviceToHost, _stream));
  };
  
  template <typename ValueType_, int ChunkSize>
  ValueType_ PrefixSum_<ValueType_, ChunkSize>::getMax() const {
    ValueType val;
    CUDA_CHECK(cudaMemcpyAsync(&val, ws+_num_elements-1,sizeof(ValueType), cudaMemcpyDeviceToHost, _stream));
    return val;
  };
  
  template <typename ValueType_, int ChunkSize>
  void PrefixSum_<ValueType_, ChunkSize>::compute() {
    for (int l=0; l<_num_levels; ++l) {
      int num_items=wsSize(l+1);
      const int n_threads = 1024;
      const int n_blocks  = (num_items + n_threads - 1) / n_threads;
      PrefixSum_forward<ValueType, ChunkSize><<<n_blocks, n_threads, 0, _stream>>>(ws,
                                                                                   wsOffset(l),
                                                                                   wsSize(l),
                                                                                   wsOffset(l+1),
                                                                                   wsSize(l+1));
    }
    for (int l=_num_levels-1; l>=0; --l) {
      int num_items=wsSize(l+1);
      const int n_threads = 1024;
      const int n_blocks  = (num_items + n_threads - 1) / n_threads;
      PrefixSum_backward<ValueType, ChunkSize><<<n_blocks, n_threads, 0, _stream>>>(ws,
                                                                                    wsOffset(l),
                                                                                    wsSize(l),
                                                                                    wsOffset(l+1),
                                                                                    wsSize(l+1));
    }
  }

  template <typename ValueType_, int ChunkSize>
  PrefixSum_<ValueType_, ChunkSize>::~PrefixSum_() {
    CUDA_CHECK(cudaFree(ws));
    CUDA_CHECK(cudaFree(device_self));
    device_self=0;
    ws=0;
  }
  template <typename T>
  __global__ void CUDA_scatterCompaction(T* dest, const T* src,
                                         const int* prefix, const int* mask, int N) {
    int i = threadIdx.x + blockIdx.x * blockDim.x;
    if (i >= N) return;
    if (mask[i]) dest[prefix[i]-1] = src[i];
  }
}
