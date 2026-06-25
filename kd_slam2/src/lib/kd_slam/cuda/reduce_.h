#pragma once
#include "cuda_common.h"
#include <iostream>
namespace kd_slam::cuda {
  
  template <typename T>
  struct OpSum{
    __CUDA_EXPORT_INLINE__
    void operator()(T& a, const T& b) const {
      a+=b;
    }
  };

  template <typename T>
  struct OpMax{
    __CUDA_EXPORT_INLINE__
    void operator()(T& a, const T& b) const {
      a = (a>b)?a:b;
    }
  };

  template <typename T>
  struct OpMin{
    __CUDA_EXPORT_INLINE__
    void operator()(T& a, const T& b) const {
      a = (a<b)?a:b;
    }
  };


  /**
     This struct implements a reduction algorithm, that
     can be used for max, min, or accumulation.
     To use:
     1. Create an instance specifying the type of the item.
     The template arguments are:
     - The type
     - The size of the reduction at every round
      
     2. initialize the buffer by calling the init method
     3. call reduce<operator>(src, size). This returns the value
  */

  template <typename T, int ChunkSize=16, int MaxThreads=1024>
  struct Reduce_{
    void init(int num_items);
    template <typename Op>
    T reduce(T* input, int num_items);

    template <typename Op>
    void reduceDeferred(T* input, int num_items);

    T getDeferredResult();
  
    ~Reduce_();

    T* ws=0;
    int ws_capacity=0;
    T* _deferred_buffer=0;
  };
}
