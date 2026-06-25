#pragma once
#include "cuda_common.h"

namespace kd_slam::cuda {
  
  template <typename ValueType_, int ChunkSize=8>
  struct PrefixSum_ {
    using ValueType=ValueType_;
    using ThisType = PrefixSum_<ValueType_>;
    static constexpr int MAX_LEVELS=32;
    int _num_elements=0;
    int _num_levels=0;
    int _h_offsets[MAX_LEVELS];
    int _ws_num_elements=0;
    int _ws_capacity=0;
    cudaStream_t _stream;
    //before compute the first _num_elements in ws contain the source 
    //after compute these elements are overwritten with the answer
    ValueType* ws=0;

    ThisType*  device_self=0;
  
    __CUDA_EXPORT_INLINE__ int wsOffset(int level) const {
      return (level==0)? 0 : _h_offsets[level-1];
    }

    __CUDA_EXPORT_INLINE__ int wsSize(int level) const {
      return (level==0)?  _h_offsets[level] : _h_offsets[level] -_h_offsets[level-1];
    }

  
  
    PrefixSum_(cudaStream_t stream=0);
    void setStream(cudaStream_t stream_) {_stream=stream_;}
    ~PrefixSum_();
  
    //to be called before, the device functions to prepare the buffers
    void initDevice(int src_size);
  
    void setSource(const ValueType* host_src, int src_size);
  
    void getResult(ValueType* host_dest);
  
    ValueType getMax() const;

    void compute();
  
  protected:
    void prepareWorkspace(int num_elements);

  };
}
