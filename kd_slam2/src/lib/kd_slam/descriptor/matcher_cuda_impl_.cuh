#pragma once
#include "descriptor_impl_.h"
#include "cuda/prefix_sum_impl_.cuh"
#include "matcher_cuda_.cuh"


namespace kd_slam {
  namespace descriptor {
    using namespace kd_slam::cuda;

    // ---------------------------------------------------------------------------
    // Device kernel
    // ---------------------------------------------------------------------------
    template <typename MatcherType>
    __global__ void kdCompare_all_kernel(typename MatcherType::Match* out_matches,
                                         int* marks_ptr,
                                         const typename MatcherType::DescriptorType* descriptor_ptrs,
                                         const size_t num_descriptors,
                                         const typename MatcherType::QDescriptors queries,
                                         const typename MatcherType::Scalar max_distance,
                                         const typename MatcherType::Scalar normal_weight,
                                         CompareType cmp,
                                         int n) {
      int tid = threadIdx.x + blockIdx.x * blockDim.x;
      if (tid >= (int)num_descriptors) return;

      const auto& item = descriptor_ptrs[tid];
      auto& best_match=out_matches[tid];
      best_match={0,0,0,0,-1}; // invalid best match

      for(int c=0; c<MatcherType::DescriptorType::NumAxesCanonizations; ++c) {
        const auto& query=queries.des[c];
        typename MatcherType::Scalar d=0;
        switch (cmp) {
        case CompareType::MED:       d = query.medCompare(item, normal_weight, n); break;
        case CompareType::Euclidean: d = query.eucCompare(item, normal_weight, n); break;
        }
        if (best_match.q_canon<0 || d < best_match.dist_match) {
          best_match = {query.ref, item.ref, (size_t)tid, typename MatcherType::Scalar(sqrt(d)), query.axes_canonization};
        }
      }
      marks_ptr[tid] = 0;
      if (best_match.q_canon>=0 && best_match.dist_match<max_distance) {
        marks_ptr[tid] = 1;
      }
    }

    // ---------------------------------------------------------------------------
    // Device kernel
    // ---------------------------------------------------------------------------
    template <typename MatcherType>
    __global__ void kdCompare_selected_kernel(typename MatcherType::Match* out_matches,
                                              int* selected_list,
                                              const size_t num_selected,     
                                              const typename MatcherType::DescriptorType* descriptor_ptrs,
                                              const size_t num_descriptors,
                                              const typename MatcherType::QDescriptors queries,
                                              const typename MatcherType::Scalar max_distance,
                                              const typename MatcherType::Scalar normal_weight,
                                              CompareType cmp,
                                         int n) {
      int tid = threadIdx.x + blockIdx.x * blockDim.x;
      if (tid >= (int)num_selected) return;

      int idx=selected_list[tid];
      if (idx<0||idx>=num_descriptors) {
        selected_list[tid]=0;
        return;
      }
      const auto& item = descriptor_ptrs[idx];
      auto& best_match=out_matches[tid];
      best_match={0,0,0,0,-1}; // invalid best match

      for(int c=0; c<MatcherType::DescriptorType::NumAxesCanonizations; ++c) {
        const auto& query=queries.des[c];
        typename MatcherType::Scalar d=0;
        switch (cmp) {
        case CompareType::MED:       d = query.medCompare(item, normal_weight, n); break;
        case CompareType::Euclidean: d = query.eucCompare(item, normal_weight, n); break;
        }
        if (best_match.q_canon<0 || d < best_match.dist_match) {
          best_match = {query.ref, item.ref, (size_t)idx, typename MatcherType::Scalar(sqrt(d)), query.axes_canonization};
        }
      }
      selected_list[tid] = 0;
      if (best_match.q_canon>=0 && best_match.dist_match<max_distance) {
        selected_list[tid] = 1;
      }
    }
    

    // ---------------------------------------------------------------------------
    // MatcherCUDA_ method implementations
    // ---------------------------------------------------------------------------
    template <typename MatcherBase_>
    MatcherCUDA_<MatcherBase_>::MatcherCUDA_(size_t capacity)
      : _capacity(capacity) {
      CUDA_CHECK(cudaMalloc((void**)&_stored_desc_ptr, sizeof(DescriptorType) * _capacity));
      this->_descriptors_ptr = _stored_desc_ptr;
      CUDA_CHECK(cudaMalloc((void**)&_marks_ptr,   sizeof(int)     * _capacity));
      CUDA_CHECK(cudaMalloc((void**)&_matches_ptr, sizeof(Match) * _capacity * 2));
      _matches_src_ptr  = _matches_ptr;
      _matches_dest_ptr = _matches_ptr + _capacity;
      this->_num_descriptors = 0;
      _num_matches = 0;
      _prefix_adder.initDevice(_capacity);
    }

    template <typename MatcherBase_>
    MatcherCUDA_<MatcherBase_>::~MatcherCUDA_() {
      if (_stored_desc_ptr) CUDA_CHECK(cudaFree(_stored_desc_ptr));
      if (_matches_ptr)     CUDA_CHECK(cudaFree(_matches_ptr));
      if (_marks_ptr)       CUDA_CHECK(cudaFree(_marks_ptr));
    }

    template <typename MatcherBase_>
    void MatcherCUDA_<MatcherBase_>::addDescriptor(const DescriptorType& item, int ref) {
      if (this->_num_descriptors >= _capacity)
        throw std::runtime_error("KDDescriptorMatcherCUDA| capacity exceeded");
      DescriptorType d = item;
      d.ref = ref;
      CUDA_CHECK(cudaMemcpy(_stored_desc_ptr + this->_num_descriptors,
                            &d, sizeof(DescriptorType), cudaMemcpyHostToDevice));
      ++this->_num_descriptors;
    }

    template <typename MatcherBase_>
    void MatcherCUDA_<MatcherBase_>::clear() {
      this->_num_descriptors = 0;
      _num_matches = 0;
    }


    template <typename MatcherBase_>
    void MatcherCUDA_<MatcherBase_>::match(std::vector<Match>& matches,
                                           const QDescriptors& queries) const {
      const int n_desc    = this->_num_descriptors;
      const int n_threads = 1024;
      const int n_blocks  = (n_desc + n_threads - 1) / n_threads;
      _prefix_adder.initDevice(n_desc);
      kdCompare_all_kernel<MatcherBase><<<n_blocks, n_threads>>>(
                                                                 _matches_src_ptr, _marks_ptr, _stored_desc_ptr,
                                                                 n_desc, queries,
                                                                 params.max_distance * params.max_distance,
                                                                 params.normal_weight,
                                                                 params.cmp,
                                                                 params.comp_size);
      CUDA_CHECK(cudaMemcpy(_prefix_adder.ws, _marks_ptr,
                            sizeof(int) * n_desc, cudaMemcpyDeviceToDevice));
      _prefix_adder.compute();
      CUDA_scatterCompaction<<<n_blocks, n_threads>>>(
                                                      _matches_dest_ptr, _matches_src_ptr,
                                                      _prefix_adder.ws, _marks_ptr, n_desc);
      _num_matches = _prefix_adder.getMax();
      matches.resize(_num_matches);
      if (_num_matches) {
        CUDA_CHECK(cudaMemcpy(matches.data(), _matches_dest_ptr,
                              sizeof(Match) * _num_matches, cudaMemcpyDeviceToHost));
      }
    }

    template <typename MatcherBase_>
    void MatcherCUDA_<MatcherBase_>::match(std::vector<Match>& matches,
                                           const std::vector<int>& candidates,
                                           const QDescriptors& queries) const {
      const int n_out=candidates.size();
      const int n_desc    = this->_num_descriptors;
      const int n_threads = 1024;
      const int n_blocks  = (n_out + n_threads - 1) / n_threads;

      CUDA_CHECK(cudaMemcpy(_marks_ptr, candidates.data(),
                            sizeof(int) * n_out, cudaMemcpyHostToDevice));

      _prefix_adder.initDevice(n_out);
      kdCompare_selected_kernel<MatcherBase><<<n_blocks, n_threads>>>(
                                                                 _matches_src_ptr,
                                                                 _marks_ptr,
                                                                 n_out,
                                                                 _stored_desc_ptr,
                                                                 n_desc, queries,
                                                                 params.max_distance * params.max_distance,
                                                                 params.normal_weight,
                                                                 params.cmp,
                                                                 params.comp_size);
      CUDA_CHECK(cudaMemcpy(_prefix_adder.ws, _marks_ptr,
                            sizeof(int) * n_out, cudaMemcpyDeviceToDevice));
      _prefix_adder.compute();
      CUDA_scatterCompaction<<<n_blocks, n_threads>>>(
                                                      _matches_dest_ptr, _matches_src_ptr,
                                                      _prefix_adder.ws, _marks_ptr, n_out);
      _num_matches = _prefix_adder.getMax();
      matches.resize(_num_matches);
      if (_num_matches) {
        CUDA_CHECK(cudaMemcpy(matches.data(), _matches_dest_ptr,
                              sizeof(Match) * _num_matches, cudaMemcpyDeviceToHost));
      }
      
    }

  }
} // namespace kd_slam::descriptor
