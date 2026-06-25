#pragma once
#include "cuda/cuda_common.h"
#include "tree_cuda_.h"
#include "tree_predicates_.h"
#include "tree_cpu_.h"

namespace kd_slam {
  using namespace kd_slam::cuda;

  template<typename KDTreeBase_>
  TreeCUDA_<KDTreeBase_>::TreeCUDA_(const KDTreeBase_& src): 
    KDTreeBase_(src){
    if(src.isGPU()) {
      const TreeCUDA_<KDTreeBase_>& cuda_tree=reinterpret_cast<const TreeCUDA_<KDTreeBase_>&>(src);
      fromCUDA(cuda_tree);
    } else {
      const TreeCPU_<KDTreeBase_>& cpu_tree=reinterpret_cast<const TreeCPU_<KDTreeBase_>&>(src);
      fromCPU(cpu_tree);
    }
  }

  template<typename KDTreeBase_>
  void TreeCUDA_<KDTreeBase_>::fromCPU(const TreeCPUType& host_src) {
    _num_nodes=host_src._num_nodes;
    
    // allocate the nodes
    CUDA_CHECK(cudaMalloc((void**)&_nodes_ptr,
                          sizeof(NodeType)*_num_nodes));
    
    CUDA_CHECK(cudaMemcpy(_nodes_ptr,
                          host_src._nodes_ptr,
                          sizeof(NodeType)*host_src._num_nodes,
                          cudaMemcpyHostToDevice));
    _num_leaves=host_src._num_leaves;
    _leaves_indices_ptr=0;
    if (_num_leaves) {
      CUDA_CHECK(cudaMalloc((void**)&_leaves_indices_ptr,
                            sizeof(size_t)*_num_leaves));
      CUDA_CHECK(cudaMemcpy(_leaves_indices_ptr,
                            host_src._leaves_indices_ptr,
                            sizeof(size_t)*_num_leaves,
                            cudaMemcpyHostToDevice));
    } 
  }
  

  template<typename KDTreeBase_>
  void TreeCUDA_<KDTreeBase_>::fromCUDA(const TreeCUDAType& device_src){
    _num_nodes=device_src._num_nodes;
    _num_points=0;
    _points_ptr=0;
    CUDA_CHECK(cudaMalloc((void**)&_nodes_ptr, sizeof(NodeType)*device_src._num_nodes));
    CUDA_CHECK(cudaMemcpy(_nodes_ptr, device_src._nodes_ptr, sizeof(NodeType)*_num_nodes, cudaMemcpyDeviceToDevice));

    _leaves_indices_ptr=0;
    _num_leaves=device_src._num_leaves;
    if (_num_leaves) {
      CUDA_CHECK(cudaMalloc((void**)&_leaves_indices_ptr,
                            sizeof(size_t)*_num_leaves));
      CUDA_CHECK(cudaMemcpy(_leaves_indices_ptr,
                            device_src._leaves_indices_ptr,
                            sizeof(size_t)*_num_leaves,
                            cudaMemcpyDeviceToDevice));
    }
  
  }

  template<typename KDTreeBase_>
  TreeCUDA_<KDTreeBase_>::~TreeCUDA_() {
    if (_nodes_ptr) {
      CUDA_CHECK(cudaFree(_nodes_ptr));
      if(_leaves_indices_ptr) {
        CUDA_CHECK(cudaFree(_leaves_indices_ptr));
      }
      _nodes_ptr=0;
      _leaves_indices_ptr=0;
    }
  }

  template <typename KDTreeBase_>
  __global__ void extractCompressedLeaves_kernel(typename KDTreeBase_::VectorType* compressed_leaves,
                                                 const typename KDTreeBase_::NodeType* nodes,
                                                 size_t num_nodes,
                                                 const size_t* leaves_indices_ptr,
                                                 size_t num_leaves) {
    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    if (tid >= (int)num_leaves) return;
    int idx=leaves_indices_ptr[tid];
    if (idx>= (int)num_nodes) return;
    const auto& n = nodes[idx];
    int dest_idx=2*tid;
    compressed_leaves[dest_idx]=n._mean;
    compressed_leaves[dest_idx+1]=n._direction;
  }

  
  template<typename KDTreeBase_>
  void TreeCUDA_<KDTreeBase_>::copyNodes(Tree_<NodeType>& other) const  {
    if (!other.isGPU())
      throw std::runtime_error("compute incompatible");
    if (other._num_nodes!=_num_nodes)
      throw std::runtime_error("size mismatch");
    CUDA_CHECK(cudaMemcpy(other._nodes_ptr,
                          _nodes_ptr,
                          sizeof(NodeType)*_num_nodes,
                          cudaMemcpyDeviceToDevice));
  }


  template <typename KDTreeBase_>
  std::vector<typename KDTreeBase_::VectorType>
  TreeCUDA_<KDTreeBase_>::extractCompressedLeaves() const {
    if (! this->_num_leaves  || ! this->_leaves_indices_ptr)
      return std::vector<VectorType>();
    std::vector<VectorType> returned(this->_num_leaves*2);

    int buffer_size=2*sizeof(VectorType)*this->_num_leaves;
    VectorType* comp_leaves;
    CUDA_CHECK(cudaMalloc((void**)&comp_leaves,
                          buffer_size));
    
    constexpr int n_threads = 256;
    const int n_blocks = (this->_num_leaves + n_threads - 1) / n_threads;
    extractCompressedLeaves_kernel<KDTreeBase_><<<n_blocks, n_threads>>>
      (comp_leaves, this->_nodes_ptr, this->_num_nodes, this->_leaves_indices_ptr, this->_num_leaves);
    
    CUDA_CHECK(cudaMemcpy(&(returned[0]),
                          comp_leaves,
                          buffer_size,
                          cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaFree(comp_leaves));
    return returned;
  }

  template <typename KDTreeBase_>
  __global__ void extractCompressedLeavesVel_kernel(typename KDTreeBase_::VectorType* compressed_leaves,
                                                    const typename KDTreeBase_::NodeType* nodes,
                                                    size_t num_nodes,
                                                    const size_t* leaves_indices_ptr,
                                                    size_t num_leaves,
                                                    typename KDTreeBase_::VelocityVectorType v) {
    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    if (tid >= (int)num_leaves) return;
    int idx = leaves_indices_ptr[tid];
    if (idx >= (int)num_nodes) return;
    const auto n = nodes[idx].applyVelocities(v);
    int dest_idx = 2*tid;
    compressed_leaves[dest_idx]   = n._mean;
    compressed_leaves[dest_idx+1] = n._direction;
  }

  template <typename KDTreeBase_>
  std::vector<typename KDTreeBase_::VectorType>
  TreeCUDA_<KDTreeBase_>::extractCompressedLeaves(const VelocityVectorType& v) const {
    if (!this->_num_leaves || !this->_leaves_indices_ptr)
      return std::vector<VectorType>();
    std::vector<VectorType> returned(this->_num_leaves*2);
    int buffer_size = 2*sizeof(VectorType)*this->_num_leaves;
    VectorType* comp_leaves;
    CUDA_CHECK(cudaMalloc((void**)&comp_leaves, buffer_size));
    constexpr int n_threads = 256;
    const int n_blocks = (this->_num_leaves + n_threads - 1) / n_threads;
    extractCompressedLeavesVel_kernel<KDTreeBase_><<<n_blocks, n_threads>>>
      (comp_leaves, this->_nodes_ptr, this->_num_nodes, this->_leaves_indices_ptr, this->_num_leaves, v);
    CUDA_CHECK(cudaMemcpy(&(returned[0]), comp_leaves, buffer_size, cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaFree(comp_leaves));
    return returned;
  }

  template <typename KDTreeBase_>
  __global__ void treeApplyVelocities_kernel(typename KDTreeBase_::NodeType* nodes,
                                             size_t num_nodes,
                                             typename KDTreeBase_::VelocityVectorType v,
                                             bool skip_leaves) {
    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    if (tid >= (int)num_nodes) return;
    auto& n = nodes[tid];
    if (n.isLeaf() && skip_leaves) return;
    n = n.applyVelocities(v);
  }

    template <typename KDTreeBase_>
  __global__ void treeApplyIsometry_kernel(typename KDTreeBase_::NodeType* nodes,
                                           size_t num_nodes,
                                           typename KDTreeBase_::IsometryType iso) {
    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    if (tid >= (int)num_nodes) return;
    auto& n = nodes[tid];
    n=applyIsometry_(iso, n);
  }

  template <typename KDTreeBase_>
  void TreeCUDA_<KDTreeBase_>::applyVelocitiesInPlace(const VelocityVectorType& v, bool skip_leaves) {
    if (!this->_num_nodes) return;
    constexpr int n_threads = 256;
    const int n_blocks = (this->_num_nodes + n_threads - 1) / n_threads;
    treeApplyVelocities_kernel<KDTreeBase_><<<n_blocks, n_threads>>>(
      this->_nodes_ptr, this->_num_nodes, v, skip_leaves);
  }

    template <typename KDTreeBase_>
  void TreeCUDA_<KDTreeBase_>::applyIsometryInPlace(const IsometryType& iso) {
      root_mean=iso*root_mean;
      root_eigenvectors=iso.linear()*root_eigenvectors;
      if (!this->_num_nodes) return;
    constexpr int n_threads = 256;
    const int n_blocks = (this->_num_nodes + n_threads - 1) / n_threads;
    treeApplyIsometry_kernel<KDTreeBase_><<<n_blocks, n_threads>>>(
      this->_nodes_ptr, this->_num_nodes, iso);
    
  }

} // namespace kd_slam
