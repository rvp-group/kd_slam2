#pragma once
#include "cuda/cuda_common.h"
#include "tree_cuda_.h"
#include "tree_predicates_.h"
#include "tree_cpu_.h"
#include "tree_convert_.h"

namespace kd_slam {
  using namespace kd_slam::cuda;

  template<typename KDTreeBase_>
  TreeCUDA_<KDTreeBase_>::TreeCUDA_(const KDTreeBase_& src): 
    KDTreeBase_(src){
    Tree_copyTo<KDTreeBase_>(*this, src);
  }

  template<typename KDTreeBase_>
  TreeCUDA_<KDTreeBase_>::TreeCUDA_(const TreeCUDA_<KDTreeBase_>& src): 
    KDTreeBase_(src){
    Tree_CUDAfromCUDA_(*this, src);
  }

  
  template<typename KDTreeBase_>
  __host__ std::shared_ptr<KDTreeBase_> TreeCUDA_<KDTreeBase_>::clone() const {
    auto tc=new TreeCUDA_<KDTreeBase_>(*this);
    return std::shared_ptr<KDTreeBase_>(tc);
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

  template<typename KDTreeBase_>
  void Tree_CUDAfromCUDA_(TreeCUDA_<KDTreeBase_>& dest, const TreeCUDA_<KDTreeBase_>& device_src){
    using NodeType=typename KDTreeBase_::NodeType;
    dest._num_nodes=device_src._num_nodes;
    dest._num_points=0;
    dest._points_ptr=0;
    CUDA_CHECK(cudaMalloc((void**)&dest._nodes_ptr, sizeof(NodeType)*device_src._num_nodes));
    CUDA_CHECK(cudaMemcpy(dest._nodes_ptr, device_src._nodes_ptr, sizeof(NodeType)*dest._num_nodes, cudaMemcpyDeviceToDevice));

    dest._leaves_indices_ptr=0;
    dest._num_leaves=device_src._num_leaves;
    if (dest._num_leaves) {
      CUDA_CHECK(cudaMalloc((void**)&dest._leaves_indices_ptr,
                            sizeof(size_t)*dest._num_leaves));
      CUDA_CHECK(cudaMemcpy(dest._leaves_indices_ptr,
                            device_src._leaves_indices_ptr,
                            sizeof(size_t)*dest._num_leaves,
                            cudaMemcpyDeviceToDevice));
    }
  
  }

  template<typename KDTreeBase_>
  void Tree_CUDAfromCPU_(TreeCUDA_<KDTreeBase_>& dest, const TreeCPU_<KDTreeBase_>& host_src) {
    using NodeType=typename KDTreeBase_::NodeType;
    dest._num_nodes=host_src._num_nodes;
    dest._num_points=0;
    dest._points_ptr=0;
    // allocate the nodes
    CUDA_CHECK(cudaMalloc((void**)&dest._nodes_ptr,
                          sizeof(NodeType)*dest._num_nodes));
    
    CUDA_CHECK(cudaMemcpy(dest._nodes_ptr,
                          host_src._nodes_ptr,
                          sizeof(NodeType)*host_src._num_nodes,
                          cudaMemcpyHostToDevice));
    dest._num_leaves=host_src._num_leaves;
    dest._leaves_indices_ptr=0;
    if (dest._num_leaves) {
      CUDA_CHECK(cudaMalloc((void**)&dest._leaves_indices_ptr,
                            sizeof(size_t)*dest._num_leaves));
      CUDA_CHECK(cudaMemcpy(dest._leaves_indices_ptr,
                            host_src._leaves_indices_ptr,
                            sizeof(size_t)*dest._num_leaves,
                            cudaMemcpyHostToDevice));
    } 

  }


  template<typename KDTreeBase_>
  void Tree_CPUfromCUDA_(TreeCPU_<KDTreeBase_>& dest, const TreeCUDA_<KDTreeBase_>& src) {
    using NodeType=typename KDTreeBase_::NodeType;
    dest._num_nodes=src._num_nodes;
    dest._num_leaves=src._num_leaves;
    dest._num_points=0;
    dest._points_ptr=0;
    dest._leaves_indices_ptr=0;
    dest._nodes_ptr=0;
    if (dest._num_nodes) {
      dest._nodes_storage.resize(dest._num_nodes);
      dest._nodes_ptr=&dest._nodes_storage[0];
      CUDA_CHECK(cudaMemcpy(dest._nodes_ptr,
                            src._nodes_ptr,
                            sizeof(NodeType)*src._num_nodes,
                            cudaMemcpyDeviceToHost));
    }
    if (dest._num_leaves) {
      dest._leaves_indices_storage.resize(dest._num_leaves);
      dest._leaves_indices_ptr=&dest._leaves_indices_storage[0];
      CUDA_CHECK(cudaMemcpy(dest._leaves_indices_ptr,
                            src._leaves_indices_ptr,
                            sizeof(size_t)*dest._num_leaves,
                            cudaMemcpyDeviceToHost));
    }

  }

  template<typename KDTreeBase_>
  void Tree_CPUfromCPU_(TreeCPU_<KDTreeBase_>& dest, const TreeCPU_<KDTreeBase_>&src);
  
  template<typename KDTreeBase_>
  void Tree_copyTo_(KDTreeBase_& dest_, const KDTreeBase_& src_) {
    using namespace kd_slam::cuda;
    if(dest_.isGPU()&&src_.isGPU()) {
      TreeCUDA_<KDTreeBase_>& dest=reinterpret_cast<TreeCUDA_<KDTreeBase_>&>(dest_);
      const TreeCUDA_<KDTreeBase_>& src=reinterpret_cast<const TreeCUDA_<KDTreeBase_>&>(src_);
      Tree_CUDAfromCUDA_(dest, src);
      return;
    }
    if(dest_.isGPU()&& !src_.isGPU()) {
      TreeCUDA_<KDTreeBase_>& dest=reinterpret_cast<TreeCUDA_<KDTreeBase_>&>(dest_);
      const TreeCPU_<KDTreeBase_>& src=reinterpret_cast<const TreeCPU_<KDTreeBase_>&>(src_);
      Tree_CUDAfromCPU_(dest, src);
      return;
    }
    if(!dest_.isGPU()&& src_.isGPU()) {
      TreeCPU_<KDTreeBase_>& dest=reinterpret_cast<TreeCPU_<KDTreeBase_>&>(dest_);
      const TreeCUDA_<KDTreeBase_>& src=reinterpret_cast<const TreeCUDA_<KDTreeBase_>&>(src_);
      Tree_CPUfromCUDA_(dest, src);
      return;
    }
    if(!dest_.isGPU()&& !src_.isGPU()) {
      TreeCPU_<KDTreeBase_>& dest=reinterpret_cast<TreeCPU_<KDTreeBase_>&>(dest_);
      const TreeCPU_<KDTreeBase_>& src=reinterpret_cast<const TreeCPU_<KDTreeBase_>&>(src_);
      Tree_CPUfromCPU_(dest, src);
      return;
    }
    
  }

  
  template <typename KDTreeBase_>
  std::shared_ptr<KDTreeBase_> Tree_toCompute_(std::shared_ptr<KDTreeBase_>& src,
                                               bool is_gpu) {
    if (! src)
      return nullptr;
    if (src->isGPU()==is_gpu)
      return src;
    KDTreeBase_* ret;
    if (is_gpu) {
      ret=new TreeCUDA_<KDTreeBase_>(*src);
    } else {
      ret=new TreeCPU_<KDTreeBase_>(*src);
    }
    return std::shared_ptr<KDTreeBase_>(ret);
  }
  
  
} // namespace kd_slam
