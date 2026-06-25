#pragma once
#include "cuda/cuda_common.h"
#include "tree_cpu_.h"
#ifdef HAVE_CUDA
#include "tree_cuda_.h"
#endif
#include <queue>

namespace kd_slam {

  template<typename KDTreeBase_>
  TreeCPU_<KDTreeBase_>::TreeCPU_(const KDTreeBase_& src) :
    KDTreeBase_(src){

#ifdef HAVE_CUDA    
    if(src.isGPU()) {
      const TreeCUDA_<KDTreeBase_>& cuda_tree=reinterpret_cast<const TreeCUDA_<KDTreeBase_>&>(src);
      fromCUDA(cuda_tree);
    } else {
#endif      
      const TreeCPU_<KDTreeBase_>& cpu_tree=reinterpret_cast<const TreeCPU_<KDTreeBase_>&>(src);
      fromCPU(cpu_tree);
#ifdef HAVE_CUDA    
    }
#endif      
  }

  template<typename KDTreeBase_>
  void TreeCPU_<KDTreeBase_>::copyNodes(Tree_<NodeType>& other) const  {
    if (other.isGPU())
      throw std::runtime_error("compute incompatible");
    if (other._num_nodes!=_num_nodes)
      throw std::runtime_error("size mismatch");
    for (size_t i=0; i<_num_nodes; ++i)
      other._nodes_ptr[i]=_nodes_ptr[i];
  }


  template<typename KDTreeBase_>
  void TreeCPU_<KDTreeBase_>::applyIsometryInPlace(const IsometryType& iso) {
    for (auto &n: _nodes_storage) {
      n=applyIsometry_(iso, n);
    }
    root_mean=iso*root_mean;
    root_eigenvectors=iso.linear()*root_eigenvectors;
  }

  template<typename KDTreeBase_>
  void TreeCPU_<KDTreeBase_>::fromCPU(const TreeCPU_<KDTreeBase_>&src)
  {
    _points_storage=src._points_storage;
    _nodes_storage=src._nodes_storage;
    _points_ptr=_points_storage ? &(*_points_storage)[0] : nullptr;
    _num_points=_points_storage ? _points_storage->size() : 0;
    _nodes_ptr=&_nodes_storage[0];
    recomputeLeaves();
  }

#ifdef HAVE_CUDA
  template<typename KDTreeBase_>
  void TreeCPU_<KDTreeBase_>::fromCUDA(const TreeCUDA_<KDTreeBase_>& src) {
    using namespace kd_slam::cuda;
    _num_nodes=src._num_nodes;
    _num_leaves=src._num_leaves;
    _num_points=0;
    _points_ptr=0;
    _leaves_indices_ptr=0;
    _nodes_ptr=0;
    if (_num_nodes) {
      _nodes_storage.resize(_num_nodes);
      _nodes_ptr=&_nodes_storage[0];
      CUDA_CHECK(cudaMemcpy(_nodes_ptr,
                            src._nodes_ptr,
                            sizeof(NodeType)*src._num_nodes,
                            cudaMemcpyDeviceToHost));
    }
    if (_num_leaves) {
      _leaves_indices_storage.resize(_num_leaves);
      _leaves_indices_ptr=&_leaves_indices_storage[0];
      CUDA_CHECK(cudaMemcpy(_leaves_indices_ptr,
                            src._leaves_indices_ptr,
                            sizeof(size_t)*_num_leaves,
                            cudaMemcpyDeviceToHost));
    }
  }
#endif
  
  template <typename KDTreeBase_>
  void TreeCPU_<KDTreeBase_>::applyPermutation(const std::vector<int>& permutation) {
    if (permutation.size()> numNodes()) {
      std::cerr <<  "perm_size: " << permutation.size() << " nodes: " << numNodes() << std::endl;
      throw std::runtime_error("Permutation size mismatch");
    }
  
    NodesVectorType dest_nodes(permutation.size());
    std::vector<int> inv_permutation(numNodes());
    std::fill(inv_permutation.begin(), inv_permutation.end(), -1);
    for (size_t i=0; i<permutation.size(); ++i) {
      inv_permutation[permutation[i]]=i;
    }
  
    for (size_t i=0; i<dest_nodes.size(); ++i){
      auto& dest=dest_nodes[i];
      const auto& src_idx=permutation[i];
      dest=_nodes_storage[src_idx];
      if (dest._idx_left>=0)
        dest._idx_left=inv_permutation[dest._idx_left];
      if (dest._idx_right>=0)
        dest._idx_right=inv_permutation[dest._idx_right];
    }
    _nodes_storage=dest_nodes;
    _nodes_ptr=&_nodes_storage[0];
    _num_nodes=permutation.size();
  }

  template <typename KDTreeBase_>
  size_t TreeCPU_<KDTreeBase_>::searchFast(const typename TreeCPU_<KDTreeBase_>::VectorType& q) const {
    size_t aux_idx=0;
    while (! _nodes_ptr[aux_idx].isLeaf()) {
      const auto& aux = _nodes_ptr[aux_idx];
      aux_idx = aux.whichSide(q) ? aux._idx_left: aux._idx_right;
    }
    const auto& aux = _nodes_ptr[aux_idx];
    Scalar min_distance=std::numeric_limits<Scalar>::max();
    size_t min_idx=0;
    for (auto i=aux._idx_start; i<aux._idx_end; ++i){
      Scalar d2=(q-Traits::coordinates(_points_ptr[i])).squaredNorm();
      if (d2<min_distance) {
        min_distance=d2;
        min_idx=i;
      }
    }
    return min_idx;
  }

  template <typename KDTreeBase_>
  void TreeCPU_<KDTreeBase_>::searchFull(std::vector<size_t>& result,
                                    const typename TreeCPU_<KDTreeBase_>::VectorType& q,
                                    const typename TreeCPU_<KDTreeBase_>::Scalar r,
                                    int idx) const {
    if (idx<0 || idx>=(int) numNodes())
      return;
    const auto& n=_nodes_ptr[idx];

    if (n.isLeaf()) {
      const auto r2=r*r;
      for (auto i=n._idx_start; i<n._idx_end; ++i){
        const auto& p=_points_ptr[i];
        Scalar d2=(q-Traits::coordinates(p)).squaredNorm();
        if (d2<r2) {
          result.push_back(i);
        }
      }
      return;
    }
    
    // compute distance from split plane
    Scalar d=n._direction.dot(q-n._mean);

    // if small we trigger the search on both branches
    if (fabs(d)<r) {
        searchFull(result, q, r, n._idx_left);
        searchFull(result, q, r, n._idx_right);
        return;
    }

    // if large we proceed greedily on one branch
    if (d>0) {
      searchFull(result, q, r, n._idx_left);
    } else {
      searchFull(result, q, r, n._idx_right);
    }
  }


  template <typename KDTreeBase_>
  std::vector<int> TreeCPU_<KDTreeBase_>::breadthFirstPermutation() const  {
    std::vector<int> permutation;
    if (! numNodes())
      return permutation;
    permutation.resize(numNodes());
    size_t pos=0;
    std::deque<int> frontier;
    frontier.push_back(0);
    while (! frontier.empty()) {
      int current_idx=frontier.front();
      frontier.pop_front();
      permutation[pos]=current_idx;
      ++pos;
      const auto& n=_nodes_ptr[current_idx];
      if (n._idx_left>=0)
        frontier.push_back(n._idx_left);
      if (n._idx_right>=0)
        frontier.push_back(n._idx_right);
    }
    permutation.resize(pos);
    return permutation;
  }

  template <typename KDTreeBase_>
  std::vector<typename KDTreeBase_::VectorType>
  TreeCPU_<KDTreeBase_>::extractCompressedLeaves() const {
    std::vector<VectorType> ret;
    ret.reserve(_num_leaves*2);
    for (size_t i=0; i<_num_leaves; ++i) {
      const auto idx=_leaves_indices_ptr[i];
      if (idx>=_num_nodes)
        throw std::runtime_error("leaves indices out of bounds");
      const auto& n=_nodes_ptr[idx];
      ret.push_back(n._mean);
      ret.push_back(n._direction);
    }
    return ret;
  }


  template <typename KDTreeBase_>
  std::vector<typename KDTreeBase_::VectorType>
  TreeCPU_<KDTreeBase_>::extractCompressedLeaves(const VelocityVectorType& v) const {
    std::vector<VectorType> ret;
    ret.reserve(_num_leaves*2);
    for (size_t i=0; i<_num_leaves; ++i) {
      const auto idx=_leaves_indices_ptr[i];
      if (idx>=_num_nodes)
        throw std::runtime_error("leaves indices out of bounds");
      const auto n=_nodes_ptr[idx].applyVelocities(v);
      ret.push_back(n._mean);
      ret.push_back(n._direction);
    }
    return ret;
  }

  template <typename KDTreeBase_>
  void TreeCPU_<KDTreeBase_>::applyVelocitiesInPlace(const VelocityVectorType& v, bool skip_leaves) {
    for (size_t i = 0; i < _num_nodes; ++i) {
      auto& n = _nodes_ptr[i];
      if (n.isLeaf() && skip_leaves) continue;
      n = n.applyVelocities(v);
    }
  }

} // namespace kd_slam
