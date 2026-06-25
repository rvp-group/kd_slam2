#pragma once
#include "tree_.h"

namespace kd_slam {

  template <typename KDTreeBase_>
  struct TreeCPU_;

/**
   GPU implementation of a tree.
   A GPU tree stores also the indices of the good leaves (according to a predicate)
 */
template <typename KDTreeBase_>
struct TreeCUDA_: public KDTreeBase_ {
  using KDTreeBase = KDTreeBase_;
  using Traits     = typename KDTreeBase::Traits;
  using NodeType     = typename KDTreeBase::NodeType;
  using PointType  = typename KDTreeBase::PointType;
  using VectorType = typename KDTreeBase::VectorType;
  using Scalar     = typename KDTreeBase::Scalar;
  using MatrixType = typename KDTreeBase::MatrixType;
  using TreeCPUType  = TreeCPU_<KDTreeBase>;
  using TreeCUDAType = TreeCUDA_<KDTreeBase>;
  static constexpr int Dim = KDTreeBase::Dim;

  static constexpr bool IsGPU=true;

  using VelocityVectorType = typename KDTreeBase::VelocityVectorType;
  using IsometryType = typename KDTreeBase::IsometryType;
  using KDTreeBase::_points_ptr;
  using KDTreeBase::_num_points;
  using KDTreeBase::_nodes_ptr;
  using KDTreeBase::_num_nodes;
  using KDTreeBase::_ts_start;
  using KDTreeBase::_leaves_indices_ptr;
  using KDTreeBase::_num_leaves;
  using KDTreeBase:: numNodes;
  using KDTreeBase::root_mean;
  using KDTreeBase::root_eigenvectors;
  
  bool isGPU() const override {return IsGPU;}
  
  void copyNodes(Tree_<NodeType>& other) const override;

  std::vector<VectorType> extractCompressedLeaves() const override;
  std::vector<VectorType> extractCompressedLeaves(const VelocityVectorType& v) const override;
  
  // dtor
  ~TreeCUDA_();

  TreeCUDA_(const KDTreeBase& src);

  void applyVelocitiesInPlace(const VelocityVectorType& v, bool skip_leaves = false) override;

  void applyIsometryInPlace(const IsometryType& iso) override;

protected:

  void fromCPU(const TreeCPU_<KDTreeBase_>&src);
  void fromCUDA(const TreeCUDA_<KDTreeBase_>& src);


};

} // namespace kd_slam
