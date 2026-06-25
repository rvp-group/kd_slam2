#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include "node_.h"


namespace kd_slam {

/*
  KDTree that uses an array of nodes to store the items
 */
template <typename Node_>
struct Tree_{
  using NodeType     = Node_;
  using Traits     = typename NodeType::Traits;
  using PointType  = typename NodeType::PointType;
  using VectorType = typename NodeType::VectorType;
  using Scalar     = typename NodeType::Scalar;
  static constexpr int Dim = NodeType::Dim;
  using MatrixType         = typename NodeType::MatrixType;
  using IsometryType       = Eigen::Transform<Scalar, Dim, Eigen::Isometry>;
  using VelocityVectorType = typename NodeType::VelocityVectorType;
    
  PointType* _points_ptr=0; // array of points in the tree
  size_t _num_points=0;     // number of points
  NodeType* _nodes_ptr=0;     // array of nodes
  size_t _num_nodes=0;      // number of nodes
  double _ts_start=0;       // global timestamp of the first point
  size_t* _leaves_indices_ptr=0;
  size_t  _num_leaves=0;
 
  MatrixType root_eigenvectors=MatrixType::Zero();
  VectorType  root_eigenvalues=VectorType::Zero();
  VectorType  root_mean=VectorType::Zero();
  
  // number of nodes
  inline size_t numNodes() const {return _num_nodes;}

  virtual void copyNodes(Tree_<NodeType>& other) const = 0;
  
  virtual std::vector<VectorType> extractCompressedLeaves() const = 0;
  virtual std::vector<VectorType> extractCompressedLeaves(const VelocityVectorType& v) const = 0;
  
  // Manipulates the tree and suppresses all nodes
  // for which the predicate is NOT true
  // the structure is changed.
  virtual bool isGPU() const =0;
  virtual void applyVelocitiesInPlace(const VelocityVectorType& v, bool skip_leaves=false) = 0;
  virtual void applyIsometryInPlace(const IsometryType& iso)=0;

  virtual ~Tree_(){}
};


} // namespace kd_slam
