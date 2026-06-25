#pragma once
#include "node_.h"
#include "count_leaf_policy_.h"


namespace kd_slam {

/**
   Label policy to construct  a tree.
   The leaves are at least min_points and at most max_points.
   The leaves normal can be propagated from the parent if flat enough (flag propagate normal)
   A leave can have a variable number of points, and the recursive split is decided based on its density. If a lot dense, the points are "merged" in a single leaf.
 */
template <typename Node_>
struct FlatLeafPolicy_: public CountLeafPolicy_<Node_> {
  using NodeType=Node_;
  using Traits= typename NodeType::Traits;
  using PointType=typename NodeType::PointType;
  using MatrixType=typename NodeType::MatrixType;
  using VectorType = typename NodeType::VectorType;
  using Scalar     = typename NodeType::Scalar;
  static constexpr int Dim=Traits::Dim;
  using BaseType=CountLeafPolicy_<Node_>;
  
  struct LabelPolicyMetadata: public BaseType::LabelPolicyMetadata {
    bool is_flat=false;
    bool has_propagated_normal=false;
    VectorType propagated_normal=VectorType::Zero();
  };

  bool propagate_normal=false;
  Scalar leaf_density=100;
  Scalar min_points=5; // if less than these bad leaf
  Scalar min_normal_incidence=0.1; //if incidence below this treshold the normal is not well defined (bad leaf)
  Scalar eigenvalue_threshold=1e-3; //if the max eigenval of covariance below this, bad leaf
  Scalar normal_eigenratio=1e-1; // if lambda_min/lambda_max above this the covariance is not flat enough. If size()>min_points it becomes a node
  
  inline bool propagateNormal(NodeType& node,
                              LabelPolicyMetadata& metadata) const {
    
    metadata.is_leaf=true;
    if (metadata.has_propagated_normal) {
      node._direction=metadata.propagated_normal;
      if (node._mean.dot(node._direction)>0) {
        node._direction=-node._direction;
      }
      return true;
    }
    return false;
  }

  void fillMetadata(LabelPolicyMetadata& metadata,
                      NodeType& node,
                      const PointType* points,
                      const LabelPolicyMetadata* parent_metadata=0) const {
    BaseType::fillMetadata(metadata, node, points, parent_metadata);
    if (parent_metadata && propagate_normal) {
      metadata.propagated_normal=parent_metadata->propagated_normal;
      metadata.has_propagated_normal=parent_metadata->has_propagated_normal;
    } else {
      metadata.propagated_normal.setZero();
      metadata.has_propagated_normal=false;
    }

    // if too few points, ignore or propagate the normal from parent if it has one
    if (metadata.is_bad || metadata.n_points<min_points) {
      metadata.is_bad=!propagateNormal(node, metadata);
      return;
    }
    
    const Scalar lambda_max=metadata.eigenvalues(Dim-1);
    const Scalar lambda_min=metadata.eigenvalues(0);
    const Scalar eigenratio=lambda_min/lambda_max;

    metadata.is_flat = eigenratio<normal_eigenratio
      && fabs(node._mean.dot(metadata.normal()))>min_normal_incidence;

    // if flat set the (local) normal for propagagtion
    if (metadata.is_flat) {
      metadata.has_propagated_normal=true;
      metadata.propagated_normal=metadata.normal();
    }
    // if at this point the thing is a node, stop
    if (metadata.is_node)
      return;

    // if it is a leaf, check for further split based on the density
    // if sparse, we turn the leaf in a node
    Scalar volume=Scalar(1.);
    for (int i=1; i<Dim; ++i)
      volume*=metadata.eigenvalues(i);
    Scalar density=metadata.n_points/volume;

    if (density<leaf_density) {
      metadata.is_leaf=false;
      metadata.is_node=true;
      metadata.is_bad=false;
      node._direction=metadata.direction();
      return;
    }

    if (metadata.is_leaf && metadata.is_flat) {
      node._direction=metadata.normal();
      return;
    }

    metadata.is_bad=!propagateNormal(node, metadata);
  }

};

} // namespace kd_slam
