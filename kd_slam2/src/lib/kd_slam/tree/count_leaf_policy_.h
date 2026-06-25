#pragma once
#include "node_.h"
#include "label_policy_base_.h"


namespace kd_slam {

/**
   Label policy to classify a node (NodeType, Leaf, BadLeaf)
   A leaf is a node with less than max_points;
 */


template <typename Node_>
struct CountLeafPolicy_ : public LabelPolicyBase_<Node_> {
  using NodeType             = Node_;
  using Traits             = typename NodeType::Traits;
  using PointType          = typename NodeType::PointType;
  using MatrixType         = typename NodeType::MatrixType;
  using Scalar             = typename NodeType::Scalar;
  static constexpr int Dim = NodeType::Dim;
  using BaseType            = LabelPolicyBase_<Node_>;
  using LabelPolicyMetadata = typename BaseType::LabelPolicyMetadata;

  Scalar max_points=50; // if more than these is a node

  void fillMetadata(LabelPolicyMetadata& metadata,
                    NodeType& node,
                    const PointType* points,
                    const LabelPolicyMetadata* parent_metadata=0) const {
    BaseType::fillMetadata(metadata, node, points, parent_metadata);
    if (metadata.is_bad) {
      metadata.is_leaf=true;
      return;
    }
    if (metadata.n_points>max_points) {
      metadata.is_node=true;
      node._direction=metadata.direction();
    } else {
      metadata.is_leaf=true;
      node._direction=metadata.normal();
    }
  }

};

} // namespace kd_slam
