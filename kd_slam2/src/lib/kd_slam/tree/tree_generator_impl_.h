#pragma once
#include "tree_generator_.h"
#include "label_policy_base_impl_.h"
#include "tree_cpu_impl_.h"

namespace kd_slam {
  template <typename Node_>
  std::shared_ptr<typename TreeGenerator_<Node_>::TreeType >
  TreeGenerator_<Node_>::makeTree(PointsVectorType& src) {
    syncParams();
    if (src.size()<(size_t)param_leaf_min_points.value())
      return nullptr;
    auto tree=std::make_shared<TreeType>(src, policy);
    tree->prune(good_leaf_predicate);
    if ((int)tree->_num_leaves<param_tree_min_good_leaves.value())
      return nullptr;
    tree->recomputeLeaves();
    return tree;
  }
  
}
