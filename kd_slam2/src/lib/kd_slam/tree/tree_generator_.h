#pragma once
#include "flat_leaf_policy_.h"
#include "tree_predicates_.h"
#include "tree_cpu_.h"
#include "srrg_config/configurable.h"
#include <memory>

namespace kd_slam {

  template <typename Node_>
  struct TreeGenerator_ : public srrg2_core::Configurable {
    using NodeType         = Node_;
    using Scalar           = typename NodeType::Scalar;
    using PointType        = typename NodeType::PointType;
    using TreeType         = TreeCPU_<Tree_<NodeType>>;
    using PointsVectorType = typename TreeType::PointsVectorType;

    PARAM(srrg2_core::PropertyBool,  propagate_normal,          "propagate normal from parent to children if flat enough", false, &_params_changed);
    PARAM(srrg2_core::PropertyInt,   tree_min_good_leaves,      "min good leaves to accept a tree",                       10,    &_params_changed);
    PARAM(srrg2_core::PropertyInt,   leaf_min_points,           "min points in leaf",                                     5,     &_params_changed);
    PARAM(srrg2_core::PropertyInt,   leaf_max_points,           "max points in leaf",                                     30,    &_params_changed);
    PARAM(srrg2_core::PropertyFloat, leaf_density,              "density of points in leaf [pts/m, m^2]",                 100.f, &_params_changed);
    PARAM(srrg2_core::PropertyFloat, leaf_min_normal_incidence, "min cos of incidence angle",                             0.1f,  &_params_changed);
    PARAM(srrg2_core::PropertyFloat, leaf_eigenvalue_threshold, "lambda_max below this: leaf discarded",                  1e-3f, &_params_changed);
    PARAM(srrg2_core::PropertyFloat, leaf_normal_eigenratio,    "below this the leaf is not flat enough",                 1e-1f, &_params_changed);

    FlatLeafPolicy_<NodeType>      policy;
    IsGoodLeafPredicate_<NodeType> good_leaf_predicate;

    void syncParams() {
      if (!_params_changed)
        return;
      policy.propagate_normal     = param_propagate_normal.value();
      policy.min_points           = param_leaf_min_points.value();
      policy.max_points           = param_leaf_max_points.value();
      policy.leaf_density         = param_leaf_density.value();
      policy.min_normal_incidence = param_leaf_min_normal_incidence.value();
      policy.eigenvalue_threshold = param_leaf_eigenvalue_threshold.value();
      policy.normal_eigenratio    = param_leaf_normal_eigenratio.value();
      _params_changed = false;
    }

    std::shared_ptr<TreeType> makeTree(PointsVectorType& src);

  protected:
    bool _params_changed = true;
  };

} // namespace kd_slam
