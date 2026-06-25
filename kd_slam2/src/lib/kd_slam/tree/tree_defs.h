#pragma once
#include "point_traits_.h"
#include "tree_predicates_.h"
#include "tree_cpu_.h"
#include "count_leaf_policy_.h"
#include "flat_leaf_policy_.h"
#include "tree_generator_.h"

namespace kd_slam {

  // using Point2f=Eigen::Vector2f;
  // using Point2fTraits=PointTraitsEigen_<Point2f>;

  using Point2f=PointStamped_<PointWithEigenMember_<Eigen::Vector2f>>;
  using Point2fTraits=PointTraitsStamped_<Point2f>;
  using NodePoint2f = Node_<Point2fTraits>;
  using TreePoint2f = Tree_<NodePoint2f>;
  using TreeCPUPoint2f  = TreeCPU_< TreePoint2f >;
  using LabelPolicyBasePoint2f = LabelPolicyBase_<NodePoint2f>;
  using CountLeafPolicyPoint2f = CountLeafPolicy_<NodePoint2f>;
  using FlatLeafPolicyPoint2f =  FlatLeafPolicy_<NodePoint2f>;
  using TreeGeneratorPoint2f  =  TreeGenerator_<NodePoint2f>;

  // using Point3f=Eigen::Vector3f;
  // using Point3fTraits=PointTraitsEigen_<Point3f>;

  using Point3f=PointStamped_<PointWithEigenMember_<Eigen::Vector3f>>;
  using Point3fTraits=PointTraitsStamped_<Point3f>;
  using NodePoint3f = Node_<Point3fTraits>;
  using TreePoint3f = Tree_<NodePoint3f>;
  using TreeCPUPoint3f  = TreeCPU_< TreePoint3f >; 
  using LabelPolicyBasePoint3f = LabelPolicyBase_<NodePoint3f>;
  using CountLeafPolicyPoint3f = CountLeafPolicy_<NodePoint3f>;
  using FlatLeafPolicyPoint3f =  FlatLeafPolicy_<NodePoint3f>;
  using TreeGeneratorPoint3f  =  TreeGenerator_<NodePoint3f>;

} // namespace kd_slam
