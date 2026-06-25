#pragma once
#include "tree_.h"


namespace kd_slam {

template <typename NodeType>
struct IsGoodLeafPredicate_ {
  using Scalar=typename NodeType::Scalar;

  size_t min_points;
  Scalar min_normal_incidence;

  __CUDA_EXPORT_INLINE__
  IsGoodLeafPredicate_(size_t mp=10, Scalar ni=0.1f): min_points(mp), min_normal_incidence(ni){}

  __CUDA_EXPORT_INLINE__
  bool operator()(const NodeType& n) const {
    using namespace std;
    if (! n.isLeaf())
      return false;
    if (! n.hasNormal())
      return false;
    Scalar normal_incidence = n._mean.dot(n._direction);
    if (fabs(normal_incidence) <= min_normal_incidence) {
      return false;
    }
    return true;
  }
};

template <typename NodeType>
struct IsLeafPredicate_ {
  __CUDA_EXPORT_INLINE__
  bool operator()(const NodeType& n) const {
    return n.isLeaf();
  }
};

template <typename NodeType>
struct IsReacheable_ {
  __CUDA_EXPORT_INLINE__
  bool operator()(const NodeType& n) const {
    return true;
  }
};

} // namespace kd_slam
