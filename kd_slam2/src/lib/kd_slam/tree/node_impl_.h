#pragma once
#include <Eigen/Eigenvalues>
#include <future>
#include "utils/stable_adder_.h"
#include "label_policy_base_.h"
#include "label_policy_base_impl_.h"



namespace kd_slam {

template <typename Traits_>
size_t Node_<Traits_>::splitRange(typename Traits_::PointType* _points) const{
  size_t s=_idx_start;
  size_t e=_idx_end-1;
  while (s<=e) {
    bool left_or_right=whichSide(Traits::coordinates(_points[s]));
    if (left_or_right) {
      ++s;
    } else {
      std::swap(_points[s], _points[e]);
      --e;
    }
  }
  return s;
}

template <typename Traits_>
template <typename LabelPolicy_>
Node_<Traits_>::Node_(typename Traits_::PointType* points,
                          NodeType* nodes,
                          std::atomic<int>& idx,
                          const size_t start_range,
                          const size_t end_range,
                          const LabelPolicy_& policy,
                          const typename LabelPolicy_::LabelPolicyMetadata* parent_metadata):
  _idx_left(-1),
  _idx_right(-1)
{
  using namespace std;
  _idx_start=start_range;
  _idx_end=end_range;

  typename LabelPolicy_::LabelPolicyMetadata metadata;
  policy.fillMetadata(metadata, *this, points, parent_metadata);

  if (! metadata.is_node) {
    return;
  }
  auto middle=splitRange(points);
  //cerr << "(n: " << idx;

  // lambda to make a node and perform side effect on the item counter
  auto makeNode = [&](size_t s, size_t e, const typename LabelPolicy_::LabelPolicyMetadata* parent_metadata) -> size_t {
    int ret=++idx;
    nodes[ret]=NodeType(points, nodes, idx, s, e, policy, parent_metadata);
    return ret;
  };

  if (policy.parallel_level<0
      || metadata.level>=policy.parallel_level){
    _idx_left=makeNode(_idx_start, middle, &metadata);
    _idx_right=makeNode(middle, _idx_end, &metadata);
  } else {
    auto h_left=std::async(std::launch::async, makeNode, _idx_start, middle, &metadata);
    auto h_right=std::async(std::launch::async, makeNode, middle, _idx_end, &metadata);
    _idx_left=h_left.get();
    _idx_right=h_right.get();
  }
}

template <typename Traits_>
std::ostream& operator << (std::ostream& os, const Node_<Traits_>& n) {
  os << n._mean.transpose()
     << " " << n._direction.transpose()
     << " " << n._idx_left
     << " " << n._idx_right
     << " " << n._idx_start
     << " " << n._idx_end;
  return os;
}


  template <typename Traits_>
  __CUDA_EXPORT_INLINE__
  Node_<Traits_> Node_<Traits_>::applyVelocities(const VelocityVectorType& v) const {
    NodeType dest = *this;
    if constexpr (Traits::HasTimestamp) {
      GeometryTraits::applyVelocities(v, dest._mean, dest._direction, dest._dts_mean);
    }
    return dest;
  }

} // namespace kd_slam
