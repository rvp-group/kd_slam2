#pragma once
#include "cuda/cuda_common.h"
#include <Eigen/Core>
#include <atomic>
#include <type_traits>
#include "utils/geometry_.h"

namespace kd_slam {

struct EmptyBase_ {};

struct TimestampInfo{
  double _dts_mean;
  double _dts_covariance;
};
  
// this one here is flattened and ready for CUDA
template <typename Traits_>
struct Node_: std::conditional_t<Traits_::HasTimestamp, TimestampInfo, EmptyBase_>
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Traits     = Traits_;
  using PointType  = typename Traits::PointType;
  using VectorType = typename Traits::VectorType;
  using Scalar     = typename Traits::Scalar;
  static constexpr int Dim = Traits::Dim;
  using NodeType    = Node_<Traits>;
  using MatrixType=Eigen::Matrix<Scalar, Dim, Dim>;
  using GeometryTraits   = typename Traits::GeometryTraits;
  using PoseHessianType  = typename GeometryTraits::HessianType;
  using IsometryType     = typename GeometryTraits::IsometryType;
  using VelocityVectorType = typename GeometryTraits::VelocityVectorType;
  //! centroid
  VectorType _mean=VectorType::Zero();
  
  
  //! dominant direction (if intermediate node) or normal (if leaf)
  VectorType _direction=VectorType::Zero();
  
  /** range in the point array*/
  size_t _idx_start=0, _idx_end=0;
  /** indices for the left and the right child in the nodes array*/
  int _idx_left=-1, _idx_right=-1;

 
  /**size of the points in the node*/
  __CUDA_EXPORT_INLINE__
  size_t size() const {return _idx_end-_idx_start;}

  /**returns a boolean telling if the search has to continue
   to the left (true) or to the right (false), by checking on
  which side of the plane the query falls*/
  __CUDA_EXPORT_INLINE__
  bool whichSide(const VectorType& query) const {
    return _direction.dot(query-_mean)>0;
  }

  /** true if the node is a leaf
   */
  __CUDA_EXPORT_INLINE__
  bool isLeaf() const {
    return _idx_left<0 && _idx_right<0;
  }

  /**true if the node has a normal well defined*/
  __CUDA_EXPORT_INLINE__
  bool hasNormal() const {
    return isLeaf() && _direction.squaredNorm()>0;
  }
  
  /**returns the normal, if valid*/
  __CUDA_EXPORT_INLINE__
  VectorType normal() const {
    if (! isLeaf())
      return VectorType::Zero();
    return _direction;
  }

  /**returns the direction, if valid*/
  __CUDA_EXPORT_INLINE__
  VectorType direction() const {
    if (isLeaf()) {
      return VectorType::Zero();
    }
    return _direction;
  }

  /**partitions the points in the range idx_start, idx_end
     in two contiguous sets.
     It returns the index of the separator of the partitions
     between start and end idx
   */
  size_t splitRange(PointType* _points) const;
  
  /**
     Internal constructor used to build the tree
     it recursively manipulates nodes
     points: the points
     nodes: the array of nodes where the recursively created children will be allocated
     idx:  the index of the next available node
     start and end range: the range in the points for this node
     policy: the policy used to control th construction
   */
  template <typename LabelPolicy_>
  Node_(PointType* points,
          NodeType* nodes,
          std::atomic<int>& idx,
          const size_t start_range,
          const size_t end_range,
          const LabelPolicy_& policy,
          const typename LabelPolicy_::LabelPolicyMetadata* parent_metadata=0);
  
  __CUDA_EXPORT_INLINE__
  NodeType applyVelocities(const VelocityVectorType& v) const;

  __CUDA_EXPORT_INLINE__ Node_(){}
};

// Bring in global (VectorType) overloads so they coexist with the Node_ overload below
using geometry::applyIsometry_;
using geometry::applyRotation_;

/**
   Multiplication by an isometry. Comes in handy
 */
template <typename Traits_>
__CUDA_EXPORT_INLINE__  Node_<Traits_> applyIsometry_(const Eigen::Transform<typename Traits_::Scalar, Traits_::Dim, Eigen::Isometry>& iso,
                                      const Node_<Traits_>& n) {
  using Scalar = typename Traits_::Scalar;
  using TransformType=Eigen::Transform<typename Traits_::Scalar, Traits_::Dim, Eigen::Isometry>;
  static constexpr int Dim=Traits_::Dim;
  using VectorType=Eigen::Matrix<Scalar, Dim, 1>;
  Node_<Traits_> returned=n;
  returned._mean=applyIsometry_<TransformType, VectorType>(iso,n._mean);
  returned._direction=applyRotation_<TransformType, VectorType>(iso,n._direction);
  return returned;
}
                              
/**
   stream
 */
template <typename Traits_>
std::ostream& operator << (std::ostream&, const Node_<Traits_>&);

} // namespace kd_slam
