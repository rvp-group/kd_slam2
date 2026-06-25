#pragma once
#include <Eigen/Core>
#include "kd_slam/utils/geometry_.h"

namespace kd_slam {

// generic traits for an Eigen vector
template <typename EigenVectorNX_>
struct PointTraitsEigen_ {
  using PointType = EigenVectorNX_;
  using VectorType = EigenVectorNX_;
  using Scalar     = typename VectorType::Scalar;
  static constexpr int Dim = VectorType::RowsAtCompileTime;
  using GeometryTraits=geometry::GeometryTraits_<Dim, Scalar>;
  static constexpr bool HasTimestamp = false;
  static inline const VectorType& coordinates(const PointType& p) { return p; }
  static inline VectorType& coordinates(PointType& p) { return p; }
  
};

template <typename VectorType_>
struct PointWithEigenMember_ {
  using VectorType=VectorType_;
  VectorType coordinates;
  using Scalar=typename VectorType::Scalar;
};

template <typename PointBase_>
struct PointStamped_: public PointBase_ {
  double stamp=0;
};

template <typename PointType_>
struct PointTraits_ {
  using PointType=PointType_;
  using VectorType = typename PointType::VectorType;
  using Scalar     = typename PointType::Scalar;
  static constexpr int Dim = decltype(PointType::coordinates)::RowsAtCompileTime;
  using GeometryTraits=geometry::GeometryTraits_<Dim, Scalar>;
  static constexpr bool HasTimestamp = false;
  static inline const VectorType& coordinates(const PointType& p) { return p.coordinates; }
  static inline VectorType& coordinates(PointType& p) { return p.coordinates; }
};

template <typename PointType_>
struct PointTraitsStamped_ : PointTraits_<PointType_> {
  static constexpr bool HasTimestamp = true;
  static inline double stamp(const PointType_& p) { return p.stamp; }
  static inline double& stamp(PointType_& p) { return p.stamp; }
};


} // namespace kd_slam
