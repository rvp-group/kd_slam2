#pragma once
#include "kd_slam/cuda/cuda_common.h"
#include <Eigen/Geometry>
namespace kd_slam{
  namespace geometry {
  
    /**
       This version of eigen does not deal well with transformations
       had to to my own ISO*point
    */
    template <bool RotateOnly, typename TransformType, typename VectorType>
    __CUDA_EXPORT_INLINE__
    VectorType applyIsometry_(const TransformType& X, const VectorType& p) {
      VectorType returned;
      if constexpr (RotateOnly){
        returned.setZero();
      } else {
        returned=X.translation();
      }
      static constexpr int Dim=VectorType::RowsAtCompileTime;
      for (int r=0; r<Dim; ++r)
        for (int c=0; c<Dim; ++c)
          returned(r)+=X.matrix()(r,c)*p(c);
      return returned;
    }

    template <typename TransformType, typename VectorType>
    __CUDA_EXPORT_INLINE__
    VectorType applyIsometry_(const TransformType& X, const VectorType& p) {
      return applyIsometry_<false>(X, p);
    }

    template <typename TransformType, typename VectorType>
    __CUDA_EXPORT_INLINE__
    VectorType applyRotation_(const TransformType& X, const VectorType& p) {
      return applyIsometry_<true>(X, p);
    }

    template <typename Scalar_, int Dim_, int Mode_>                                                                     
    Eigen::Isometry3f toIsometry3f(const Eigen::Transform<Scalar_, Dim_, Mode_>& iso) {
      if constexpr (Dim_ == 3) {
        return iso.template cast<float>();
      } else {
        Eigen::Isometry3f result = Eigen::Isometry3f::Identity();
        result.linear().template topLeftCorner<2,2>() = iso.linear().template cast<float>();
        result.translation().template head<2>() = iso.translation().template cast<float>();
        return result;
      }
    }
        
    template <typename Scalar_>
    using Vector3_=Eigen::Matrix<Scalar_,3,1>;

    template <typename Scalar_>
    using Matrix3_=Eigen::Matrix<Scalar_,3,3>;

    template <int Dim_, typename Scalar_>
    struct GeometryTraits_;

  }
}
