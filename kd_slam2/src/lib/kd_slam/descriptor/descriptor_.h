#pragma once
#include "cuda/cuda_common.h"
#include "tree/tree_.h"

namespace kd_slam {
  namespace descriptor {

    template <typename T>
    __CUDA_EXPORT_INLINE__ bool getBit(const T& v, int bit) {
      return v&(1<<bit);
    }

    template <typename T>
    __CUDA_EXPORT_INLINE__ void setBit(T& v, int bit, bool value) {
      if (value) {
        v|=(1<<bit);
      } else {
        v&=~(1<<bit);
      }
    }

                   
    template <typename Node_, int Size_>
    struct Descriptor_ {
      using NodeType= Node_;
      using Traits = typename NodeType::Traits;
      using PointType=typename NodeType::PointType;
      using VectorType=typename NodeType::VectorType;
      using MatrixType=typename NodeType::MatrixType;
      using IsometryType = typename Traits::GeometryTraits::IsometryType;
      using Scalar=typename NodeType::Scalar;
      static constexpr int Dim=NodeType::Dim;
      static constexpr int Size=Size_;
      static constexpr int NumAxesCanonizations=1<<(Dim-1);
      IsometryType root_transform=IsometryType::Identity();
      using ThisType=Descriptor_<NodeType, Size>;
      struct DescriptorEntry {
        VectorType m;
        VectorType d;
        __CUDA_EXPORT_INLINE__ Scalar distance(const DescriptorEntry& other,
                                               const Scalar dir_weight=1.) const {
          return (m-other.m).squaredNorm()+dir_weight*(d-other.d).squaredNorm();
        }
      };
      DescriptorEntry values[Size];
      int axes_canonization=-1; // -1: invalid descriptor
      size_t ref=0;

      __CUDA_EXPORT_INLINE__ Scalar eucCompare(const ThisType& other, const Scalar dir_weight=1., int n=-1) const;

      __CUDA_EXPORT_INLINE__ Scalar medCompare(const ThisType& other, const Scalar dir_weight=1., int n=-1) const;

      static __CUDA_EXPORT_INLINE__ MatrixType applyCanonization(const MatrixType& src_R, int canonization);

      __CUDA_EXPORT_INLINE__ ThisType remap(int target_canonization) const;

      IsometryType toPose() const {
        return root_transform;
      }
      
      struct QDescriptors {
        ThisType des[NumAxesCanonizations];
      };

      QDescriptors buildQDescriptors();
  
    };

    template <typename KDTreeType_, int Size_>
    std:: ostream& operator << (std::ostream& os,
                                const Descriptor_<KDTreeType_, Size_>& desc);                            


  }
} // namespace kd_slam::descriptor
