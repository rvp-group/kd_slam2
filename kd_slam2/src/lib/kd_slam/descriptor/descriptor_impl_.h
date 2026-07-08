#pragma once
#include "descriptor_.h"


namespace kd_slam { namespace descriptor {

    template <typename Node_, int Size_>
    __CUDA_EXPORT_INLINE__
    typename Node_::Scalar
    Descriptor_<Node_, Size_>::eucCompare(const typename Descriptor_<Node_, Size_>::ThisType& other,
                                          const typename Node_::Scalar w, int n) const {
      const int end = (n < 0 || n > Size) ? Size : n;
      Scalar res=Scalar(0);
      for (int i=0; i<end; ++i) {
        res+=values[i].distance(other.values[i], w);
      }
      return res;
    }

    template <typename Node_, int Size_>
    __CUDA_EXPORT_INLINE__
    typename Node_::Scalar
    Descriptor_<Node_, Size_>::medCompare(const typename Descriptor_<Node_, Size_>::ThisType& other,
                                          const typename Node_::Scalar dir_weight, int n) const{
      using namespace std;
      const int end = (n < 0 || n > Size) ? Size : n;
      int parent_indices[Size];
      Scalar result=Scalar(0);
      for (int al_idx=0; al_idx<end;  al_idx+=2) {
        int bl_idx=0;
        int parent_idx_pos=-1;
        if (al_idx>1) {
          parent_idx_pos=al_idx/2-1;
          bl_idx =  (parent_indices[parent_idx_pos]+1)<<1;
        }
        const int ar_idx=al_idx+1;
        const int br_idx=bl_idx+1;
        const auto& al = values[al_idx];
        const auto& ar = values[ar_idx];
        const auto& bl = other.values[bl_idx];
        const auto& br = other.values[br_idx];
        const Scalar ab = al.distance(bl, dir_weight)+ar.distance(br, dir_weight);
        const Scalar ba = ar.distance(bl, dir_weight)+al.distance(br, dir_weight);
        if (ab<ba) {
          result+=ab;
          parent_indices[al_idx]=bl_idx;
          parent_indices[ar_idx]=br_idx;
        } else {
          result+=ba;
          parent_indices[al_idx]=br_idx;
          parent_indices[ar_idx]=bl_idx;
        }
      }
      return result;
    }

    template <typename Node_, int Size_>
    __CUDA_EXPORT_INLINE__
    typename  Node_::MatrixType Descriptor_<Node_, Size_>::applyCanonization(const typename Node_::MatrixType& src_R, int canonization) {
      
      MatrixType R=src_R;
      bool flip_last=false;
      // flip the axes of Rotation based on the canonization;
      for (int i=0; i<Dim-1; ++i) {
        if(getBit(canonization,i)) {
          R.col(i)=-R.col(i);
          flip_last=!flip_last;
        }
      }
      // ensure positive determinant on last axis
      if (flip_last) {
        R.col(Dim-1)=-R.col(Dim-1);
      }
      return R;
    }


    template <typename KDTreeType_, int Size_>
    std:: ostream& operator << (std::ostream& os, const Descriptor_<KDTreeType_, Size_>& desc) {
      int previous_level=0;
      for (int i=0; i<Size_; ++i) {
        int level=log2(i+2);
        if (level>previous_level) {
          os << "LEVEL " << level << std::endl;
        }
        os << "(" <<  desc.values[i].m.transpose() << " " << desc.values[i].d.transpose() << ")" << std::endl;
        previous_level=level;
      }
      return os;
    };
                            


    template <typename Node_, int Size_>
    __CUDA_EXPORT_INLINE__
    typename Descriptor_<Node_, Size_>::ThisType
    Descriptor_<Node_, Size_>::remap(int target_canonization) const {
      // retrieve the can0 eigenvectors
      MatrixType R_0=applyCanonization(root_transform.linear(), axes_canonization);
      MatrixType R_target=applyCanonization(R_0, target_canonization);
      MatrixType R_remap=R_target.transpose()*root_transform.linear();
      ThisType remapped;
      remapped.axes_canonization = target_canonization;
      remapped.ref = ref;
      for (int i = 0; i < Size; ++i) {
        remapped.values[i].m = R_remap * values[i].m;
        remapped.values[i].d = R_remap * values[i].d;
      }
      remapped.root_transform.linear()=R_target;
      remapped.root_transform.translation()=root_transform.translation();
      return remapped;
    }


    template <typename Node_, int Size_>
    typename Descriptor_<Node_, Size_>::QDescriptors
    Descriptor_<Node_, Size_>::buildQDescriptors() {
      QDescriptors q;
      if (axes_canonization!=0)
        throw std::runtime_error("QDescriptors only from can0");
      q.des[0] = *this;
      for (int c = 1; c < NumAxesCanonizations; ++c)
        q.des[c] = remap(c);
      return q;
    }

  }
} // namespace kd_slam::descriptor
