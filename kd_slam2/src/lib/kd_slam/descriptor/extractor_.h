#pragma once
#include "descriptor_.h"


namespace kd_slam {
  namespace descriptor {

    template <typename Tree_, int Level_>
    struct Extractor_{
      using KDTree=Tree_;
      using NodeType= typename KDTree::NodeType;
      using PointType=typename KDTree::PointType;
      using MatrixType=typename KDTree::MatrixType;
      static constexpr int Dim=KDTree::Dim;
      static constexpr int Level=Level_;
      static constexpr int Size=(1<<Level)-2;
      using DescriptorType=Descriptor_<NodeType, Size>;
      static constexpr int NumAxesCanonizations=DescriptorType::NumAxesCanonizations;

  
      static DescriptorType extract(const KDTree& tree, int canonization);  
  
    };

  } } // namespace kd_slam::descriptor
