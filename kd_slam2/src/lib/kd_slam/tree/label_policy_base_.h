#pragma once


namespace kd_slam {

template <typename Node_>
struct LabelPolicyBaseMetadata_{
  using NodeType=Node_;
  using PointType=typename NodeType::PointType;
  using MatrixType=typename NodeType::MatrixType;
  using VectorType = typename NodeType::VectorType;
  using Scalar     = typename NodeType::Scalar;
  static constexpr int Dim=NodeType::Dim;
  
  VectorType mean=VectorType::Zero();
  MatrixType cov=MatrixType::Zero();
  VectorType eigenvalues=VectorType::Zero();
  MatrixType eigenvectors=MatrixType::Zero();
  size_t n_points=0;
  bool is_node=false;
  bool is_leaf=false;
  bool is_bad=true;
  int level=0;
  double ts_start=0;
  inline const VectorType normal() const {return eigenvectors.col(0);}
  inline const VectorType direction() const {return eigenvectors.col(Dim-1);}
};

template <typename Node_>
struct LabelPolicyBase_{
  using NodeType=Node_;
  using Traits= typename NodeType::Traits;
  using PointType=typename NodeType::PointType;
  using MatrixType=typename NodeType::MatrixType;
  using VectorType = typename NodeType::VectorType;
  using Scalar     = typename NodeType::Scalar;
  static constexpr int Dim=Traits::Dim;
  using LabelPolicyBase=LabelPolicyBase_<NodeType>;
  using LabelPolicyMetadata=LabelPolicyBaseMetadata_<NodeType>;
  int parallel_level=-1;

  /* fills in metadata
     - mean
     - covariance
     - flag "is_bad" if n points<Dim
     - n_points
     - eigenvalues and eigenvectors
     reorients smaller and largest evectors based on the mean

     fills in node
     - mean;
*/
  
  void fillMetadata(LabelPolicyMetadata& metadata,
                    NodeType& node,
                    const PointType* points,
                    const LabelPolicyMetadata* parent_metadata=0) const;
};

} // namespace kd_slam
