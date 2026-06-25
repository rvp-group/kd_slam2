#pragma once
#include <Eigen/Eigenvalues>
#include "label_policy_base_.h"
#include "utils/stable_adder_.h"
#include "node_.h"
#include <Eigen/Eigenvalues>
#include "label_policy_base_.h"


namespace kd_slam {

template <typename NodeType_>
void LabelPolicyBase_<NodeType_>::fillMetadata(LabelPolicyMetadata& metadata,
                                                 NodeType& node,
                                                 const PointType* points,
                                                 const LabelPolicyMetadata* parent_metadata) const {

  using namespace std;
  using namespace kd_slam::utils;
  static constexpr int  AccumulatorDim=((Dim+1)*Dim)/2+Dim;
  using AccumulatorVector=Eigen::Matrix<Scalar,AccumulatorDim,1>;
  metadata.n_points=node._idx_end-node._idx_start;
  if (!metadata.n_points) {
    metadata.is_bad=true;
    return;
  }
  if constexpr(NodeType_::Traits::HasTimestamp) {
    double ts_start=std::numeric_limits<double>::max();
    if (! parent_metadata) {
      for (size_t i=node._idx_start; i<node._idx_end; ++i) {
        const auto& ts=Traits::stamp(points[i]);
        ts_start=std::min(ts_start, ts);
      }
      metadata.ts_start=ts_start;
    } else {
      metadata.ts_start=parent_metadata->ts_start;
    }
  }
  metadata.level=parent_metadata ?  parent_metadata->level+1 : 0;
  const Scalar isize=Scalar(1.)/metadata.n_points;
  if constexpr(NodeType_::Traits::HasTimestamp) {
    StableAdder_<Eigen::Vector2d> ts_adder;
    ts_adder.reset();
    for (size_t i=node._idx_start; i<node._idx_end; ++i) {
      const auto dts=Traits::stamp(points[i])-metadata.ts_start;
      ts_adder.add(Eigen::Vector2d(dts, dts*dts));
    }
    Eigen::Vector2d ts2=ts_adder.sum()*isize;
    node._dts_mean=ts2(0);
    node._dts_covariance=ts2(1)-ts2(0)*ts2(0);
  }
  
  StableAdder_<AccumulatorVector> accumulator;
  accumulator.reset();
  for (size_t i=node._idx_start; i<node._idx_end; ++i) {
    const auto& v=Traits::coordinates(points[i]);
    AccumulatorVector packed;
    int k=0;
    for (int r=0; r<Dim; ++r)
      for(int c=r; c<Dim; ++c, ++k)
        packed(k)=v(r)*v(c);
    for (int r=0; r<Dim; ++r, ++k)
      packed(k)=v(r);
    accumulator.add(packed);
  }
  AccumulatorVector packed=accumulator.sum()*isize;
  int k=0;
  for (int r=0; r<Dim; ++r)
    for(int c=r; c<Dim; ++c, ++k)
      metadata.cov(r,c)=metadata.cov(c,r)=packed(k);
  for (int r=0; r<Dim; ++r, ++k)
    metadata.mean(r)=packed(k);
  node._mean=metadata.mean;
  metadata.cov-=metadata.mean*metadata.mean.transpose();
  if (metadata.n_points<Dim) {
    node._direction.setZero();
    metadata.is_bad=true;
    return;
  }
  Eigen::SelfAdjointEigenSolver<MatrixType> eig;
  if constexpr (Dim>3) {
    eig.compute(metadata.cov);
  } else {
    eig.computeDirect(metadata.cov);
  }
  metadata.eigenvectors=eig.eigenvectors();
  metadata.eigenvalues=eig.eigenvalues();

  // reorient the normal and the direction
  if (metadata.mean.dot(metadata.eigenvectors.col(0))>0) {
    metadata.eigenvectors.col(0)=-metadata.eigenvectors.col(0);
  }
  if (metadata.mean.dot(metadata.eigenvectors.col(Dim-1))>0) {
    metadata.eigenvectors.col(Dim-1)=-metadata.eigenvectors.col(Dim-1);
  }
  metadata.is_bad=false;
}

} // namespace kd_slam
