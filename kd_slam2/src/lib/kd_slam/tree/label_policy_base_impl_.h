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
    using AccumulatorVector=Eigen::Matrix<double,AccumulatorDim,1>;
    metadata.n_points=node._idx_end-node._idx_start;
    if (!metadata.n_points) {
      metadata.is_bad=true;
      return;
    }
    metadata.level=parent_metadata ?  parent_metadata->level+1 : 0;
    // lambda to compute the min of a set of points
    auto minTs = [&](size_t idx_start, size_t idx_end) -> double { 
      double ts_start=std::numeric_limits<double>::max();
      if constexpr(NodeType_::Traits::HasTimestamp) {
        for (size_t i=idx_start; i<idx_end; ++i) {
          const auto& ts=Traits::stamp(points[i]);
          ts_start=std::min(ts_start, ts);
        }
      }
     return ts_start;
    };
    
    // lambda to accumulate the stamp
    auto accumulate = [&](size_t idx_start, size_t idx_end) -> std::pair<Eigen::Vector2d, AccumulatorVector> {
      //StableAdder_<Eigen::Vector2d> ts_adder;
      //ts_adder.reset();
      Eigen::Vector2d ts_sum=Eigen::Vector2d::Zero();
      AccumulatorVector pt_sum=AccumulatorVector::Zero();
      //StableAdder_<AccumulatorVector> point_adder;
      //point_adder.reset();
      for (size_t i=idx_start; i<idx_end; ++i) {
        if constexpr(NodeType_::Traits::HasTimestamp) {
          const auto dts=Traits::stamp(points[i])-metadata.ts_start;
          ts_sum+=Eigen::Vector2d(dts, dts*dts);
          //ts_adder.add(Eigen::Vector2d(dts, dts*dts));
        }
        AccumulatorVector packed;
        const auto& v=Traits::coordinates(points[i]);
        int k=0;
        for (int r=0; r<Dim; ++r)
          for(int c=r; c<Dim; ++c, ++k)
            packed(k)=v(r)*v(c);
        for (int r=0; r<Dim; ++r, ++k)
          packed(k)=v(r);
        pt_sum+=packed;
        //point_adder.add(packed);
      } 
      //return std::make_pair(ts_adder.sum(), point_adder.sum());
      return std::make_pair(ts_sum, pt_sum);
    };

    if constexpr(NodeType_::Traits::HasTimestamp) {
      if (! parent_metadata) {
        metadata.ts_start=minTs(node._idx_start, node._idx_end);
      } else {
        metadata.ts_start=parent_metadata->ts_start;
      }
    }
    const Scalar isize=Scalar(1.)/metadata.n_points;
    auto acc_result = accumulate(node._idx_start, node._idx_end);
    
    if constexpr(NodeType_::Traits::HasTimestamp) {
      const auto& ts_sum=acc_result.first;
      Eigen::Vector2d ts2=ts_sum*isize;
      node._dts_mean=ts2(0);
      node._dts_covariance=ts2(1)-ts2(0)*ts2(0);
    }
    const auto& pt_sum=acc_result.second*isize;
    int k=0;
    Eigen::Matrix<double, Dim, Dim> d_cov;
    Eigen::Matrix<double, Dim, 1>   d_mean;
  
    for (int r=0; r<Dim; ++r)
      for(int c=r; c<Dim; ++c, ++k)
        d_cov(r,c)=d_cov(c,r)=pt_sum(k);
    for (int r=0; r<Dim; ++r, ++k)
      d_mean(r)=pt_sum(k);
    node._mean=metadata.mean=d_mean.template cast<Scalar>();
    d_cov-=d_mean*d_mean.transpose();
    metadata.cov=d_cov.template cast<Scalar>();
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
