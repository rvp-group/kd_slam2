#pragma once
#include "voxelizer_.h"
#include <cmath>
#include <unordered_map>
#include <Eigen/Eigenvalues>

namespace kd_slam {
  namespace utils {


    template <typename PointTraits_>
    Voxelizer_<PointTraits_>::Accumulator::Accumulator() {
      PointTraits_::coordinates(_sum)=VectorType::Zero();
      _sum2=MatrixType::Zero();
      _sum_origin=VectorType::Zero();
      if constexpr (PointTraits_::HasTimestamp)
        PointTraits_::stamp(_sum) = 0;
    }
    
    template <typename PointTraits_>
    void Voxelizer_<PointTraits_>::Accumulator::add(const typename PointTraits_::PointType& p,
                                                          const VectorType& origin,
                                                          bool with_cov) {
      ++_count;
      const auto& coords=PointTraits_::coordinates(p);
      PointTraits_::coordinates(_sum) += coords;
      _sum_origin += origin;
      if (with_cov) {
        if (!_ref_set) { _ref = coords; _ref_set = true; }
        const VectorType delta = coords - _ref;
        _sum2.noalias() += delta * delta.transpose();
      }
      if constexpr (PointTraits_::HasTimestamp) {
        PointTraits_::stamp(_sum)+=(PointTraits_::stamp(p));
      }
    }
    
    template <typename PointTraits_>
    void Voxelizer_<PointTraits_>::Accumulator::normalize() {
      if (_count) {
        if (_last_update_count==_count)
          return;
        PointTraits_::coordinates(_mean) = PointTraits_::coordinates(_sum)*(Scalar(1)/Scalar(_count));
        if constexpr (PointTraits_::HasTimestamp) {
          PointTraits_::stamp(_mean) = PointTraits_::stamp(_sum)*(Scalar(1)/Scalar(_count));
        }
        _normal.setZero();
        
        if (_sum2!=MatrixType::Zero() && _count>Dim+1) {
          const VectorType local_mean = PointTraits_::coordinates(_mean) - _ref;
          auto cov=_sum2*(Scalar(1.)/_count) - local_mean*local_mean.transpose();
          Eigen::SelfAdjointEigenSolver<MatrixType> solver;
          solver.computeDirect(cov);
          _normal = solver.eigenvectors().col(0);
          const VectorType mean_origin = _sum_origin * (Scalar(1) / Scalar(_count));
          if (_normal.dot(mean_origin - PointTraits_::coordinates(_mean)) < Scalar(0))
            _normal = -_normal;
          

        }
        _last_update_count=_count;
      }
    }

    template <typename PointTraits_>
    void Voxelizer_<PointTraits_>::clear() {
      voxels.clear();
    }
    
    template <typename PointTraits_>
    void Voxelizer_<PointTraits_>::add(const PointsVectorType& src,
                                       const typename PointTraits_::GeometryTraits::IsometryType& iso,
                                       bool with_cov,
                                       const VectorType& origin) {

      Scalar boundary=params.use_boundary?Scalar(sqrt(params.space_res*Dim)):Scalar(0);

      _inv_space_resolution=0;
      if (params.space_res>0) {
        _inv_space_resolution=Scalar(1)/params.space_res;
      }
      _inv_time_resolution=0;
      if (params.time_res>0) {
        _inv_time_resolution=Scalar(1)/params.time_res;
      }

      _min_ts=0;
      // scan for the min ts to avoid numerical issues
      if constexpr (PointTraits_::HasTimestamp) {
        _min_ts=std::numeric_limits<double>::max();
        for (const auto& p : src) {
          _min_ts=std::min(_min_ts, PointTraits_::stamp(p));
        }
      }

      // we build the voxels with all points whose squared_norm is <max_range+voxel_boundary)^2
      Scalar upper_bound2=pow(params.range_max+boundary,2);
      Scalar lower_bound = std::max(Scalar(0), params.range_min-boundary);
      Scalar lower_bound2=pow(lower_bound, 2);
      for (const auto& p : src) {
        Scalar n2=PointTraits::coordinates(p).squaredNorm();
        if (n2>upper_bound2 || n2<lower_bound2)
          continue;
        PointType pw=p;
        PointTraits_::coordinates(pw)=iso*PointTraits_::coordinates(p);
        IndexType idx = toIndex(pw);
        voxels[idx].add(pw, origin, with_cov);
      }
    }

    template <typename PointTraits_>
    typename Voxelizer_<PointTraits_>::PointsVectorType
    Voxelizer_<PointTraits_>::voxelize(const PointsVectorType& src)  {
      syncParams();
      clear();
      PointsVectorType dest;
      dest.reserve (src.size());
      auto rmin2=pow(params.range_min, 2);
      auto rmax2=pow(params.range_max, 2);
      if (params.space_res <= 0.f) {
        dest.resize(src.size());
        int k=0;
        for (size_t i=0; i<src.size(); ++i) {
          const auto& p=src[i];
          Scalar n2=PointTraits::coordinates(p).squaredNorm();
          if (n2<rmin2||n2>rmax2)
            continue;
          dest[k]=p;
          ++k;
        }
        dest.resize(k);
        dest.shrink_to_fit();
        return dest;
      }
      add(src);
      for (auto& [idx, acc] : voxels) {
        auto out_p=acc.mean();
        Scalar n2=PointTraits::coordinates(out_p).squaredNorm();
        if (n2>rmax2 || n2<rmin2)
          continue;
        dest.push_back(out_p);
      }
      dest.shrink_to_fit();
      return dest;
    }
    

    template <typename PointTraits_>
    void Voxelizer_<PointTraits_>::clip(const VectorType& center, Scalar max_range) {
      const Scalar max_range2 = max_range * max_range;
      for (auto it = voxels.begin(); it != voxels.end(); ) {
        if ((it->second.meanCoords() - center).squaredNorm() > max_range2)
          it = voxels.erase(it);
        else
          ++it;
      }
    }

    template <typename PointTraits_>
    std::optional<typename Voxelizer_<PointTraits_>::Scalar>
    Voxelizer_<PointTraits_>::query(const PointType& p) {
      const auto idx = toIndex(p);
      auto it = voxels.find(idx);
      if (it == voxels.end())
        return std::nullopt;
      auto& acc = it->second;
      const VectorType n = acc.normal();
      if (n.squaredNorm() < Scalar(0.5))
        return std::nullopt;
      const VectorType mean_c = PointTraits_::coordinates(acc.mean());
      const VectorType dp = PointTraits_::coordinates(p) - mean_c;
      return n.dot(dp);
    }

  }
}
