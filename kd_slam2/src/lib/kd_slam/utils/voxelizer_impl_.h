#pragma once
#include "voxelizer_.h"
#include <cmath>
#include <array>
#include <algorithm>
#include <unordered_map>
#include <Eigen/Eigenvalues>

namespace kd_slam {
  namespace utils {
    using namespace std;
    template <typename PointTraits_>
    typename Voxelizer_<PointTraits_>::PointsVectorType
    Voxelizer_<PointTraits_>::voxelize(const PointsVectorType& src) {
      syncParams();
      _inv_space_resolution=params.space_res>0?1./params.space_res:0;
      _inv_time_resolution=params.time_res>0?1./params.time_res:0;
      using IndexType  = std::array<int, IndexDim>;
      struct PointEntry {
        IndexType idx;
        size_t pos;
        inline bool operator<(const PointEntry& other){
          for (int i=0; i<IndexDim; ++i) {
            if (idx[i]<other.idx[i]) return true;
            if (idx[i]>other.idx[i]) return false;
          }
          return false;
        }
      };
        
      PointsVectorType dest;
      if (src.empty()) {
        return dest;
      }
      
      std::vector<PointEntry> point_entries;
      point_entries.resize(src.size());
      double start_ts=0;
      if constexpr (PointTraits_::HasTimestamp) {
        start_ts=PointTraits_::stamp(src[0]);
      }
      Scalar d2_min=pow(params.range_min,2);
      Scalar d2_max=pow(params.range_max,2);
        
      //clip and add
      size_t num_entries=0;
      dest.reserve(src.size());
      
      for(size_t i=0; i<src.size(); ++i){
        const auto& p=src[i];
        const auto& coords=PointTraits_::coordinates(p);
        auto d2=coords.squaredNorm();
        if (d2<d2_min || d2>d2_max)
          continue;
        if (_inv_space_resolution==0) {
          dest.push_back(p);
          continue;
        }
        auto& index = point_entries[num_entries].idx;
        for (int i=0; i<VectorType::RowsAtCompileTime; ++i) {
          index[i]=(int) (PointTraits_::coordinates(p)(i)*_inv_space_resolution);
        }
        if constexpr (PointTraits_::HasTimestamp) {
          double ts=PointTraits_::stamp(p);
          int ts_idx = params.time_res>0 ? (int) ((ts-start_ts)*_inv_time_resolution) : 0;
          index[VectorType::RowsAtCompileTime]=ts_idx;
        }
        point_entries[num_entries].pos=i;
        ++num_entries;
      }
      if (_inv_space_resolution==0) {
        return dest;
      }
      if (! num_entries)
        return dest;

      std::sort(point_entries.begin(), point_entries.begin()+num_entries);

      PointEntry prev=point_entries[0];
      const PointType& prev_pt=src[prev.pos];
      PointCoordinatesType coords_acc=PointTraits::coordinates(prev_pt);
      double ts_acc=0;
      if constexpr (PointTraits_::HasTimestamp) {
        ts_acc = PointTraits::stamp(prev_pt);
      }
      int cnt=1;
      for (size_t i=1; i<num_entries; ++i) {
        const auto& curr=point_entries[i];
        auto p=src[curr.pos];
        if (prev<curr) {
          PointType p_dest;
          PointTraits::coordinates(p_dest)=coords_acc*(1./cnt);
          if constexpr (PointTraits_::HasTimestamp) {
            PointTraits::stamp(p_dest)=ts_acc*(1./cnt);
          }
          dest.push_back(p_dest);
          cnt=0;
          ts_acc=0;
          coords_acc.setZero();
        } 
        if constexpr (PointTraits_::HasTimestamp) {
          ts_acc+=PointTraits::stamp(p);
        }
        coords_acc+=PointTraits::coordinates(p);
        ++cnt;
        prev=curr;
      }
      PointType p_dest;
      PointTraits::coordinates(p_dest)=coords_acc*(1./cnt);
      if constexpr (PointTraits_::HasTimestamp) {
        PointTraits::stamp(p_dest)=ts_acc*(1./cnt);
      }
      dest.push_back(p_dest);
      dest.shrink_to_fit();
      return dest;
    }
  }
}
