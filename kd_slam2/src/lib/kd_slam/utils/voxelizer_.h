#pragma once
#include <vector>
#include <optional>
#include "srrg_config/configurable.h"
namespace kd_slam {
  namespace utils {
    template <typename Scalar_>
    struct VoxelizerParams_{
      using Scalar=Scalar_;
      Scalar range_min=0;
      Scalar range_max=100;
      Scalar space_res=0;
      Scalar time_res=0;
      bool use_boundary=false;
      bool compute_covariance=false;
    };

    // ---- voxel grid downsampling ---------------------------------------------
    // Bins points by floor(coord/resolution) and averages coordinates per bin.
    // resolution <= 0 returns the input unchanged.


    
    template <typename PointTraits_>
    struct Voxelizer_ : public srrg2_core::Configurable {
      using PointTraits=PointTraits_;
      using Scalar = typename PointTraits::Scalar;
      using PointType=typename PointTraits::PointType;
      using PointCoordinatesType  = typename PointTraits::VectorType;
      using PointsVectorType=std::vector<PointType>;
      using VectorType=typename PointTraits::VectorType;
      using Params=VoxelizerParams_<Scalar>;
      using IsometryType =typename PointTraits_::GeometryTraits::IsometryType;
      static constexpr int Dim=VectorType::RowsAtCompileTime;

      PARAM(srrg2_core::PropertyFloat, range_min,    "min lidar range [m]",               0.f,   &_params_changed);
      PARAM(srrg2_core::PropertyFloat, range_max,    "max lidar range [m]",               100.f, &_params_changed);
      PARAM(srrg2_core::PropertyFloat, space_res,    "spatial voxel res [m]; 0=disable",  0.f,   &_params_changed);
      PARAM(srrg2_core::PropertyFloat, time_res,     "temporal voxel res [s]; 0=disable", 0.f,   &_params_changed);
      PARAM(srrg2_core::PropertyBool,  use_boundary, "conservative boundary clipping",    false, &_params_changed);

      PARAM(srrg2_core::PropertyBool,  compute_covariance, "computes the covariance",    false, &_params_changed);

      struct Accumulator {
        using MatrixType = Eigen::Matrix<Scalar, Dim, Dim>;
        Accumulator();
        void add(const PointType& p, const VectorType& origin, bool with_cov=false);
        inline  PointType mean() {
          normalize();
          return _mean;
        }
        inline VectorType meanCoords() const {
          if (_count == 0) return VectorType::Zero();
          return PointTraits_::coordinates(_sum) * (Scalar(1) / Scalar(_count));
        }
        inline VectorType normal() {
          normalize();
          return _normal;
        }
      protected:
        void normalize();
        int        _count = 0;
        PointType  _sum;
        MatrixType _sum2;
        VectorType _ref = VectorType::Zero();
        bool       _ref_set = false;
        VectorType _normal=VectorType::Zero();
        VectorType _sum_origin=VectorType::Zero();
        PointType _mean;
        int _last_update_count=-1;
      };

      static constexpr int IndexDim =
        PointTraits_::HasTimestamp ?
        VectorType::RowsAtCompileTime + 1:
        VectorType::RowsAtCompileTime;
  
      using IndexType  = Eigen::Matrix<int, IndexDim, 1>;
      struct IndexHash {
        size_t operator()(const IndexType& idx) const {
          size_t seed = 0;
          for (int i = 0; i < idx.size(); ++i)
            seed ^= std::hash<int>()(idx[i]) + 0x9e3779b9u + (seed << 6) + (seed >> 2);
          return seed;
        }
      };
      using IndexMap = std::unordered_map<IndexType, Accumulator, IndexHash>;
      
      IndexType toIndex(const PointType& p)  {
        IndexType index;
        for (int i=0; i<VectorType::RowsAtCompileTime; ++i) {
          index(i)=(int) (PointTraits_::coordinates(p)(i)*_inv_space_resolution);
        }
        if constexpr (PointTraits_::HasTimestamp) {
          double ts = PointTraits_::stamp(p)-_min_ts;
          int ts_idx = params.time_res>0 ? (int) (ts*_inv_time_resolution) : 0;
          index(VectorType::RowsAtCompileTime)=ts_idx;
        }
        return index;
      }

      void clear();
      void add (const PointsVectorType& src,
                const IsometryType& iso=IsometryType::Identity(),
                bool with_cov=false,
                const VectorType& origin=VectorType::Zero());

      void syncParams() const {
        if (!_params_changed)
          return;
        params.range_min    = param_range_min.value();
        params.range_max    = param_range_max.value();
        params.space_res    = param_space_res.value();
        params.time_res     = param_time_res.value();
        params.use_boundary = param_use_boundary.value();
        params.compute_covariance = param_compute_covariance.value();
        _params_changed = false;
      }

      void clip(const VectorType& center, Scalar max_range);
      std::optional<Scalar> query(const PointType& p);
      PointsVectorType voxelize(const PointsVectorType& src);
      
      mutable Params params;

      IndexMap voxels;

    protected:
      mutable bool _params_changed = true;
      Scalar _inv_space_resolution, _inv_time_resolution;
      double _min_ts;
    };
    
  }
}
