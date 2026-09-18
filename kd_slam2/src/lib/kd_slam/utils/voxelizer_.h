#pragma once
#include <vector>
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

      static constexpr int IndexDim =
        PointTraits_::HasTimestamp ?
        VectorType::RowsAtCompileTime + 1:
        VectorType::RowsAtCompileTime;
  
      PointsVectorType voxelize(const PointsVectorType& src);
      
      void syncParams() {
        if (!_params_changed)
          return;
        params.range_min    = param_range_min.value();
        params.range_max    = param_range_max.value();
        params.space_res    = param_space_res.value();
        params.time_res     = param_time_res.value();
        _params_changed = false;
      }
      VoxelizerParams_<Scalar> params;
    protected:
      mutable bool _params_changed = true;
      Scalar _inv_space_resolution, _inv_time_resolution;
      double _min_ts;
      size_t _previous_voxel_size=0;
    };
    
  }
}
