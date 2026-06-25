#pragma once
namespace kd_slam {
  namespace map {

    // Primary template -- specialized per dimension in d2/d3 typedefs.
    // Provides srrg2_solver type aliases used by SLAM_ for PGO variables/factors.
    template <typename Node_>
    struct MapTraits_;

  } // namespace slam
} // namespace kd_slam
