#include "scanner.h"
#include "cticp_.h"

namespace kd_slam::d2 {

  using Geometry=GeometryTraits_<2, Scalar>;
  
  void Scanner::makeScan(PointsVectorType& scan,
                         const std::vector<VectorType>& scene,
                         const IsometryType& start_pose,
                         const PerturbationPoseType& velocities,
                         double t_start) {
    const float alpha_step = (params.alpha_max - params.alpha_min) / params.num_beams;
    const float half_step  = 0.5f * alpha_step;
    const float beam_time  = params.scan_time / params.num_beams;
    const float d2_max = params.max_range * params.max_range;
    const float d2_min = params.min_range * params.min_range;

    for (int i = 0; i < params.num_beams; ++i) {
      double integrated_time = beam_time * i;
      double stamp = t_start + integrated_time;
      float alpha = params.alpha_min + alpha_step * i;

      IsometryType scan_origin = IsometryType::Identity();
      scan_origin.linear() = Geometry::rot2d(alpha);

      IsometryType sensor_motion = Geometry::expmap(velocities * (Scalar)integrated_time);

      IsometryType world_origin = start_pose * sensor_motion * scan_origin;

      float best_d2 = d2_max;
      bool  found   = false;

      for (const auto& s : scene) {
        VectorType dp = s - world_origin.translation();
        float d2 = dp.squaredNorm();
        if (d2 < d2_min || d2 > best_d2)
          continue;
        VectorType dp_local = world_origin.linear().transpose() * dp;
        if (dp_local.x() <= 0)
          continue;
        float bearing = atan2f(dp_local.y(), dp_local.x());
        if (bearing < -half_step || bearing > half_step)
          continue;
        best_d2 = d2;
        found   = true;
      }

      if (found) {
        PointType p_out;
        PointTraits::coordinates(p_out) = scan_origin * VectorType(sqrtf(best_d2), 0.f);
        PointTraits::stamp(p_out) = stamp;
        scan.push_back(p_out);
      }
    }
  }

} // namespace kd_slam::d2
