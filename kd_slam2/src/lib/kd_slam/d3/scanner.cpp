#include "scanner.h"

namespace kd_slam::d3 {

void Scanner::makeScan(PointsVectorType& scan,
                       const std::vector<VectorType>& scene,
                       const IsometryType& start_pose,
                       const PerturbationPoseType& velocities,
                       double t_start) {
  float h_fov           = params.alpha_max - params.alpha_min;
  float scan_h_aperture = h_fov / params.num_h_samples;
  float v_fov           = params.phi_max - params.phi_min;
  float scan_v_aperture = v_fov / params.num_v_samples;
  float scan_v_ires     = 1.f / scan_v_aperture;
  float beam_time       = params.scan_time / params.num_h_samples;
  float d2_max          = params.max_range * params.max_range;
  float d2_min          = params.min_range * params.min_range;

  Eigen::Isometry3f pi_left, pi_right;
  pi_left.setIdentity();
  pi_left.linear()  = Eigen::AngleAxisf( 0.5f * scan_h_aperture, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  pi_right.setIdentity();
  pi_right.linear() = Eigen::AngleAxisf(-0.5f * scan_h_aperture, Eigen::Vector3f::UnitZ()).toRotationMatrix();

  std::vector<Eigen::Vector3f> v_rays(params.num_v_samples);
  for (int i = 0; i < params.num_v_samples; ++i)
    v_rays[i] = Eigen::AngleAxisf(i * scan_v_aperture + params.phi_min,
                                  -Eigen::Vector3f::UnitY()) * Eigen::Vector3f::UnitX();

  VectorType tv = velocities.block<3,1>(0,0);
  VectorType rv = velocities.block<3,1>(3,0);
  float r_norm  = rv.norm();
  VectorType r_axis = VectorType::UnitZ();
  if (r_norm > 1e-6f)
    r_axis = rv.normalized();
  else
    r_norm = 0;

  std::vector<float> v_buffer(params.num_v_samples);
  for (int i = 0; i < params.num_h_samples; ++i) {
    double integrated_time = beam_time * i;
    double stamp = t_start + integrated_time;

    IsometryType sensor_origin;
    sensor_origin.setIdentity();
    sensor_origin.linear() = Eigen::AngleAxisf(
        params.alpha_min + scan_h_aperture * i,
        Eigen::Vector3f::UnitZ()).toRotationMatrix();

    IsometryType sensor_motion;
    sensor_motion.linear()      = Eigen::AngleAxisf(r_norm * integrated_time, r_axis).toRotationMatrix();
    sensor_motion.translation() = tv * (float)integrated_time;

    IsometryType world_origin = start_pose * sensor_motion * sensor_origin;
    const VectorType n_left  = world_origin.linear() * pi_left.linear().col(1);
    const VectorType n_right = world_origin.linear() * pi_right.linear().col(1);

    std::fill(v_buffer.begin(), v_buffer.end(), std::numeric_limits<float>::max());

    for (const auto& s : scene) {
      const auto dp = s - world_origin.translation();
      const auto d2 = dp.squaredNorm();
      if (d2 < d2_min || d2 > d2_max) continue;
      if (n_left.dot(dp)  > 0)         continue;
      if (n_right.dot(dp) < 0)         continue;
      VectorType ps = world_origin.linear().transpose() * dp;
      float elevation = atan2(ps.z(), ps.x());
      if (elevation < params.phi_min || elevation > params.phi_max) continue;
      int bin = int((elevation - params.phi_min) * scan_v_ires + 0.5f);
      if (bin < 0 || bin >= params.num_v_samples) continue;
      v_buffer[bin] = std::min(v_buffer[bin], d2);
    }

    for (int j = 0; j < params.num_v_samples; ++j) {
      const float range2 = v_buffer[j];
      if (range2 < d2_min || range2 > d2_max) continue;
      PointType p_out;
      PointTraits::coordinates(p_out) = sensor_origin * v_rays[j] * sqrtf(range2);
      PointTraits::stamp(p_out) = stamp;
      scan.push_back(p_out);
    }
  }
}

} // namespace kd_slam::d3
