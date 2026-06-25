#include "typedefs.h"
#include "cticp_.h"
#include "cticp_impl_.h"

namespace kd_slam::d2 {

  using Geometry=GeometryTraits_<2, Scalar>;
    
  static void addItem(PointsVectorType& dest,
                      Scalar resolution,
                      Scalar width,
                      IsometryType origin) {
    size_t num_points = width / resolution;
    size_t k = dest.size();
    dest.resize(dest.size() + num_points);
    for (; k < dest.size(); ++k) {
      VectorType p = VectorType::Random() * width;
      p.y() = 0;
      PointTraits::coordinates(dest[k]) = origin * p;
    }
  }

  void generateScene(PointsVectorType& dest,
                     Scalar range,
                     Scalar resolution,
                     int num_lines) {
    for (int i = 0; i < num_lines; ++i) {
      Scalar line_span = range * 0.3;
      PerturbationPoseType pert = PerturbationPoseType::Random();
      pert.head<2>() *= range;
      pert(2) *= M_PI;
      IsometryType iso = Geometry::expmap(pert);
      addItem(dest, resolution, line_span, iso);
    }
  }

  static void addItem(std::vector<VectorType>& dest,
                      Scalar resolution,
                      Scalar width,
                      IsometryType origin) {
    size_t num_points = width / resolution;
    size_t k = dest.size();
    dest.resize(dest.size() + num_points);
    for (; k < dest.size(); ++k) {
      VectorType p = VectorType::Random() * width;
      p.y() = 0;
      dest[k] = origin * p;
    }
  }

  void generateScene(std::vector<VectorType>& dest,
                     Scalar range,
                     Scalar resolution,
                     int num_lines) {
    for (int i = 0; i < num_lines; ++i) {
      Scalar line_span = range * 0.3;
      PerturbationPoseType pert = PerturbationPoseType::Random();
      pert.head<2>() *= range;
      pert(2) *= M_PI;
      IsometryType iso = Geometry::expmap(pert);
      addItem(dest, resolution, line_span, iso);
    }
  }

  void linearDeskew(PointsVectorType& dest,
                    const PointsVectorType& src,
                    const PerturbationPoseType& velocities,
                    double t_start) {
    if (t_start < 0) {
      t_start = std::numeric_limits<double>::max();
      for (const auto& p : src)
        t_start = std::min(t_start, (double)PointTraits::stamp(p));
    }
    dest = src;
    for (auto& d : dest) {
      double dt = PointTraits::stamp(d) - t_start;
      IsometryType motion = Geometry::expmap(velocities * (Scalar)dt);
      PointTraits::coordinates(d) = motion * PointTraits::coordinates(d);
    }
  }

  TreeCPUType linearDeskew(const TreeCPUType& src,
                           const PerturbationPoseType& velocities) {
    TreeCPUType dest(src);
    if (!dest.numNodes())
      return dest;
    float root_dt = dest._nodes_storage[0]._dts_mean;
    IsometryType root_motion = Geometry::expmap(velocities * root_dt);
    dest.root_eigenvectors = root_motion.linear() * dest.root_eigenvectors;
    dest.root_mean = root_motion * dest.root_mean;
    for (auto& n : dest._nodes_storage)
      n = CTICP_2D_Traits::applyVelocities(velocities, n);
    return dest;
  }

} // namespace kd_slam::d2
