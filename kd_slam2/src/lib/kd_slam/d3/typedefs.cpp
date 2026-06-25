#include "typedefs.h"
#include "kd_slam/tree/tree_cpu_.h"
#include "cticp_impl_.h"

namespace kd_slam::d3 {
  using Geometry = geometry::GeometryTraits_<3, Scalar>;
  
  static void addItem(PointsVectorType& dest,
                      Scalar resolution,
                      Scalar width,
                      IsometryType origin) {
    size_t num_points = (width * width) / (resolution * resolution);
    size_t k = dest.size();
    dest.resize(dest.size() + num_points);
    for (; k < dest.size(); ++k) {
      VectorType p = VectorType::Random() * width;
      p.z() = 0;
      PointTraits::coordinates(dest[k]) = origin * p;
    }
  }

  void generateScene(PointsVectorType& dest,
                     Scalar range,
                     Scalar resolution,
                     int num_planes) {
    for (int i = 0; i < num_planes; ++i) {
      Scalar plane_side = range * 0.3;
      PerturbationPoseType pert = PerturbationPoseType::Random();
      pert.head<3>() *= range;
      pert.tail<3>().normalize();
      pert.tail<3>() *= drand48();
      IsometryType iso = Geometry::expmap(pert);
      addItem(dest, resolution, plane_side, iso);
    }
  }

  static void addItem(std::vector<VectorType>& dest,
                      Scalar resolution,
                      Scalar width,
                      IsometryType origin) {
    size_t num_points = (width * width) / (resolution * resolution);
    size_t k = dest.size();
    dest.resize(dest.size() + num_points);
    for (; k < dest.size(); ++k) {
      VectorType p = VectorType::Random() * width;
      p.z() = 0;
      dest[k] = origin * p;
    }
  }

  void generateScene(std::vector<VectorType>& dest,
                     Scalar range,
                     Scalar resolution,
                     int num_planes) {
    for (int i = 0; i < num_planes; ++i) {
      Scalar plane_side = range * 0.3;
      PerturbationPoseType pert = PerturbationPoseType::Random();
      pert.head<3>() *= range;
      pert.tail<3>().normalize();
      pert.tail<3>() *= drand48();
      IsometryType iso = Geometry::expmap(pert);
      addItem(dest, resolution, plane_side, iso);
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
    for (auto& d : dest)
      d = CTICP_3D_Traits::applyVelocities(velocities, d);
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
      n = CTICP_3D_Traits::applyVelocities(velocities, n);
    return dest;
  }

} // namespace kd_slam::d3
