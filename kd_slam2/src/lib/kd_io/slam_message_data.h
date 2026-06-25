#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "utils/geometry_.h"
#include "utils/geometry_2d_.h"
#include "utils/geometry_3d_.h"
#include "kd_slam/tree/tree_defs.h"

struct RosHeader {
  uint64_t    stamp_ns = 0;
  std::string frame_id;
};

struct ImuData {
  RosHeader          header;
  Eigen::Quaterniond orientation          = Eigen::Quaterniond::Identity();
  Eigen::Vector3d    angular_velocity     = Eigen::Vector3d::Zero();
  Eigen::Vector3d    linear_acceleration  = Eigen::Vector3d::Zero();
};

struct OdometryData {
  RosHeader          header;
  std::string        child_frame_id;
  Eigen::Vector3d    position             = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation          = Eigen::Quaterniond::Identity();
  Eigen::Matrix<double,6,6> pose_covariance  = Eigen::Matrix<double,6,6>::Zero();
  Eigen::Vector3d    linear_velocity      = Eigen::Vector3d::Zero();
  Eigen::Vector3d    angular_velocity     = Eigen::Vector3d::Zero();
  Eigen::Matrix<double,6,6> twist_covariance = Eigen::Matrix<double,6,6>::Zero();
};

struct TransformData {
  RosHeader          header;
  std::string        child_frame_id;
  Eigen::Vector3d    translation          = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rotation             = Eigen::Quaterniond::Identity();
};

struct TFMessageData {
  std::vector<TransformData> transforms;
};

struct LaserScanData {
  RosHeader header;
  float     angle_min       = 0.f;
  float     angle_max       = 0.f;
  float     angle_increment = 0.f;
  float     range_min       = 0.f;
  float     range_max       = 0.f;
  std::vector<Eigen::Vector2f> points;   // already unprojected to (x,y)
};

struct PointXYZT {
  Eigen::Vector3f coords;
  double t = 0.0;  // absolute timestamp [s]; 0.0 if no per-point field
};

struct PointCloudXYZTData {
  RosHeader header;
  bool has_per_point_timestamps = false;
  std::vector<PointXYZT> points;
};

struct PointLivox {
  Eigen::Vector3f coords;
  uint32_t t;
  float intensity;
  uint8_t tag;
  uint8_t line;
};

struct PointCloudLivox {
  RosHeader header;
  uint64_t timebase = 0;  // abs time of first point, ns (from CustomMsg::timebase)
  bool has_per_point_timestamps = false;
  std::vector<PointLivox> points;
};

struct PointXYT {
  Eigen::Vector2f coords = Eigen::Vector2f::Zero();
  double t = 0.0;  // absolute timestamp [s]; 0.0 if no per-point field
};

struct PointCloudXYTData {
  RosHeader header;
  bool has_per_point_timestamps = false;
  std::vector<PointXYT> points;
};

// Flat (type-erased) descriptor message.
// dim = 2 or 3; level = extraction level; Size = (1<<level)-2
// entries: Size * 2 * dim floats -- interleaved as [m0_0..m0_{dim-1} d0_0..d0_{dim-1} m1... d1...]
// root_eigenvectors: dim*dim floats, col-major -- stored to allow re-canonization without the tree.
struct KDDescriptorData {
  RosHeader   header;
  std::string src_topic;
  int32_t     dim               = 0;
  int32_t     level             = 0;
  int32_t     axes_canonization = -1;
  std::vector<float> root_eigenvectors;  // dim*dim, col-major
  std::vector<float> entries;            // ((1<<level)-2) * 2 * dim
};

struct KDTreeData2f {
  RosHeader        header;
  std::string      src_topic;
  kd_slam::TreeCPUPoint2f tree;    // default-constructed; _points_storage=nullptr, no points
};

struct KDTreeData3f {
  RosHeader        header;
  std::string      src_topic;
  kd_slam::TreeCPUPoint3f tree;    // default-constructed; _points_storage=nullptr, no points
};

