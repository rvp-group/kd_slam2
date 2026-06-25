#include "rosbag2_message_writer.h"
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <kd_slam2_msgs/msg/kd_tree2f.hpp>
#include <kd_slam2_msgs/msg/kd_tree3f.hpp>
#include <kd_slam2_msgs/msg/kd_descriptor.hpp>
#include <cstring>

// ---------------------------------------------------------------------------
// Conversion helpers
// ---------------------------------------------------------------------------

static std_msgs::msg::Header makeHeader(const RosHeader& h) {
  std_msgs::msg::Header r;
  r.stamp.sec     = static_cast<int32_t>(h.stamp_ns / 1'000'000'000ULL);
  r.stamp.nanosec = static_cast<uint32_t>(h.stamp_ns % 1'000'000'000ULL);
  r.frame_id      = h.frame_id;
  return r;
}

static sensor_msgs::msg::Imu toRosMsg(const ImuData& m) {
  sensor_msgs::msg::Imu r;
  r.header                = makeHeader(m.header);
  r.orientation.x         = m.orientation.x();
  r.orientation.y         = m.orientation.y();
  r.orientation.z         = m.orientation.z();
  r.orientation.w         = m.orientation.w();
  r.angular_velocity.x    = m.angular_velocity.x();
  r.angular_velocity.y    = m.angular_velocity.y();
  r.angular_velocity.z    = m.angular_velocity.z();
  r.linear_acceleration.x = m.linear_acceleration.x();
  r.linear_acceleration.y = m.linear_acceleration.y();
  r.linear_acceleration.z = m.linear_acceleration.z();
  return r;
}

static nav_msgs::msg::Odometry toRosMsg(const OdometryData& m) {
  nav_msgs::msg::Odometry r;
  r.header                = makeHeader(m.header);
  r.child_frame_id        = m.child_frame_id;
  r.pose.pose.position.x= m.position.x();
  r.pose.pose.position.y= m.position.y();
  r.pose.pose.position.z= m.position.z();
  r.pose.pose.orientation.x= m.orientation.x();
  r.pose.pose.orientation.y= m.orientation.y();
  r.pose.pose.orientation.z= m.orientation.z();
  r.pose.pose.orientation.w= m.orientation.w();
  r.twist.twist.linear.x   = m.linear_velocity.x();
  r.twist.twist.linear.y   = m.linear_velocity.y();
  r.twist.twist.linear.z   = m.linear_velocity.z();
  r.twist.twist.angular.x   = m.angular_velocity.x();
  r.twist.twist.angular.y   = m.angular_velocity.y();
  r.twist.twist.angular.z   = m.angular_velocity.z();
  Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(r.pose.covariance.data())  = m.pose_covariance;
  Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(r.twist.covariance.data()) = m.twist_covariance;
  return r;
}

static sensor_msgs::msg::PointField makeField(const char* name, uint32_t offset, uint8_t dtype) {
  sensor_msgs::msg::PointField f;
  f.name = name; f.offset = offset; f.datatype = dtype; f.count = 1;
  return f;
}

static sensor_msgs::msg::PointCloud2 toRosMsg(const PointCloudXYZTData& m) {
  sensor_msgs::msg::PointCloud2 r;
  r.header     = makeHeader(m.header);
  r.height     = 1;
  r.width      = static_cast<uint32_t>(m.points.size());
  bool stamped = m.has_per_point_timestamps;
  r.point_step = stamped ? 20u : 12u;
  r.fields     = { makeField("x", 0, sensor_msgs::msg::PointField::FLOAT32),
                   makeField("y", 4, sensor_msgs::msg::PointField::FLOAT32),
                   makeField("z", 8, sensor_msgs::msg::PointField::FLOAT32) };
  if (stamped) r.fields.push_back(makeField("t", 12, sensor_msgs::msg::PointField::FLOAT64));
  r.is_bigendian = false;
  r.row_step     = r.point_step * r.width;
  r.is_dense     = true;
  r.data.resize(r.row_step);
  uint8_t* p = r.data.data();
  for (const auto& pt : m.points) {
    float
      x = pt.coords.x(),
      y = pt.coords.y(),
      z = pt.coords.z();
    memcpy(p, &x, 4); p += 4;
    memcpy(p, &y, 4); p += 4;
    memcpy(p, &z, 4); p += 4;
    if (stamped) { memcpy(p, &pt.t, 8); p += 8; }
  }
  return r;
}

static sensor_msgs::msg::PointCloud2 toRosMsg(const PointCloudXYTData& m) {
  sensor_msgs::msg::PointCloud2 r;
  r.header     = makeHeader(m.header);
  r.height     = 1;
  r.width      = static_cast<uint32_t>(m.points.size());
  bool stamped = m.has_per_point_timestamps;
  r.point_step = stamped ? 16u : 8u;
  r.fields     = { makeField("x", 0, sensor_msgs::msg::PointField::FLOAT32),
                   makeField("y", 4, sensor_msgs::msg::PointField::FLOAT32) };
  if (stamped) r.fields.push_back(makeField("t", 8, sensor_msgs::msg::PointField::FLOAT64));
  r.is_bigendian = false;
  r.row_step     = r.point_step * r.width;
  r.is_dense     = true;
  r.data.resize(r.row_step);
  uint8_t* p = r.data.data();
  for (const auto& pt : m.points) {
    float
      x = pt.coords.x(),
      y = pt.coords.y();
    memcpy(p, &x, 4); p += 4;
    memcpy(p, &y, 4); p += 4;
    if (stamped) { memcpy(p, &pt.t, 8); p += 8; }
  }
  return r;
}

static tf2_msgs::msg::TFMessage toRosMsg(const TFMessageData& m) {
  tf2_msgs::msg::TFMessage r;
  for (const auto& td : m.transforms) {
    geometry_msgs::msg::TransformStamped ts;
    ts.header             = makeHeader(td.header);
    ts.child_frame_id     = td.child_frame_id;
    ts.transform.translation.x = td.translation.x();
    ts.transform.translation.y = td.translation.y();
    ts.transform.translation.z = td.translation.z();
    ts.transform.rotation.x = td.rotation.x();
    ts.transform.rotation.y = td.rotation.y();
    ts.transform.rotation.z = td.rotation.z();
    ts.transform.rotation.w = td.rotation.w();
    r.transforms.push_back(std::move(ts));
  }
  return r;
}

static kd_slam2_msgs::msg::KDDescriptor toRosMsg(const KDDescriptorData& m) {
  kd_slam2_msgs::msg::KDDescriptor r;
  r.header            = makeHeader(m.header);
  r.src_topic         = m.src_topic;
  r.dim               = m.dim;
  r.level             = m.level;
  r.axes_canonization = m.axes_canonization;
  r.root_eigenvectors = m.root_eigenvectors;
  r.entries           = m.entries;
  return r;
}

// ---------------------------------------------------------------------------
// Rosbag2MessageWriter
// ---------------------------------------------------------------------------

template<typename T>
void Rosbag2MessageWriter::_install(const std::string& ros_type) {
  _type_handlers[typeid(Message_<T>)] = {
    ros_type,
    [](rosbag2_cpp::Writer& w, const std::string& topic, uint64_t ts, const MessageBase& m) {
      const T& data = static_cast<const Message_<T>&>(m);
      w.write(toRosMsg(data), topic, rclcpp::Time(static_cast<int64_t>(ts)));
    }
  };
}

// Forward-declare specializations so the constructor picks them up, not the primary template.
template<> void Rosbag2MessageWriter::_install<KDTreeData2f>(const std::string&);
template<> void Rosbag2MessageWriter::_install<KDTreeData3f>(const std::string&);

Rosbag2MessageWriter::Rosbag2MessageWriter() {
  _install<ImuData>            ("sensor_msgs/msg/Imu");
  _install<PointCloudXYZTData> ("sensor_msgs/msg/PointCloud2");
  _install<PointCloudXYTData>  ("sensor_msgs/msg/PointCloud2");
  _install<OdometryData>       ("nav_msgs/Odometry");
  _install<TFMessageData>      ("tf2_msgs/msg/TFMessage");
  _install<KDTreeData2f>       ("kd_slam2_msgs/msg/KDTree2f");
  _install<KDTreeData3f>       ("kd_slam2_msgs/msg/KDTree3f");
  _install<KDDescriptorData>   ("kd_slam2_msgs/msg/KDDescriptor");
}

void Rosbag2MessageWriter::open() {
  rosbag2_storage::StorageOptions opts;
  opts.uri        = param_out_path.value();
  opts.storage_id = "mcap";
  _writer.open(opts);
  _open = true;
}

void Rosbag2MessageWriter::write(const std::string& topic,
                                  uint64_t ts_ns,
                                  const MessageBase& msg) {
  if (!_open) return;
  auto it = _type_handlers.find(typeid(msg));
  if (it == _type_handlers.end()) return;
  if (_created_topics.insert(topic).second) {
    rosbag2_storage::TopicMetadata tm;
    tm.name                 = topic;
    tm.type                 = it->second.ros_type;
    tm.serialization_format = "cdr";
    _writer.create_topic(tm);
  }
  it->second.fn(_writer, topic, ts_ns, msg);
}

void Rosbag2MessageWriter::close() {
  if (!_open) return;
  _open = false;
  _writer.close();
}

// KDTree write handlers need access to the tree internals -- specialize manually.
// The generic _install<T> above calls toRosMsg(data) but KDTreeData2f/3f
// don't have a free toRosMsg -- their serialization is inline here.

// Explicit specializations that replace the generic _install for KDTree types.
template<>
void Rosbag2MessageWriter::_install<KDTreeData2f>(const std::string& ros_type) {
  _type_handlers[typeid(Message_<KDTreeData2f>)] = {
    ros_type,
    [](rosbag2_cpp::Writer& w, const std::string& topic, uint64_t ts, const MessageBase& m) {
      const KDTreeData2f& src = static_cast<const Message_<KDTreeData2f>&>(m);
      using RosTree = kd_slam2_msgs::msg::KDTree2f;
      using RosNode = kd_slam2_msgs::msg::KDNode2f;
      RosTree ros;
      ros.header    = makeHeader(src.header);
      ros.src_topic = src.src_topic;
      ros.ts_start  = src.tree._ts_start;
      for (int i = 0; i < 2; ++i) {
        ros.root_mean[i]        = src.tree.root_mean[i];
        ros.root_eigenvalues[i] = src.tree.root_eigenvalues[i];
      }
      for (int col = 0; col < 2; ++col)
        for (int row = 0; row < 2; ++row)
          ros.root_eigenvectors[col * 2 + row] =
              static_cast<float>(src.tree.root_eigenvectors(row, col));
      ros.nodes.resize(src.tree._num_nodes);
      for (size_t i = 0; i < src.tree._num_nodes; ++i) {
        const auto& n = src.tree._nodes_storage[i];
        RosNode& k    = ros.nodes[i];
        for (int d = 0; d < 2; ++d) k.mean[d]      = n._mean[d];
        for (int d = 0; d < 2; ++d) k.direction[d] = n._direction[d];
        k.idx_start = static_cast<uint32_t>(n._idx_start);
        k.idx_end   = static_cast<uint32_t>(n._idx_end);
        k.idx_left  = n._idx_left; k.idx_right = n._idx_right;
        k.dts_mean  = n._dts_mean; k.dts_covariance = n._dts_covariance;
      }
      w.write(ros, topic, rclcpp::Time(static_cast<int64_t>(ts)));
    }
  };
}

template<>
void Rosbag2MessageWriter::_install<KDTreeData3f>(const std::string& ros_type) {
  _type_handlers[typeid(Message_<KDTreeData3f>)] = {
    ros_type,
    [](rosbag2_cpp::Writer& w, const std::string& topic, uint64_t ts, const MessageBase& m) {
      const KDTreeData3f& src = static_cast<const Message_<KDTreeData3f>&>(m);
      using RosTree = kd_slam2_msgs::msg::KDTree3f;
      using RosNode = kd_slam2_msgs::msg::KDNode3f;
      RosTree ros;
      ros.header    = makeHeader(src.header);
      ros.src_topic = src.src_topic;
      ros.ts_start  = src.tree._ts_start;
      for (int i = 0; i < 3; ++i) {
        ros.root_mean[i]        = src.tree.root_mean[i];
        ros.root_eigenvalues[i] = src.tree.root_eigenvalues[i];
      }
      for (int col = 0; col < 3; ++col)
        for (int row = 0; row < 3; ++row)
          ros.root_eigenvectors[col * 3 + row] =
              static_cast<float>(src.tree.root_eigenvectors(row, col));
      ros.nodes.resize(src.tree._num_nodes);
      for (size_t i = 0; i < src.tree._num_nodes; ++i) {
        const auto& n = src.tree._nodes_storage[i];
        RosNode& k    = ros.nodes[i];
        for (int d = 0; d < 3; ++d) k.mean[d]      = n._mean[d];
        for (int d = 0; d < 3; ++d) k.direction[d] = n._direction[d];
        k.idx_start = static_cast<uint32_t>(n._idx_start);
        k.idx_end   = static_cast<uint32_t>(n._idx_end);
        k.idx_left  = n._idx_left; k.idx_right = n._idx_right;
        k.dts_mean  = n._dts_mean; k.dts_covariance = n._dts_covariance;
      }
      w.write(ros, topic, rclcpp::Time(static_cast<int64_t>(ts)));
    }
  };
}


void __attribute__((constructor)) kd_io_register_writer() {
  BOSS_REGISTER_CLASS(Rosbag2MessageWriter);
}
