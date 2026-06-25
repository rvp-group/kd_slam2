#include "rosbag2_message_reader.h"
#include <rosbag2_storage/storage_filter.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <kd_slam2_msgs/msg/kd_tree2f.hpp>
#include <kd_slam2_msgs/msg/kd_tree3f.hpp>
#include <kd_slam2_msgs/msg/kd_descriptor.hpp>
#include <cstring>

static rclcpp::SerializedMessage fromRaw(const rcutils_uint8_array_t& data) {
  return rclcpp::SerializedMessage(data);
}

static RosHeader fromRosHeader(const std_msgs::msg::Header& h) {
  RosHeader r;
  r.stamp_ns = static_cast<uint64_t>(h.stamp.sec) * 1'000'000'000ULL + h.stamp.nanosec;
  r.frame_id = h.frame_id;
  return r;
}

static double readTs(const uint8_t* p, int off, uint8_t dtype, double msg_t) {
  using PF = sensor_msgs::msg::PointField;
  if (dtype == PF::UINT32) { uint32_t v; memcpy(&v, p+off, 4); return msg_t + v * 1e-9; }
  if (dtype == PF::INT32)  { int32_t  v; memcpy(&v, p+off, 4); return msg_t + v * 1e-9; }
  if (dtype == PF::FLOAT64){ double   v; memcpy(&v, p+off, 8); return v > 1e8 ? v : msg_t + v; }
  if (dtype == PF::FLOAT32){ float   v; memcpy(&v, p+off, 4); return v > 1e8 ? v : msg_t + v; }
  throw std::runtime_error("can't cast the timestamp");
}

template <typename CloudMsgType>
static std::shared_ptr<MessageBase>
pc2ToCloud(sensor_msgs::msg::PointCloud2& ros) {
  constexpr bool is3D = std::is_same_v<CloudMsgType, PointCloudXYZTData>;
  int off_t = -1; uint8_t type_t = 7;
  for (const auto& f : ros.fields)
    if (f.name == "t" || f.name == "time" || f.name == "timestamp" || f.name == "time_stamp")
      { off_t = static_cast<int>(f.offset); type_t = f.datatype; }
  auto msg = std::make_shared<Message_<CloudMsgType>>();
  msg->header = fromRosHeader(ros.header);
  msg->has_per_point_timestamps = (off_t >= 0);
  const double msg_t = msg->header.stamp_ns * 1e-9;
  const size_t n = static_cast<size_t>(ros.width) * ros.height;
  msg->points.reserve(n);
  sensor_msgs::PointCloud2Iterator<float> it_x(ros, "x"), it_y(ros, "y");
  const uint8_t* raw = ros.data.data();
  if constexpr (is3D) {
    sensor_msgs::PointCloud2Iterator<float> it_z(ros, "z");
    for (size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z, raw += ros.point_step) {
      Eigen::Vector3f coords(*it_x, *it_y, *it_z);
      if (!coords.allFinite()) continue;
      PointXYZT pt;
      pt.coords = coords;
      pt.t = off_t >= 0 ? readTs(raw, off_t, type_t, msg_t) : 0.0;
      msg->points.push_back(pt);
    }
  } else {
    for (size_t i = 0; i < n; ++i, ++it_x, ++it_y, raw += ros.point_step) {
      Eigen::Vector2f coords(*it_x, *it_y);
      if (!coords.allFinite()) continue;
      PointXYT pt;
      pt.coords = coords;
      pt.t = off_t >= 0 ? readTs(raw, off_t, type_t, msg_t) : 0.0;
      msg->points.push_back(pt);
    }
  }
  msg->points.shrink_to_fit();
  return msg;
}

static std::shared_ptr<MessageBase>
pc2ToAny(const rcutils_uint8_array_t& data) {
  auto ser = fromRaw(data);
  sensor_msgs::msg::PointCloud2 ros;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2>{}.deserialize_message(&ser, &ros);
  bool has_z = false;
  for (const auto& f : ros.fields) if (f.name == "z") { has_z = true; break; }
  return has_z ? pc2ToCloud<PointCloudXYZTData>(ros) : pc2ToCloud<PointCloudXYTData>(ros);
}

static std::shared_ptr<MessageBase>
imuToMsg(const rcutils_uint8_array_t& data) {
  auto ser = fromRaw(data);
  sensor_msgs::msg::Imu ros;
  rclcpp::Serialization<sensor_msgs::msg::Imu>{}.deserialize_message(&ser, &ros);
  auto msg = std::make_shared<Message_<ImuData>>();
  msg->header           = fromRosHeader(ros.header);
  msg->orientation      = Eigen::Quaterniond(
      ros.orientation.w, ros.orientation.x,
      ros.orientation.y, ros.orientation.z);
  msg->angular_velocity    = {ros.angular_velocity.x,
                               ros.angular_velocity.y,
                               ros.angular_velocity.z};
  msg->linear_acceleration = {ros.linear_acceleration.x,
                               ros.linear_acceleration.y,
                               ros.linear_acceleration.z};
  return msg;
}

static std::shared_ptr<MessageBase>
tfMessageToMsg(const rcutils_uint8_array_t& data) {
  auto ser = fromRaw(data);
  tf2_msgs::msg::TFMessage ros;
  rclcpp::Serialization<tf2_msgs::msg::TFMessage>{}.deserialize_message(&ser, &ros);
  auto msg = std::make_shared<Message_<TFMessageData>>();
  for (const auto& t : ros.transforms) {
    TransformData td;
    td.header         = fromRosHeader(t.header);
    td.child_frame_id = t.child_frame_id;
    td.translation    = {t.transform.translation.x,
                         t.transform.translation.y,
                         t.transform.translation.z};
    td.rotation       = Eigen::Quaterniond(t.transform.rotation.w,
                                           t.transform.rotation.x,
                                           t.transform.rotation.y,
                                           t.transform.rotation.z);
    msg->transforms.push_back(std::move(td));
  }
  return msg;
}

static std::shared_ptr<MessageBase>
odometryToMsg(const rcutils_uint8_array_t& data) {
  auto ser = fromRaw(data);
  nav_msgs::msg::Odometry ros;
  rclcpp::Serialization<nav_msgs::msg::Odometry>{}.deserialize_message(&ser, &ros);
  auto msg = std::make_shared<Message_<OdometryData>>();
  msg->header           = fromRosHeader(ros.header);
  msg->child_frame_id   = ros.child_frame_id;
  msg->position         = {ros.pose.pose.position.x,
                           ros.pose.pose.position.y,
                           ros.pose.pose.position.z};
  msg->orientation      = Eigen::Quaterniond(ros.pose.pose.orientation.w,
                                             ros.pose.pose.orientation.x,
                                             ros.pose.pose.orientation.y,
                                             ros.pose.pose.orientation.z);
  msg->linear_velocity = {ros.twist.twist.linear.x,
                          ros.twist.twist.linear.y,
                          ros.twist.twist.linear.z};
  msg->angular_velocity    = {ros.twist.twist.angular.x,
                               ros.twist.twist.angular.y,
                               ros.twist.twist.angular.z};
  msg->pose_covariance  = Eigen::Map<const Eigen::Matrix<double,6,6,Eigen::RowMajor>>(ros.pose.covariance.data());
  msg->twist_covariance = Eigen::Map<const Eigen::Matrix<double,6,6,Eigen::RowMajor>>(ros.twist.covariance.data());
  return msg;
}

template <typename T> constexpr int DimFromTree_;
template<> constexpr int DimFromTree_<kd_slam2_msgs::msg::KDTree2f> = 2;
template<> constexpr int DimFromTree_<kd_slam2_msgs::msg::KDTree3f> = 3;

template <typename T> struct TreeDataFromRos_;
template<> struct TreeDataFromRos_<kd_slam2_msgs::msg::KDTree2f> { using type = KDTreeData2f; };
template<> struct TreeDataFromRos_<kd_slam2_msgs::msg::KDTree3f> { using type = KDTreeData3f; };

template <typename RosMsgType_>
static std::shared_ptr<MessageBase>
kdTreefToMsg_(const rcutils_uint8_array_t& data) {
  static constexpr int Dim=DimFromTree_<RosMsgType_>;
  using TreeDataType = typename TreeDataFromRos_<RosMsgType_>::type;
  auto ser = fromRaw(data);
  RosMsgType_ ros;
  rclcpp::Serialization<RosMsgType_>{}.deserialize_message(&ser, &ros);
  auto msg = std::make_shared<Message_<TreeDataType>>();
  msg->header    = fromRosHeader(ros.header);
  msg->src_topic = ros.src_topic;
  auto& tree     = msg->tree;
  tree._ts_start = ros.ts_start;
  for (int i = 0; i < Dim; ++i) {
    tree.root_mean[i]        = ros.root_mean[i];
    tree.root_eigenvalues[i] = ros.root_eigenvalues[i];
  }
  for (int col = 0; col < Dim; ++col)
    for (int row = 0; row < Dim; ++row)
      tree.root_eigenvectors(row, col) = ros.root_eigenvectors[col * Dim + row];
  size_t n = ros.nodes.size();
  tree._nodes_storage.resize(n);
  for (size_t i = 0; i < n; ++i) {
    const auto& k = ros.nodes[i];
    auto& nd      = tree._nodes_storage[i];
    for (int d = 0; d < Dim; ++d) nd._mean[d]      = k.mean[d];
    for (int d = 0; d < Dim; ++d) nd._direction[d] = k.direction[d];
    nd._idx_start = k.idx_start; nd._idx_end  = k.idx_end;
    nd._idx_left  = k.idx_left;  nd._idx_right = k.idx_right;
    nd._dts_mean  = k.dts_mean;  nd._dts_covariance = k.dts_covariance;
  }
  tree._nodes_ptr = n ? tree._nodes_storage.data() : nullptr;
  tree._num_nodes = n;
  return msg;
}

static std::shared_ptr<MessageBase>
kdDescriptorToMsg(const rcutils_uint8_array_t& data) {
  auto ser = fromRaw(data);
  kd_slam2_msgs::msg::KDDescriptor ros;
  rclcpp::Serialization<kd_slam2_msgs::msg::KDDescriptor>{}.deserialize_message(&ser, &ros);
  auto msg = std::make_shared<Message_<KDDescriptorData>>();
  msg->header            = fromRosHeader(ros.header);
  msg->src_topic         = ros.src_topic;
  msg->dim               = ros.dim;
  msg->level             = ros.level;
  msg->axes_canonization = ros.axes_canonization;
  msg->root_eigenvectors = ros.root_eigenvectors;
  msg->entries           = ros.entries;
  return msg;
}

// ---------------------------------------------------------------------------
// Static type-string -> deserializer registry
// ---------------------------------------------------------------------------

const std::unordered_map<std::string, Rosbag2MessageReader::DeserFn>&
Rosbag2MessageReader::typeHandlers() {
  static const std::unordered_map<std::string, DeserFn> m = {
    {"sensor_msgs/msg/PointCloud2",       pc2ToAny},
    {"sensor_msgs/msg/Imu",              imuToMsg},
    {"nav_msgs/msg/Odometry",            odometryToMsg},
    {"tf2_msgs/msg/TFMessage",           tfMessageToMsg},
    {"kd_slam2_msgs/msg/KDTree2f",       kdTreefToMsg_<kd_slam2_msgs::msg::KDTree2f>},
    {"kd_slam2_msgs/msg/KDTree3f",       kdTreefToMsg_<kd_slam2_msgs::msg::KDTree3f>},
    {"kd_slam2_msgs/msg/KDDescriptor",   kdDescriptorToMsg},
  };
  return m;
}

// ---------------------------------------------------------------------------
// Rosbag2MessageReader methods
// ---------------------------------------------------------------------------

void Rosbag2MessageReader::open() {
  const auto& path   = param_bag_path.value();
  const auto& topics = param_topics.value();

  _reader.open(path);

  const auto& type_map = typeHandlers();

  // Build topic -> handler from bag metadata + type registry
  std::vector<std::string> filter_topics;
  for (const auto& meta : _reader.get_metadata().topics_with_message_count) {
    const std::string& topic_name = meta.topic_metadata.name;
    const std::string& type_name  = meta.topic_metadata.type;

    // TF static topics are always subscribed regardless of the topics filter
    bool is_tf = (type_name == "tf2_msgs/msg/TFMessage");
    if (!is_tf && !topics.empty()) {
      bool found = false;
      for (const auto& t : topics) if (t == topic_name) { found = true; break; }
      if (!found) continue;
    }

    auto it = type_map.find(type_name);
    if (it == type_map.end()) {
      std::cerr << "[Rosbag2MessageReader] unsupported type " << type_name
                << " on topic " << topic_name << " -- skipping\n";
      continue;
    }
    _handlers[topic_name] = it->second;
    filter_topics.push_back(topic_name);
  }

  if (!filter_topics.empty()) {
    rosbag2_storage::StorageFilter filter;
    filter.topics = filter_topics;
    _reader.set_filter(filter);
  }
  _good = true;
}

std::shared_ptr<MessageBase>
Rosbag2MessageReader::readOne(const std::string& topic,
                               uint64_t /*offset*/,
                               uint64_t stamp_ns) {
  auto it = _handlers.find(topic);
  if (it == _handlers.end()) return nullptr;
  _reader.seek(stamp_ns);
  while (_reader.has_next()) {
    auto bag_msg = _reader.read_next();
    if (bag_msg->topic_name != topic) continue;
    auto dt = static_cast<int64_t>(bag_msg->recv_timestamp) - static_cast<int64_t>(stamp_ns);
    if (std::abs(dt) < 1000) {
      auto m = it->second(*bag_msg->serialized_data);
      if (!m) continue;
      m->topic        = topic;
      m->log_stamp_ns = static_cast<uint64_t>(bag_msg->recv_timestamp);
      return m;
    }
    if (dt > 10000) return nullptr;
  }
  return nullptr;
}

std::shared_ptr<MessageBase> Rosbag2MessageReader::readOne() {
  while (_reader.has_next()) {
    auto bag_msg = _reader.read_next();
    auto it = _handlers.find(bag_msg->topic_name);
    if (it == _handlers.end()) continue;
    auto m = it->second(*bag_msg->serialized_data);
    if (!m) continue;
    m->topic        = bag_msg->topic_name;
    m->log_stamp_ns = static_cast<uint64_t>(bag_msg->recv_timestamp);
    return m;
  }
  _good = false;
  return nullptr;
}

void __attribute__((constructor)) kd_io_register_reader() {
  BOSS_REGISTER_CLASS(Rosbag2MessageReader);
}
