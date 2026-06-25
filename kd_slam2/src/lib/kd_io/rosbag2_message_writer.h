#pragma once
#include "message_writer.h"
#include "slam_messages.h"
#include <rosbag2_cpp/writer.hpp>
#include <functional>
#include <set>
#include <string>
#include <typeindex>
#include <unordered_map>

// ---------------------------------------------------------------------------
// Rosbag2MessageWriter -- MessageWriter backed by rosbag2_cpp.
// open() uses param_out_path. Topics are created automatically on first write.
// Supported types: ImuData, PointCloudXYZTData, PointCloudXYTData,
//                  KDTreeData2f, KDTreeData3f, KDDescriptorData.
// ---------------------------------------------------------------------------
class Rosbag2MessageWriter : public MessageWriter {
public:
  Rosbag2MessageWriter();

  void open() override;
  void write(const std::string& topic, uint64_t ts_ns, const MessageBase& msg) override;
  bool isOpen() const override { return _open; }
  void close() override;

private:
  struct TypeHandler {
    std::string ros_type;
    std::function<void(rosbag2_cpp::Writer&, const std::string&, uint64_t, const MessageBase&)> fn;
  };

  rosbag2_cpp::Writer _writer;
  std::unordered_map<std::type_index, TypeHandler> _type_handlers;
  std::set<std::string> _created_topics;
  bool _open = false;

  template<typename T>
  void _install(const std::string& ros_type);
};
