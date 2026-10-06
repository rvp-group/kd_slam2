#pragma once
#include "kd_io/message_reader.h"
#include <rosbag2_cpp/reader.hpp>
#include <rcutils/types/uint8_array.h>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

// ---------------------------------------------------------------------------
// Rosbag2MessageReader -- MessageReader backed by rosbag2_cpp.
// open() reads bag metadata to auto-detect the type of each configured topic
// and installs the appropriate deserializer. No type registration from outside.
// ---------------------------------------------------------------------------
class Rosbag2MessageReader : public MessageReader {
public:
  void open() override;
  std::shared_ptr<MessageBase> readOne() override;
  std::shared_ptr<MessageBase> readOne(const std::string& topic,
                                       uint64_t offset,
                                       uint64_t stamp_ns) override;
  bool isGood() const override { return _good; }
  bool isOpen() const override { return _good; }

private:
  using DeserFn = std::function<std::shared_ptr<MessageBase>(const rcutils_uint8_array_t&)>;

  rosbag2_cpp::Reader _reader;
  std::unordered_map<std::string, DeserFn> _handlers;
  bool _good = false;

  static const std::unordered_map<std::string, DeserFn>& typeHandlers();
};
