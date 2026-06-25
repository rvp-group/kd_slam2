#pragma once
#include "slam_messages.h"
#include "srrg_config/configurable.h"
#include "srrg_property/property_vector.h"
#include <memory>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// MessageReader -- abstract Configurable interface for reading typed messages.
// Concrete subclasses (e.g. Rosbag2MessageReader) live in kd_io.
// ---------------------------------------------------------------------------
struct MessageReader : public srrg2_core::Configurable {
  PARAM(srrg2_core::PropertyString,               bag_path, "input bag path",    "", nullptr);
  PARAM_VECTOR(srrg2_core::PropertyVector_<std::string>, topics, "topics to read", nullptr);

  virtual void open()  = 0;
  virtual bool isGood() const = 0;
  virtual bool isOpen() const = 0;
  virtual std::shared_ptr<MessageBase> readOne() = 0;
  virtual std::shared_ptr<MessageBase> readOne(const std::string& topic,
                                               uint64_t offset,
                                               uint64_t stamp_ns) = 0;
  virtual ~MessageReader() = default;
};
