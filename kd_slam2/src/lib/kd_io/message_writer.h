#pragma once
#include "slam_messages.h"
#include "srrg_config/configurable.h"
#include <cstdint>
#include <string>

// ---------------------------------------------------------------------------
// MessageWriter -- abstract Configurable interface for writing typed messages.
// Concrete subclasses (e.g. Rosbag2MessageWriter) live in kd_io.
// ---------------------------------------------------------------------------
struct MessageWriter : public srrg2_core::Configurable {
  PARAM(srrg2_core::PropertyString, out_path, "output bag path", "", nullptr);

  virtual void open()  = 0;
  virtual void write(const std::string& topic, uint64_t ts_ns, const MessageBase& msg) = 0;
  virtual bool isOpen() const = 0;
  virtual void close() = 0;
  virtual ~MessageWriter() = default;
};
