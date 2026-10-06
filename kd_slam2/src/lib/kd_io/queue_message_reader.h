#pragma once
#include "message_reader.h"
#include "message_queue.h"

// ---------------------------------------------------------------------------
// MessageReader -- abstract Configurable interface for reading typed messages.
// Concrete subclasses (e.g. Rosbag2MessageReader) live in kd_io.
// ---------------------------------------------------------------------------
struct QueueMessageReader : public MessageReader {
  void open()  ;
  bool isGood() const override;
  bool isOpen() const override;
  std::shared_ptr<MessageBase> readOne() override ;
  std::shared_ptr<MessageBase> readOne(const std::string& topic,
                                       uint64_t offset,
                                       uint64_t stamp_ns) override;
  std::shared_ptr<MessageQueueBounded> q; // queue to drain
  bool good=false;
};
