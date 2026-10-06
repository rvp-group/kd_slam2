#pragma once
#include "message_writer.h"
#include "srrg_boss/serializer.h"

struct BossMessageWriter : public MessageWriter {
  void open() override;
  void write(const std::string& topic, uint64_t ts_ns, MessageBase& msg) override;
  bool isOpen() const ;
  void close() override;
  std::unique_ptr<srrg2_core::Serializer> serializer=nullptr;
};
