#include "message_reader.h"
#include "srrg_boss/deserializer.h"
// ---------------------------------------------------------------------------
// MessageReader -- abstract Configurable interface for reading typed messages.
// Concrete subclasses (e.g. Rosbag2MessageReader) live in kd_io.
// ---------------------------------------------------------------------------
struct BossMessageReader : public MessageReader {
  
  void open()  override;
  bool isGood() const override;
  bool isOpen() const override;
  std::shared_ptr<MessageBase> readOne() ;
  std::shared_ptr<MessageBase> readOne(const std::string& topic,
                                       uint64_t offset,
                                       uint64_t stamp_ns) override;
  std::unique_ptr<srrg2_core::Deserializer> deserializer=nullptr;
  bool end_reached=true;
};
