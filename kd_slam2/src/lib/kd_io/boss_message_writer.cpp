#include "boss_message_writer.h"
using namespace srrg2_core;

void BossMessageWriter::open() {
  if (serializer)
    serializer.reset();
  serializer.reset(new Serializer);
  serializer->setFilePath(param_out_path.value());
  serializer->setBinaryPath(param_out_path.value()+".d/<classname>.<nameAttribute>.<id>.<ext>");
}
void BossMessageWriter::write(const std::string& topic, uint64_t ts_ns,  MessageBase& msg) {
  if (! serializer.get())
    throw std::runtime_error("no file open");
  msg.topic = topic;
  msg.log_stamp_ns=ts_ns;
  msg.file_offset=0;
  serializer->writeObject(msg);
  
}
bool BossMessageWriter::isOpen() const  {
  return (serializer.get());
}
void BossMessageWriter::close() {
  serializer.reset();
}
