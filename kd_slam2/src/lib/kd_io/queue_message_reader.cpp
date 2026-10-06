#include "queue_message_reader.h"

void QueueMessageReader::open() {
  if (! q)
    throw std::runtime_error ("no queue");
  good=true;
}
bool QueueMessageReader::isGood() const {
  return good;
}

bool QueueMessageReader::isOpen() const {
  return good;
    
}

std::shared_ptr<MessageBase> QueueMessageReader::readOne(){
  auto m=q->pop();
  if (!m) {
    good=false;
  }
  return m;
}

std::shared_ptr<MessageBase> QueueMessageReader::readOne(const std::string& ,
                                                         uint64_t,
                                                         uint64_t) {
  throw std::runtime_error("not implemented");
}

