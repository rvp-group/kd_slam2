#include "boss_message_reader.h"
#include <stdexcept>

using namespace srrg2_core;

void BossMessageReader::open() {
  if (deserializer)
    deserializer.reset();
  deserializer.reset(new Deserializer);
  deserializer->setFilePath(param_bag_path.value());
  end_reached=false;
  std::sort(param_topics.value().begin(), param_topics.value().end());
}

bool BossMessageReader::isGood() const {
  return deserializer.get();
}

bool BossMessageReader::isOpen() const {
  return deserializer && !end_reached;
}

std::shared_ptr<MessageBase> BossMessageReader::readOne() {
  if (! isGood()){
    return nullptr;
  }
  std::shared_ptr<Serializable> o=nullptr;
  while ((o=deserializer->readObjectShared())) {
    if (!o)
      break;
    auto msg_o=std::dynamic_pointer_cast<MessageBase>(o);
    if (msg_o) {
      if (std::binary_search(param_topics.value().begin(),
                             param_topics.value().end(),
                             msg_o->topic) ) {
        return msg_o;
      }
    }
  }
  deserializer.reset();
  end_reached=true;
  return nullptr;
}
std::shared_ptr<MessageBase> BossMessageReader::readOne(const std::string& topic,
                                                        uint64_t offset,
                                                        uint64_t stamp_ns) {
  throw std::runtime_error("no readOne in boss deserializer");
  return nullptr;
}
