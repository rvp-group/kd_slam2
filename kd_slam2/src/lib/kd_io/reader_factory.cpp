#include "reader_factory.h"
#include "srrg_boss/serializable.h"

using namespace srrg2_core;
std::shared_ptr<MessageReader> makeMessageReader(const std::string& type,
                                                 const std::string& bag_path,
                                                 const std::vector<std::string>& topics) {
  auto inst=Serializable::createInstance(type);
  auto r=dynamic_cast<MessageReader*>(inst);
  if (!r)
    return nullptr;
  r->param_bag_path.setValue(bag_path);
  r->param_topics.value() = topics;
  return std::shared_ptr<MessageReader>(r);
}

std::shared_ptr<MessageWriter> makeMessageWriter(const std::string& type,
                                          const std::string& out_path) {
  auto inst=Serializable::createInstance(type);
  auto w=dynamic_cast<MessageWriter*>(inst);
  if (!w)
    return nullptr;
  w->param_out_path.setValue(out_path);
  return std::shared_ptr<MessageWriter>(w);
}
