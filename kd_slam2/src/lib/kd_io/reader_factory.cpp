#include "reader_factory.h"
#include "rosbag2_message_reader.h"
#include "rosbag2_message_writer.h"

std::shared_ptr<MessageReader> makeRosbag2Reader(
    const std::string& bag_path,
    const std::vector<std::string>& topics) {
  auto r = std::make_shared<Rosbag2MessageReader>();
  r->param_bag_path.setValue(bag_path);
  r->param_topics.value() = topics;
  return r;
}

std::shared_ptr<MessageWriter> makeRosbag2Writer(const std::string& out_path) {
  auto w = std::make_shared<Rosbag2MessageWriter>();
  w->param_out_path.setValue(out_path);
  return w;
}
