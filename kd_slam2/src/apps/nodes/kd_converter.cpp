#include <iostream>
#include <string>
#include "srrg_system_utils/parse_command_line.h"
#include "srrg_system_utils/system_utils.h"
#include "srrg_property/property_container_manager.h"
#include "kd_slam/utils/time_utils.h"
#include "kd_slam/frame/frame_queue_bounded.h"
#include "kd_io/tree_loader_.h"
#include "kd_io/rosbag2_message_writer.h"
#include "kd_slam/d2/typedefs.h"
#include "kd_slam/d3/typedefs.h"
#include "kd_slam/event/event.h"
#include "kd_slam/event/event_queue.h"
#include "kd_viewer/kd_viewer.h"
#include "kd_viewer/drawable_kd_slam_.h"
#include "kd_slam/slam/slam_proc_.h"
#include "kd_slam/cuda/cuda_common.h"
#include "kd_slam/event/event_logger.h"
#include "kd_io/kd_tum_writer_.h"
#include "kd_io/kd_state_writer_.h"
#include "kd_io/tum_io.h"
#include "instances_2d.h"
#include "instances_3d.h"

using namespace srrg2_core;
using namespace kd_slam;
using namespace kd_slam::frame;
using namespace kd_slam::event;

const char* banner[] = {
  "kd_converter: config-driven converter",
  "usage: kd_converter [options]",
  nullptr
};

template <typename NodeType_>
struct AppTraits_ {
  using NodeType         = NodeType_;
  using TreeBaseType     = kd_slam::Tree_<NodeType>;
  using TreeCPUType      = kd_slam::TreeCPU_<TreeBaseType>;
  using LoaderType       = kd_slam::TreeLoader_<NodeType>;
  static constexpr int Dim   = NodeType_::Dim;
};

using AppTraits2D = AppTraits_<d2::NodeType>;
using AppTraits3D = AppTraits_<d3::NodeType>;


template <typename AppTraits>
static int run(PropertyContainerManager& manager,
               const std::string& loader_name,
               srrg2_core::ArgumentString& a_input,
               srrg2_core::ArgumentString& a_cloud_topic,
               srrg2_core::ArgumentString& a_imu_topic,
               srrg2_core::ArgumentString& a_tree_topic,
               srrg2_core::ArgumentString& a_output) {

  using namespace std;
  using LoaderType      = typename AppTraits::LoaderType;

  auto loader_ = manager.getByName(loader_name);
  if (!loader_) {
    cerr << "loader '" << loader_name << "' not found\n";
    return 1;
  }
  auto loader = std::dynamic_pointer_cast<LoaderType>(loader_);
  if (!loader) {
    cerr << "wrong loader type: " << loader_->className() << "\n";
    return 1;
  }

  {
    auto reader = loader->param_reader.value();
    if (reader) {
      if (a_input.isSet())      reader->param_bag_path.setValue(a_input.value());
      if (a_cloud_topic.isSet()) reader->param_topics.value().push_back(a_cloud_topic.value());
      if (a_tree_topic.isSet())  reader->param_topics.value().push_back(a_tree_topic.value());
      if (a_imu_topic.isSet())   reader->param_topics.value().push_back(a_imu_topic.value());
    }
  }
  if (a_output.isSet()) {
    auto writer = loader->param_writer.value();
    if (!writer) {
      writer = std::make_shared<Rosbag2MessageWriter>();
      loader->param_writer.setValue(writer);
    }
    writer->param_out_path.setValue(a_output.value());
  }
  loader->param_verbose.setValue(true);
  loader->run();
  loader->join();
  return 0;
}

int main(int argc, char** argv) {
  srrg2_core::srrgInit(argc, argv);
  kd_slam_registerTypes2D();
  kd_slam_registerTypes3D();

  using namespace std;
  using namespace kd_slam::cuda;
  srrg2_core::ParseCommandLine cmd(argv, banner);
  srrg2_core::ArgumentString a_config(&cmd, "c",  "config",       "pipeline config file",                                "config.json");
  srrg2_core::ArgumentString a_loader(&cmd, "L",  "loader",       "name of loader in config",                            "loader");
  srrg2_core::ArgumentFlag   a_2d    (&cmd, "2",  "two-dim",      "use 2D pipeline");
  srrg2_core::ArgumentString a_input (&cmd, "i",  "input",        "name of input bag or mcap file",                      "");
  srrg2_core::ArgumentString a_output (&cmd, "o",  "output",        "name of output mcap",                      "");
  srrg2_core::ArgumentString a_cloud_topic (&cmd, "tc",  "cloud-topic",        "name of cloud in input stream",                      "");
  srrg2_core::ArgumentString a_tree_topic (&cmd, "tt",  "tree-topic",        "name of tree in input stream",                      "");
  srrg2_core::ArgumentString a_imu_topic (&cmd, "ti",  "imu-topic",        "name of imu in input stream",                      "");
  cmd.parse();
  PropertyContainerManager manager;
  cmd.summary();
  manager.read(a_config.value());
  
  if (a_2d.isSet())
    return run<AppTraits2D>(manager, a_loader.value(), a_input, a_cloud_topic, a_imu_topic, a_tree_topic, a_output);
  else
    return run<AppTraits3D>(manager, a_loader.value(), a_input, a_cloud_topic, a_imu_topic, a_tree_topic, a_output);
}
