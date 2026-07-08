#include "kd_app_common_.h"
#include "kd_slam/slam/slam_proc_.h"
#include "kd_slam/cuda/cuda_common.h"
#include "kd_io/kd_state_writer_.h"
#include "instances_2d.h"
#include "instances_3d.h"

using namespace srrg2_core;
using namespace kd_slam;
using namespace kd_slam::frame;
using namespace kd_slam::event;
using namespace kd_slam::apps;

const char* banner[] = {
  "kd_slam2_runner: config-driven SLAM/odometry pipeline",
  "usage: kd_slam2_runner [options]",
  nullptr
};

template <typename N>
struct AppTraits_ : public AppTraitsBase_<N, kd_slam::slam::SLAM_<N>> {
  using StateWriterType = kd_slam::KDStateWriter_<N>;
};

using AppTraits2D = AppTraits_<d2::NodeType>;
using AppTraits3D = AppTraits_<d3::NodeType>;

template <typename AppTraits>
static int run(PropertyContainerManager& manager,
               const std::string&        loader_name,
               const std::string&        proc_name,
               EventSinkType             ev_sink_type,
               ArgumentString& a_input,
               ArgumentString& a_output_tum,
               ArgumentString& a_output_state,
               ArgumentString& a_output_map) {
  using namespace std;
  using ProcType        = typename AppTraits::ProcType;
  using LoaderType      = typename AppTraits::LoaderType;
  using FrameTree       = typename AppTraits::FrameTree;
  using StateWriterType = typename AppTraits::StateWriterType;

  auto loader_ = manager.getByName(loader_name);
  if (!loader_) {
    cerr << "loader '" << loader_name << "' not found\n";
    return 1;
  }
  auto loader = dynamic_pointer_cast<LoaderType>(loader_);
  if (!loader) {
    cerr << "wrong loader type: " << loader_->className() << "\n";
    return 1;
  }

  if (a_input.isSet())
    if (auto reader = loader->param_reader.value())
      reader->param_bag_path.setValue(a_input.value());

  auto proc = manager.getByName<ProcType>(proc_name);
  if (!proc) {
    cerr << "proc '" << proc_name << "' not found or wrong type\n";
    return 1;
  }
  proc->syncParams();

  FrameQueueBounded queue(4);
  loader->setFrameQueue(queue);
  loader->run();

  KDAppRunner_<AppTraits> runner;
  runner.on_map_save = [&]() {
    if (a_output_map.isSet() && !a_output_map.value().empty())
      saveMap(a_output_map.value(), *proc->map());
  };
  runner.setup(proc, ev_sink_type, a_output_tum.isSet() ? a_output_tum.value() : "");

  if (runner.viewer) {
    //runner.viewer->on_bundle    = [&]() { proc->bundle(); };
    //runner.viewer->on_ct_bundle = [&]() { proc->bundleCT(); };
    runner.viewer->on_map_save  = runner.on_map_save;
  }
  
  // state writer is runner-specific; wire it after setup
  shared_ptr<StateWriterType> state_writer;
  ofstream os_state;
  if (a_output_state.isSet()) {
    os_state.open(a_output_state.value());
    state_writer = make_shared<StateWriterType>();
    state_writer->output_stream = &os_state;
    if (runner.ev_queue)
      runner.ev_queue->event_sinks.push_back(state_writer);
    else
      proc->event_sinks.push_back(state_writer);
  }

  bool first_frame=true;
  auto proc_loop = [&]() {
    while (true) {
      auto frame = queue.pop();
      if (!frame) {
        proc->push(frame);
        break;
      }
      if (first_frame) {
        if (runner.viewer)
          proc->run(false);
        else
          proc->run(true);
        first_frame=false;
      }
      auto ft = dynamic_pointer_cast<FrameTree>(frame);
      if (!ft || !ft->tree)
        continue;
      proc->push(ft);
    }
    proc->printLoopStats(cerr);
  };


  runner.run(proc, loader, proc_loop);
  return 0;
}

int main(int argc, char** argv) {
  srrgInit(argc, argv);
  kd_slam_registerTypes2D();
  kd_slam_registerTypes3D();
#ifdef HAVE_CUDA
  using namespace kd_slam::cuda;
  CUDA_CHECK(cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync));
#endif

  ParseCommandLine cmd(argv, banner);
  ArgumentString a_config      (&cmd, "c",    "config",          "pipeline config file",          "config.json");
  ArgumentString a_loader      (&cmd, "L",    "loader",          "name of loader in config",      "loader");
  ArgumentString a_proc        (&cmd, "p",    "proc",            "name of processor in config",   "sink");
  ArgumentFlag   a_2d          (&cmd, "2",    "two-dim",         "use 2D pipeline");
  ArgumentInt    a_viewer      (&cmd, "V",    "viewer",          "0:disabled 1:logger 2:GL",      0);
  ArgumentString a_input       (&cmd, "i",    "input",           "input bag or mcap file",        "");
  ArgumentString a_output_tum  (&cmd, "ot",   "output-tum",      "output TUM trajectory",         "");
  ArgumentString a_output_state(&cmd, "os",   "output-state",    "output state filename",         "");
  ArgumentString a_output_map  (&cmd, "om",   "output-map",      "output map filename",           "");
  cmd.parse();

  EventSinkType ev_sink_type = parseViewerArg(a_viewer);
  std::cerr << "Viewer: [" << sink2str[a_viewer.value()] << "]\n";
  PropertyContainerManager manager;
  cmd.summary();
  manager.read(a_config.value());

  if (a_2d.isSet())
    return run<AppTraits2D>(manager, a_loader.value(), a_proc.value(), ev_sink_type,
                            a_input, a_output_tum, a_output_state, a_output_map);
  return run<AppTraits3D>(manager, a_loader.value(), a_proc.value(), ev_sink_type,
                          a_input, a_output_tum, a_output_state, a_output_map);
}
