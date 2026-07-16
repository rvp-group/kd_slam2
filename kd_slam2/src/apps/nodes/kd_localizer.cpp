#include "kd_app_common_.h"
#include "kd_slam/localizer/localizer_proc_.h"
#include "kd_io/kd_state_writer_.h"
#include "instances_2d.h"
#include "instances_3d.h"

using namespace srrg2_core;
using namespace kd_slam;
using namespace kd_slam::frame;
using namespace kd_slam::event;
using namespace kd_slam::apps;

const char* banner[] = {
  "kd_localizer: config-driven localizer pipeline",
  "usage: kd_localizer [options]",
  nullptr
};

template <typename N>
struct AppTraits_ : public AppTraitsBase_<N, kd_slam::slam::Localizer_<N>> {
  using StateWriterType = kd_slam::KDStateWriter_<N>;
};

using AppTraits2D = AppTraits_<d2::NodeType>;
using AppTraits3D = AppTraits_<d3::NodeType>;

template <typename AppTraits>
static int run(PropertyContainerManager& manager,
               const std::string&        loader_name,
               const std::string&        proc_name,
               EventSinkType             ev_sink_type,
               ArgumentString& a_input_bag,
               ArgumentString& a_input_map,
               ArgumentString& a_output_tum,
               ArgumentString& a_output_state) {
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

  if (a_input_bag.isSet())
    if (auto reader = loader->param_reader.value())
      reader->param_bag_path.setValue(a_input_bag.value());

  auto proc = manager.getByName<ProcType>(proc_name);
  if (!proc) {
    cerr << "proc '" << proc_name << "' not found or wrong type\n";
    return 1;
  }

  if (! a_input_map.isSet()) {
    cerr << "I need a map to run localize" << endl;
    return 1;
  }

  proc->syncParams();


  FrameQueueBounded queue(4);
  loader->setFrameQueue(queue);
  loader->run();
  cerr << "loader started on bag " << a_input_bag.value() << endl;
  KDAppRunner_<AppTraits> runner;
  runner.setup(proc, ev_sink_type, a_output_tum.isSet() ? a_output_tum.value() : "");

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

    bool first_frame = true;
    auto proc_loop = [&]() {
    auto m = loadMap<typename AppTraits::NodeType>(a_input_map.value(), proc->isGPU());
    proc->setMap(m);
    proc->run(true);
    while (true) {
      auto frame = queue.pop();
      if (!frame) {
        proc->push(frame);
        break;
      }
      if (first_frame) {
        proc->run(false);
        first_frame=false;
      }
      auto ft = dynamic_pointer_cast<FrameTree>(frame);
      if (!ft || !ft->tree)
        continue;
      proc->push(ft);
    }
  };

  runner.run(proc, loader, proc_loop);
  return 0;
}

int main(int argc, char** argv) {
  setupMain(argc, argv);

  ParseCommandLine cmd(argv, banner);
  ArgumentString a_config      (&cmd, "c",    "config",          "pipeline config file",          "config.json");
  ArgumentString a_loader      (&cmd, "L",    "loader",          "name of loader in config",      "loader");
  ArgumentString a_proc        (&cmd, "p",    "proc",            "name of processor in config",   "sink");
  ArgumentFlag   a_2d          (&cmd, "2",    "two-dim",         "use 2D pipeline");
  ArgumentInt    a_viewer      (&cmd, "V",    "viewer",          "0:disabled 1:logger 2:GL",      0);
  ArgumentString a_input_bag       (&cmd, "i",    "input",           "input bag or mcap file",        "");
  ArgumentString a_input_map       (&cmd, "im",    "input-map",           "input map",        "");
  ArgumentString a_output_tum  (&cmd, "ot",   "output-tum",      "output TUM trajectory",         "");
  ArgumentString a_output_state(&cmd, "os",   "output-state",    "output state filename",         "");
  cmd.parse();

  EventSinkType ev_sink_type = parseViewerArg(a_viewer);
  std::cerr << "Viewer: [" << sink2str[a_viewer.value()] << "]\n";
  PropertyContainerManager manager;
  cmd.summary();
  manager.read(a_config.value());

  if (a_2d.isSet())
    return run<AppTraits2D>(manager, a_loader.value(), a_proc.value(), ev_sink_type,
                            a_input_bag, a_input_map, a_output_tum, a_output_state);
  return run<AppTraits3D>(manager, a_loader.value(), a_proc.value(), ev_sink_type,
                            a_input_bag, a_input_map, a_output_tum, a_output_state);
}
