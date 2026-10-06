#include <thread>
#include "kd_live_running_context.h"
#include "srrg_system_utils/parse_command_line.h"
#include "srrg_system_utils/system_utils.h"
#include "srrg_property/property_container_manager.h"
#include <srrg_config/property_configurable.h>
#include <srrg_solver/solver_core/solver.h>
#include <srrg_solver/solver_core/factor_graph.h>
#include "kd_io/queue_message_reader.h"


using namespace srrg2_core;
using namespace srrg2_solver;
using namespace kd_slam;
using namespace kd_slam::frame;
using namespace kd_slam::event;
using namespace std;


KDLiveRunningContext::KDLiveRunningContext():
  proc_queue(4),
  ev_queue(new kd_slam::event::EventQueue),
  message_queue(new MessageQueueBounded(100))
{}
  
int KDLiveRunningContext::setup(char** argv) {
  ParseCommandLine cmd(argv, banner);
  ArgumentString a_config      (&cmd, "c",    "config",          "pipeline config file",          "config.json");
  ArgumentString a_loader      (&cmd, "L",    "loader",          "name of loader in config",      "loader");
  ArgumentString a_proc        (&cmd, "p",    "proc",            "name of processor in config",   "sink");
  ArgumentFlag   a_gl          (&cmd, "g",    "gl",              "enable GL viewer");
  ArgumentFlag   a_logger      (&cmd, "l",    "logger",          "enable event logger to stderr");
  ArgumentString a_input       (&cmd, "i",    "input",           "input_address for json file",        "192.168.1.160");
  ArgumentInt a_port           (&cmd, "po",    "port",           "input_port, -1 if file",        -1);
  ArgumentString a_output_tum  (&cmd, "ot",   "output-tum",      "output TUM trajectory",         "");
  ArgumentString a_output_state(&cmd, "os",   "output-state",    "output state filename",         "");
  ArgumentString a_output_map  (&cmd, "om",   "output-map",      "output map filename",           "");
  cmd.parse();
  ouster_port=a_port.value();
  ouster_host=a_input.value();
  cmd.summary();
  // parse the args
  PropertyContainerManager manager;
  manager.read(a_config.value());

  if(a_output_map.isSet())
    map_filename=a_output_map.value();
    
  // read the config
  auto loader_ = manager.getByName(a_loader.value());
  if (!loader_) {
    cerr << "loader '" << a_loader.value() << "' not found\n";
    return -1;
  }
  loader = dynamic_pointer_cast<LoaderType>(loader_);
  if (!loader) {
    cerr << "wrong loader type: " << loader_->className() << "\n";
    return -2;
  }

  auto reader_ = manager.getByName("message_reader");
  auto reader = dynamic_pointer_cast<QueueMessageReader>(reader_);
  if (! reader) {
    cerr << "queue not set" << endl;
    return -3;
  }
  
  proc = manager.getByName<ProcType>(a_proc.value());
  if (!proc) {
    cerr << "proc '" << a_proc.value() << "' not found or wrong type\n";
    return -4;
  }

  // sync the configuration
  proc->syncParams();

  // connect the compute
  reader->q=message_queue;
  loader->setFrameQueue(proc_queue);

  //create  event listeners if needed
  if (a_gl.isSet())
    sinks.push_back(make_shared<GLDrawableType>());

  if (a_logger.isSet())
    sinks.push_back(make_shared<LoggerType>());

  shared_ptr<TUMWriterType> tum_writer;
  if (a_output_tum.isSet()) {
    os_tum.open(a_output_tum.value());
    tum_writer = make_shared<TUMWriterType>();
    tum_writer->output_stream = &os_tum;
    sinks.push_back(tum_writer);
  }

  shared_ptr<StateWriterType> state_writer;
  if (a_output_state.isSet()) {
    os_state.open(a_output_state.value());
    state_writer = make_shared<StateWriterType>();
    state_writer->output_stream = &os_state;
    sinks.push_back(state_writer);
  }

  // connect the listeners
  drawable=nullptr;

  ouster_logger= std::make_shared<KDLiveLogger>();
  sinks.push_back(ouster_logger);
  for (auto& s : sinks) {
    ev_queue->event_sinks.push_back(s);
    if (auto d = std::dynamic_pointer_cast<GLDrawableType>(s)) {
      drawable = d;
      viewer = std::make_shared<KDViewer>(*proc, drawable);
    }
  }
  proc->event_sinks.push_back(ev_queue);
  loader->event_sinks.push_back(ev_queue);

  on_map_save = [&]() {
    if (! map_filename.empty()) {
      cerr << "saving map [" << map_filename << "]"<< endl;
      saveMap(map_filename, *proc->map());
    }
  };
  cerr << "setup complete" << endl;
  return 1;
}

// main slam loop
void KDLiveRunningContext::loop(){
  pthread_setname_np(pthread_self(), "runner");
  while (true) {
    auto frame = proc_queue.pop();
    if (! frame) {
      cerr << "compute over" << endl;
      break;
    }
    auto ft = dynamic_pointer_cast<FrameTree>(frame);
    if (!ft || !ft->tree)
      continue;
    proc->push(ft);
  }
  proc->printLoopStats(cerr);
}
  
void KDLiveRunningContext::run(){
  // drain callback for the events
  // if the viewer is up it will be its task
  // otherwise it will be the task of an own thread
  auto drain_cb = [&]() -> bool {
    if (ev_queue->isDone())
      return false;
    ev_queue->flush();
    return true;
  };

  loader->run();
  if (viewer) {
    viewer->loop_callback = drain_cb;
    std::thread proc_thread(&KDLiveRunningContext::loop, this);
    viewer->loop();
    // unlock in case it is paused
    if (! proc->running())
      proc->run(true);
    proc->setDone();
    proc_thread.join();
  } else {
    std::thread drain_thread([&]() {
      while (drain_cb()){
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    });
    loop();
    proc->setDone();
    drain_thread.join();
  }
  if (loader)
    loader->join();
  cerr << "Compute over" << endl;
  sleep(1);
  on_map_save();

}

const char* KDLiveRunningContext::banner[] = {
  "kd_slam2_runner: config-driven SLAM/odometry pipeline",
  "usage: kd_slam2_runner [options]",
  nullptr
};
