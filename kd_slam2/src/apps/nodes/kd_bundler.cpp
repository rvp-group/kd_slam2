#include "kd_app_common_.h"
#include "kd_slam/bundler/bundler_proc_.h"
#include "kd_slam/cuda/cuda_common.h"
#include "instances_2d.h"
#include "instances_3d.h"

using namespace srrg2_core;
using namespace kd_slam;
using namespace kd_slam::frame;
using namespace kd_slam::event;
using namespace kd_slam::apps;

const char* banner[] = {
  "kd_bundler: load a saved map and runs optional bundle adjustment",
  "usage: kd_bundler [options]",
  nullptr
};

template <typename N>
using AppTraits_ = AppTraitsBase_<N, kd_slam::slam::Bundler_<N>>;

using AppTraits2D = AppTraits_<d2::NodeType>;
using AppTraits3D = AppTraits_<d3::NodeType>;

template <typename AppTraits>
static int run(PropertyContainerManager& manager,
               const std::string&        proc_name,
               EventSinkType             ev_sink_type,
               ArgumentString& a_input_map,
               ArgumentString& a_output_map,
               ArgumentFlag&   a_bundle,
               ArgumentFlag&   a_ct_bundle) {
  using namespace std;
  using ProcType   = typename AppTraits::ProcType;
  using NodeType   = typename AppTraits::NodeType;

  if (!a_input_map.isSet()) {
    cerr << "input map (-im) required\n";
    return -1;
  }

  auto proc = manager.getByName<ProcType>(proc_name);
  if (!proc) { cerr << "proc '" << proc_name << "' not found or wrong type\n"; return 1; }
  proc->syncParams();

  KDAppRunner_<AppTraits> runner;
  runner.on_map_save = [&]() {
    if (a_output_map.isSet() && !a_output_map.value().empty())
      saveMap(a_output_map.value(), *proc->map());
  };
  runner.setup(proc, ev_sink_type, "");

  if (runner.viewer) {
    runner.viewer->on_bundle         = [&]() { proc->bundle(); };
    runner.viewer->on_ct_bundle      = [&]() { proc->bundleCT(); };
    runner.viewer->on_map_doctor     = [&]() { proc->cure(); };
    runner.viewer->on_map_save       = runner.on_map_save;
    runner.viewer->on_add_pose_noise = [&]() { proc->addPoseNoise(0.1f, 0.05f); };
    runner.viewer->on_add_vel_noise      = [&]() {
        proc->addVelNoise(2.0f, 1.f);
        runner.viewer->on_add_vel_noise = nullptr;
      };
    runner.viewer->on_ct_bundle_vel_only = [&]() { proc->bundleCTVelocityOnly(); };
  }

  auto proc_loop = [&]() {
    auto m = loadMap<NodeType>(a_input_map.value(), proc->isGPU());
    proc->setMap(m);
    if (a_bundle.isSet())    proc->bundle();
    if (a_ct_bundle.isSet()) proc->bundleCT();
  };

  runner.run(proc, nullptr, proc_loop);
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
  ArgumentString a_config    (&cmd, "c",  "config",     "pipeline config file",          "config.json");
  ArgumentString a_proc      (&cmd, "p",  "proc",       "name of processor in config",   "sink");
  ArgumentFlag   a_2d        (&cmd, "2",  "two-dim",    "use 2D pipeline");
  ArgumentInt    a_viewer    (&cmd, "V",  "viewer",     "0:disabled 1:logger 2:GL",      0);
  ArgumentString a_input_map (&cmd, "im", "input-map",  "input map file (.kd)",          "");
  ArgumentString a_output_map(&cmd, "om", "output-map", "output map filename",           "");
  ArgumentFlag   a_bundle    (&cmd, "b",  "bundle",     "run ICP bundle adjustment");
  ArgumentFlag   a_ct_bundle (&cmd, "cb", "ct-bundle",  "run CT-ICP bundle adjustment");
  cmd.parse();

  EventSinkType ev_sink_type = parseViewerArg(a_viewer);
  std::cerr << "Viewer: [" << sink2str[a_viewer.value()] << "]\n";
  PropertyContainerManager manager;
  cmd.summary();
  manager.read(a_config.value());

  if (a_2d.isSet())
    return run<AppTraits2D>(manager,  a_proc.value(), ev_sink_type,
                            a_input_map, a_output_map, a_bundle, a_ct_bundle);
  return run<AppTraits3D>(manager,  a_proc.value(), ev_sink_type,
                          a_input_map, a_output_map, a_bundle, a_ct_bundle);
}
