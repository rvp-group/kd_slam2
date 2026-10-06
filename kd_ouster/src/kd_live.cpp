#include <signal.h>
#include <atomic>
#include "kd_live_logger.h"
#include "kd_live_ouster.h"
#include "kd_live_running_context.h"
#include "kd_live_web_server.h"
#include "dlfcn.h"

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace kd_slam;
using namespace kd_slam::frame;
using namespace kd_slam::event;
using namespace std;


volatile bool run=true;
// Whole-process shutdown, SIGINT
void handle_sigint(int sig) {
    run=0;
}

void setupMain(int argc, char**argv) {
  srrgInit(argc, argv);
  bool load_cuda_init=dlopen("libkd_slam_cuda_init.so",RTLD_GLOBAL | RTLD_NOW);
  bool load_2d_icp=dlopen("libkd_slam_icp_2d_cuda.so",RTLD_GLOBAL | RTLD_NOW);
  bool load_2d_slam=dlopen("libkd_slam_slam_2d_cuda.so",RTLD_GLOBAL | RTLD_NOW);
  bool load_3d_icp=dlopen("libkd_slam_icp_3d_cuda.so",RTLD_GLOBAL | RTLD_NOW);
  bool load_3d_slam=dlopen("libkd_slam_slam_3d_cuda.so",RTLD_GLOBAL | RTLD_NOW);
  cerr << "loading cuda layer: " << endl;
  cerr << " CUDA_INIT: " << load_cuda_init << endl;
  cerr << " ICP_2D:    " << load_2d_icp << endl;
  cerr << " SLAM_2D:   " << load_2d_slam << endl; 
  cerr << " ICP_3D:    " << load_3d_icp << endl;
  cerr << " SLAM_3D:   " << load_3d_slam << endl;
}

int main(int argc, char** argv) {
  setupMain(argc, argv);
  signal(SIGINT, handle_sigint);

  KDLiveRunningContext slam_context;
  int setup_result = slam_context.setup(argv);
  if (setup_result!=1) {
    cerr << "setup error" << endl;
    return -1;
  }
  // this starts the ouster;
  KDLiveOuster ouster_reader(slam_context.ouster_host,
               slam_context.ouster_port,
               slam_context.message_queue,
               slam_context.ev_queue,
               run);
  
  KDWebServer web_server(slam_context, 8080);
  ouster_reader.run();
  web_server.run();
  slam_context.run();
  ouster_reader.join();
  web_server.join();
  return 0;
}

