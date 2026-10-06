#pragma once
#include <functional>
#include <fstream>
#include <thread>
#include <dlfcn.h>

#include "srrg_system_utils/parse_command_line.h"
#include "srrg_system_utils/system_utils.h"
#include "srrg_property/property_container_manager.h"
#include "srrg_config/configurable_manager.h"
#include "kd_slam/frame/frame_queue_bounded.h"
#include "kd_io/tree_loader_.h"
#include "kd_slam/d2/typedefs.h"
#include "kd_slam/d3/typedefs.h"
#include "kd_slam/event/event.h"
#include "kd_slam/event/event_queue.h"
#include "kd_viewer/kd_viewer.h"
#include "kd_viewer/drawable_kd_slam_.h"
#include "kd_slam/event/event_logger.h"
#include "kd_io/kd_tum_writer_.h"
#include "kd_io/kd_map_io_.h"

namespace kd_slam {
  namespace apps {

    // Base AppTraits: common types, parameterised on NodeType and ProcType.
    // Each app defines its own AppTraits_ by aliasing or extending this.
    template <typename NodeType_, typename ProcType_>
    struct AppTraitsBase_ {
      using NodeType       = NodeType_;
      using ProcType       = ProcType_;
      using FrameTree      = kd_slam::frame::FrameTree_<NodeType>;
      using TUMWriterType  = kd_slam::KDTumWriter_<NodeType>;
      using LoggerType     = kd_slam::event::EventLogger;
      using GLDrawableType = kd_slam::slam::DrawableKDSlam_<NodeType>;
      using LoaderType     = kd_slam::TreeLoader_<NodeType>;
    };

    // Shared run machinery: event sink wiring, viewer loop dispatch.
    //
    // Usage pattern:
    //   KDAppRunner_<AppTraits> runner;
    //   runner.on_map_save = [&](){ ... };
    //   std::list<event::EventSinkPtr> sinks = { ... };  // additive, caller's choice
    //   runner.setup(proc, loader, sinks);
    //   if (runner.viewer) {
    //     runner.viewer->on_bundle    = [&](){ proc->bundle(); };
    //     runner.viewer->on_ct_bundle = [&](){ proc->bundleCT(); };
    //     runner.viewer->on_map_save  = runner.on_map_save;
    //   }
    //   runner.run(proc, loader, proc_loop);          // default loop callback
    //   runner.run(proc, loader, proc_loop, my_cb);   // custom loop callback
    //
    // proc_loop must contain the application work only; setDone/join are
    // handled by run().  loader may be nullptr for batch (no bag) modes.
    //
    // Threading is implied by the sink list, not chosen separately: if a
    // GLDrawableType is among the sinks, proc/loader push into an EventQueue
    // and the GL viewer drains it on the main thread (it owns the GL
    // context); otherwise every sink is wired straight into proc/loader,
    // since there is no cross-thread handoff to do.
    template <typename AppTraits>
    struct KDAppRunner_ {
      using ProcType       = typename AppTraits::ProcType;
      using LoaderType     = typename AppTraits::LoaderType;
      using GLDrawableType = typename AppTraits::GLDrawableType;

      std::shared_ptr<kd_slam::event::EventQueue> ev_queue;
      std::shared_ptr<GLDrawableType>             drawable;
      std::shared_ptr<KDViewer>                   viewer;

      std::function<void()> on_map_save = [](){};

      void setup(std::shared_ptr<ProcType>              proc,
                 std::shared_ptr<LoaderType>             loader,
                 const std::list<kd_slam::event::EventSinkPtr>& sinks) {
        drawable = nullptr;
        for (auto& s : sinks)
          if (auto d = std::dynamic_pointer_cast<GLDrawableType>(s))
            drawable = d;

        if (drawable) {
          ev_queue = std::make_shared<kd_slam::event::EventQueue>();
          for (auto& s : sinks)
            ev_queue->event_sinks.push_back(s);
          
          proc->event_sinks.push_back(ev_queue);

          if (loader)
            loader->event_sinks.push_back(ev_queue);
          viewer = std::make_shared<KDViewer>(*proc, drawable);
          
        } else {
          for (auto& s : sinks) {
            proc->event_sinks.push_back(s);
            if (loader) loader->event_sinks.push_back(s);
          }
        }
      }

      // loop_callback: nullptr uses the default ev_queue flush.
      // Provide a custom one to inject per-frame logic (e.g. proc->run(false)).
      void run(std::shared_ptr<ProcType>   proc,
               std::shared_ptr<LoaderType> loader,
               std::function<void()>       proc_loop,
               std::function<bool()>       loop_callback = nullptr) {
        if (viewer) {
          viewer->on_map_save = on_map_save;
        }
        if (drawable && viewer) {
          if (!loop_callback)
            loop_callback = [this]() -> bool {
              if (ev_queue->isDone()) return false;
              ev_queue->flush();
              return true;
            };
          viewer->loop_callback = loop_callback;
          std::thread proc_thread(proc_loop);
          viewer->loop();
          proc->setDone();
          proc_thread.join();
        } else {
          proc_loop();
          proc->setDone();
        }
        if (loader)
          loader->join();
        cerr << "Compute over" << endl;
        on_map_save();
      }
    };

  } // namespace apps

  void setupMain(int argc, char**argv) {
    srrgInit(argc, argv);
    std::string dl_path=crawlForFile("dl.conf");
    std::ifstream is(dl_path);
    if (is.good()) {
      ConfigurableManager::initFactory(dl_path);
    }
    
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

} // namespace kd_slam
