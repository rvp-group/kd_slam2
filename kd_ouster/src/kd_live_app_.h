#pragma once
#include <functional>
#include <fstream>
#include <thread>
#include <dlfcn.h>

#include "srrg_system_utils/parse_command_line.h"
#include "srrg_system_utils/system_utils.h"
#include "srrg_property/property_container_manager.h"
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
        ev_queue = std::make_shared<kd_slam::event::EventQueue>();
        drawable=nullptr;

        for (auto& s : sinks) {
          ev_queue->event_sinks.push_back(s);
          if (auto d = std::dynamic_pointer_cast<GLDrawableType>(s)) {
            drawable = d;
            viewer = std::make_shared<KDViewer>(*proc, drawable);
          }
        }
        proc->event_sinks.push_back(ev_queue);
        loader->event_sinks.push_back(ev_queue);
      }

      // Provide a custom one to inject per-frame logic (e.g. proc->run(false)).
      void run(std::shared_ptr<ProcType>   proc,
               std::shared_ptr<LoaderType> loader,
               std::function< void() >  proc_loop) {
      
        auto loop_cb = [&]() -> bool {
          if (ev_queue->isDone()) return false;
          ev_queue->flush();
          return true;
        };

        if (viewer) {
          viewer->on_map_save = on_map_save;
          viewer->loop_callback = loop_cb;
          std::thread proc_thread(proc_loop);
          viewer->loop();
          // unlock in case it is paused
          if (! proc->running())
            proc->run(true);
          proc->setDone();
          proc_thread.join();
        } else {
          std::thread drain_thread([&]() {
            while (loop_cb()){
              std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
          });
          proc_loop();
          proc->setDone();
          drain_thread.join();
        }
        if (loader)
          loader->join();
        cerr << "Compute over" << endl;
        on_map_save();
      }
    };
  } // namespace apps


} // namespace kd_slam
