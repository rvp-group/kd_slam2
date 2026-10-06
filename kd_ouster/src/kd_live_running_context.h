#pragma once
#include <list>
#include <fstream>
#include "kd_live_logger.h"
#include "srrg_property/property_container_manager.h"
#include "kd_slam/frame/frame_queue_bounded.h"
#include "kd_slam/slam/slam_proc_.h"
#include "kd_io/tree_loader_.h"
#include "kd_slam/event/event_logger.h"
#include "kd_slam/d3/typedefs.h"
#include "kd_slam2/kd_io/kd_state_writer_.h"
#include "kd_slam2/kd_io/message_queue.h"
#include "kd_io/kd_tum_writer_.h"
#include "kd_io/kd_map_io_.h"
#include "instances_3d.h"
#include "kd_viewer/kd_viewer.h"
#include "kd_viewer/drawable_kd_slam_.h"
#include "kd_slam/event/event_queue.h"

struct KDLiveRunningContext {
  using NodeType       = kd_slam::d3::NodeType;
  using ProcType       = kd_slam::slam::SLAM_<NodeType>;
  using FrameTree      = kd_slam::frame::FrameTree_<NodeType>;
  using TUMWriterType  = kd_slam::KDTumWriter_<NodeType>;
  using LoggerType     = kd_slam::event::EventLogger;
  using GLDrawableType = kd_slam::slam::DrawableKDSlam_<NodeType>;
  using LoaderType     = kd_slam::TreeLoader_<NodeType>;
  using StateWriterType = kd_slam::KDStateWriter_<NodeType>;

  std::shared_ptr<KDLiveLogger> ouster_logger;
  std::shared_ptr<ProcType> proc;
  std::shared_ptr<LoaderType> loader;
  kd_slam::FrameQueueBounded proc_queue;
  std::list<kd_slam::event::EventSinkPtr> sinks;
  std::shared_ptr<kd_slam::event::EventQueue> ev_queue;
  std::shared_ptr<MessageQueueBounded> message_queue;
  std::shared_ptr<GLDrawableType>             drawable;
  std::shared_ptr<kd_slam::KDViewer>                   viewer;
  std::function<void()> on_map_save = [](){}; // do nothing

  std::ofstream os_tum;
  std::ofstream os_state;

  std::string map_filename="";

  // bit dirty but we read the params here
  std::string ouster_host;
  int ouster_port;
  
  KDLiveRunningContext();
  
  static const char* banner[];
  
  int setup(char** argv) ;
  // main slam loop
  void loop();  
  void run();
};

