#pragma once
#include <iostream>
#include <string>
#include "kd_slam/event/event_dispatcher.h"
#include "kd_slam/map/map_event_.h"
#include "kd_slam/d3/typedefs.h"
#include "json_item.h"


struct OusterEvent: public kd_slam::event::EventBase {
  size_t num_imus;
  size_t num_clouds;
  size_t dropped_imus;
  size_t dropped_clouds;
  size_t q_capacity;
  size_t q_size;
  void print(std::ostream& os) const override;
};

struct KDLiveLogger: public  kd_slam::event::EventDispatcher {
  using EvOuster=OusterEvent;
  using EvTree=kd_slam::map::EventLoadTree_<kd_slam::d3::NodeType>;
  using EvProc=kd_slam::map::EventFrameProcessed_<kd_slam::d3::NodeType>;
  EvOuster last_ouster;
  EvProc   last_proc;
  EvTree   last_tree;
  KDLiveLogger();
  std::string statsJson();
  
};
