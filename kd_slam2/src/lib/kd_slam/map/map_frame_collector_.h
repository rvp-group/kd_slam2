#pragma once
#include "event.h"
#include "map_event_.h"
#include <map>
#include <set>
#include <memory>
#include <list>
#include "kd_slam/event/event_dispatcher.h"

namespace kd_slam {
  namespace map {
    using namespace event;
    
    template <typename NodeType_>
    struct MapFrameCollector_ :  public EventDispatcher {
      
      using NodeType     = NodeType_;
      using IsometryType = typename NodeType_::IsometryType;
      using VectorType   = typename NodeType_::VectorType;
      using VelocityVectorType = typename NodeType::Traits::GeometryTraits::VelocityVectorType;
      static constexpr int Dim = NodeType_::Dim;
      using EvAdded = EventFrameAdded_<NodeType_>;
      using EvKF    = EventKeyframeAdded_<NodeType_>;
      using EvOdom  = EventOdometry_<NodeType_>;
      using EvFact  = EventFactorAdded_<NodeType_>;
      using EvPGO   = EventPGOOptimized_<NodeType_>;
      using EvLoop  = EventLoopSearch_<NodeType_>;
      using EvReloc = EventRelocalize_<NodeType_>;
      using EvProc  = EventFrameProcessed_<NodeType_>;
      using EvVel   = EventVelocity_<NodeType_>;
      using EvFacCh = EventFactorChanged;
      using FrameInfoPtr=std::shared_ptr<EvAdded>;
    
      std::map<int, FrameInfoPtr> _frames;
      std::map<int, FrameInfoPtr> _keyframes;
      std::map<int, std::list<std::shared_ptr<EvVel> > > _velocities;
      std::map<size_t, std::shared_ptr<EvFact>> _factors_map;
      std::list<std::shared_ptr<EvFact> > _factors;
      FrameInfoPtr _current;
      MapFrameCollector_();
    };

  } // namespace kd_slam
}
