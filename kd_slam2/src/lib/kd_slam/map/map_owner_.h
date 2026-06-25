#pragma once
#include "srrg_config/configurable.h"
#include "map_.h"
#include "kd_slam/event/event.h"
#include "map_event_.h"

namespace kd_slam {
  namespace map {

    template <typename Node_>
    struct MapOwner_ :
      public event::EventPublisher,
      public srrg2_core::Configurable
    {
      using MapType           = Map_<Node_>;
      using NodeType            = typename MapType::NodeType;
      using Scalar              = typename MapType::Scalar;
      using TreeBaseType        = typename MapType::TreeBaseType;
      using AlignerBase         = typename MapType::AlignerBase;
      using ICPStats            = typename MapType::ICPStats;
      using IsometryType        = typename MapType::IsometryType;
      using PoseHessianType     = typename MapType::PoseHessianType;
      using FrameTree           = typename MapType::FrameTree;
      using FrameTreePtr        = typename MapType::FrameTreePtr;
      using VelocityVectorType  = typename MapType::VelocityVectorType;
      using DescriptorExtractor = typename MapType::DescriptorExtractor;
      using DescriptorType      = typename MapType::DescriptorType;
      using Frame               = typename MapType::Frame;
      using FramePtr            = typename MapType::FramePtr;
      using SolverVariableType            = typename MapType::SolverVariableType;
      using SolverPGOFactorType           = typename MapType::SolverPGOFactorType;
      using SolverVelocityVariableType    = typename MapType::SolverVelocityVariableType;
      using SolverVelocityPriorFactorType = typename MapType::SolverVelocityPriorFactorType;
      using SolverGravityPriorFactorType  = typename MapType::SolverGravityPriorFactorType;
      using SolverFactorBridge            = typename MapType::SolverFactorBridge;
      using SolverFactorBridgePtr         = typename MapType::SolverFactorBridgePtr;
      using PGOFactor                     = typename MapType::PGOFactor;
      using PGOFactorPtr                  = typename MapType::PGOFactorPtr;
      using MultiViewICPFactor            = typename MapType::MultiViewICPFactor;
      using MultiViewICPFactorPtr         = typename MapType::MultiViewICPFactorPtr;
      using MultiViewCTICPFactor          = typename MapType::MultiViewCTICPFactor;
      using MultiViewCTICPFactorPtr       = typename MapType::MultiViewCTICPFactorPtr;

      using EventFrameAdded     = EventFrameAdded_<Node_>;
      using EventKeyframeAdded  = EventKeyframeAdded_<Node_>;
      using EventFrameProcessed = EventFrameProcessed_<Node_>;
      using EventFactorAdded    = EventFactorAdded_<Node_>;
      using EventPGOOptimized   = EventPGOOptimized_<Node_>;
      using EventLoopSearch     = EventLoopSearch_<Node_>;
      using EventRelocalize     = EventRelocalize_<Node_>;
      using EventOdometry       = EventOdometry_<Node_>;
      using EventVelocity       = EventVelocity_<Node_>;

      virtual void setMap(std::shared_ptr<MapType> m) { _map=m; }
      std::shared_ptr<MapType> map() {return _map;}
      KDStatus status()      const { return _status; }
      virtual double currentStamp() const {return 0;}
      virtual void reset() { if (_map) _map->clear();}
      virtual void syncParams(){}
    protected:
      MapOwner_() : _map(std::make_shared<MapType>()) {}
      void saveFrames();
      void restoreFrames();
      void pushFrameEvents();
      void pushFactorUpdateEvents();
      void pushFactorEvents();
      void pushOptimizeEvent(double ts=0, const std::string& msg="");
      void pushFrameProcessedEvent(double ts, KDStatus _status);

      // applies velocities to the current keyframes.
      //call the saveFrames first if you want to preserve the original estimate.
      void applyKFVelocities(); 
      bool         _param_changed = false;
      
      std::shared_ptr<MapType> _map;
      inline std::shared_ptr<srrg2_solver::FactorGraph>& factorGraph() {return _map->_factor_graph;}
      KDStatus     _status        = Init;
    };

  } // namespace slam
} // namespace kd_slam
