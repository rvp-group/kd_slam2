#pragma once
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <list>
#include "kd_slam/map/map_event_.h"
#include "kd_slam/map/map_common.h"
#include "kd_slam/icp/icp_stats_.h"
#include "kd_io/tree_loader_.h"

namespace kd_slam {
  using namespace event;
  using namespace map;

  template <typename NodeType_>
  struct KDStateReader_  {

    using NodeType           = NodeType_;
    using Scalar             = typename NodeType_::Scalar;
    using IsometryType       = typename NodeType_::IsometryType;
    using VectorType         = typename NodeType_::VectorType;
    using VelocityVectorType = typename NodeType_::Traits::GeometryTraits::VelocityVectorType;
    using PoseHessianType    = typename NodeType_::PoseHessianType;
    using ICPStats           = icp::ICPStats_<Scalar>;
    static constexpr int Dim  = NodeType_::Dim;
    static constexpr int HDim = PoseHessianType::RowsAtCompileTime;
    using TreeLoaderType      = TreeLoader_<NodeType>;
    
    bool open(const std::string& filename);
    std::shared_ptr<TreeLoaderType> _loader;
  protected:
    struct FrameRecord {
      int count = 0, kf_ref = -1;
      double ts = 0;
      IsometryType pose_in_kf = IsometryType::Identity();
      FrameAddedReason reason;
      std::string topic;
      uint64_t bag_offset = 0;
      uint64_t stamp_ns = 0;
    };

    struct KeyframeRecord: public FrameRecord {
      int kf_idx = -1;
    };

    struct VelocityRecord {
      double ts;
      int count = -1;
      VelocityVectorType velocity;
    };


    struct FactorRecord {
      int from_ref = -1, to_ref = -1;
      KDFactorType type = Odometry;
      IsometryType Z = IsometryType::Identity();
      PoseHessianType omega = PoseHessianType::Zero();
      size_t key;
    };

    std::map<int, KeyframeRecord> _keyframe_records; // keyed by kf_idx
    std::map<int, FrameRecord>    _frame_records;    // keyed by count
    std::list<FactorRecord>       _factor_records;
    std::list<VelocityRecord>          _velocity_records;
    static IsometryType       _readIsometry(std::istringstream& ss);
    static VelocityVectorType _readVelocity(std::istringstream& ss);
  };

  template <typename NodeType_>
  struct KDStateReplay_:
    public KDStateReader_<NodeType_>,
    public event::EventPublisher
  {
    using NodeType           = NodeType_;
    using Scalar             = typename NodeType_::Scalar;
    using IsometryType       = typename NodeType_::IsometryType;
    using VectorType         = typename NodeType_::VectorType;
    using VelocityVectorType = typename NodeType_::Traits::GeometryTraits::VelocityVectorType;
    using PoseHessianType    = typename NodeType_::PoseHessianType;
    using ICPStats           = icp::ICPStats_<Scalar>;
    static constexpr int Dim  = NodeType_::Dim;
    static constexpr int HDim = PoseHessianType::RowsAtCompileTime;

    using ReaderType = KDStateReader_<NodeType_>;
    using ReaderType::_keyframe_records;
    using ReaderType::_frame_records;
    using ReaderType::_factor_records;
    using ReaderType::_velocity_records;
    using ReaderType::_loader;

    using EvAdded = EventFrameAdded_<NodeType_>;
    using EvKF    = EventKeyframeAdded_<NodeType_>;
    using EvOdom  = EventOdometry_<NodeType_>;
    using EvFact  = EventFactorAdded_<NodeType_>;
    using EvVel   = EventVelocity_<NodeType_>;
    using EvProc  = EventFrameProcessed_<NodeType_>;
    using TreeLoaderType      = TreeLoader_<NodeType>;

    virtual void replay();
  };
  
} // namespace kd_slam
