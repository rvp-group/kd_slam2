#pragma once
#include <iostream>
#include "kd_slam/map/map_frame_collector_.h"
namespace kd_slam {
  using namespace std;
  using namespace event;
  
  template <typename NodeType_>
  struct KDStateWriter_ : public map::MapFrameCollector_<NodeType_> {
    KDStateWriter_();
    using NodeType     = NodeType_;
    using IsometryType = typename NodeType_::IsometryType;
    using VectorType   = typename NodeType_::VectorType;
    using BaseType     = map::MapFrameCollector_<NodeType_>;
    using BaseType::Dim;
    using BaseType::_frames;
    using BaseType::_keyframes;
    using BaseType::_velocities;
    using BaseType::_factors;
    std::ostream* output_stream=nullptr;
    void setDone() override;
  };

} // namespace kd_slam
