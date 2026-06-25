#pragma once
#include <iostream>
#include "kd_slam/map/map_frame_collector_.h"
namespace kd_slam {
  using namespace std;
  using namespace event;
  
  template <typename NodeType_>
  struct KDTumWriter_ : public map::MapFrameCollector_<NodeType_> {
    KDTumWriter_();
    using NodeType     = NodeType_;
    using IsometryType = typename NodeType_::IsometryType;
    using VectorType   = typename NodeType_::VectorType;
    using BaseType     = map::MapFrameCollector_<NodeType_>; 
    using BaseType::_frames;
    using BaseType::_keyframes;
    using BaseType::_factors;
    std::ostream* output_stream=nullptr;
    void setDone() override;
  };

} // namespace kd_slam
