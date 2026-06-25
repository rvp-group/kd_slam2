#pragma once
#include "frame_base.h"
#include <memory>

namespace kd_slam {
  namespace frame {

    struct FrameConsumerBase {
      virtual void push(std::shared_ptr<FrameBase> f) = 0;
      virtual ~FrameConsumerBase() = default;
    };

  } // namespace frame
} // namespace kd_slam
