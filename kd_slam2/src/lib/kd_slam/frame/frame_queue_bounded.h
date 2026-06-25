#pragma once
#include <queue>
#include "frame_queue_base.h"

namespace kd_slam {
  namespace frame {

    struct FrameQueueBounded:  public FrameQueueBase {
      FrameQueueBounded(int capacity_=2):capacity(capacity_)  {}
      
      void push(std::shared_ptr<FrameBase> f) override;

      std::shared_ptr<FrameBase> pop() override;

      int size() const override;

      void setDone() override;

      std::queue<std::shared_ptr<FrameBase>> q;
  
      size_t capacity;

    };

  }
}
