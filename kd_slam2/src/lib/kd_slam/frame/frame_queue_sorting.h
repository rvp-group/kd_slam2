#pragma once
#include "frame_queue_base.h"
#include <set>
#include <queue>

namespace kd_slam {
  namespace frame {

    struct FrameQueueSorting:  public FrameQueueBase {
      double sorting_interval=1.0f;
      size_t max_capacity=100;
  
      struct TsPtrCompare {
        inline bool operator()(const std::shared_ptr<FrameBase>& a,
                               const std::shared_ptr<FrameBase>& b) const {
          const auto& ta = (a==nullptr)?0:a->ts;
          const auto& tb = (b==nullptr)?0:b->ts;
          return ta<tb;
        }
      };

      void push(std::shared_ptr<FrameBase> f) override;
  
      std::shared_ptr<FrameBase> pop() override;

      int size() const override;

      void setDone() override;

      std::set<std::shared_ptr<FrameBase>, TsPtrCompare> sorted_queue;

      std::queue<std::shared_ptr<FrameBase>> out_q;


    };
  }
}
