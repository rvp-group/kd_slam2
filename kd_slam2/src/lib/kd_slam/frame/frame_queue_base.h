#pragma once
#include <mutex>
#include <condition_variable>
#include "frame_base.h"

namespace kd_slam {
  namespace frame {

    // multithreaded frame queue of fixed capacity
    struct FrameQueueBase {

      // pushes one element in the queue, or stops the caller
      // if the queue exceeds the capacity
      virtual void push(std::shared_ptr<FrameBase> f) = 0;
  
      // pops one element in the queue, or stops the caller
      // if queue is empty
      virtual std::shared_ptr<FrameBase> pop() = 0;

      // typed return value
      // it scans the queue until an element of FrameType is found,
      // skipping all others
      template <typename FrameType>
      std::shared_ptr<FrameType> popT();

      // size accessor
      virtual int size() const = 0;

      // flushes
      virtual void setDone() = 0;

      std::mutex mtx;
      std::condition_variable cv;
      bool done = false;

    };


    template <typename FrameType>
    std::shared_ptr<FrameType> FrameQueueBase::popT() {
      std::shared_ptr<FrameType> p_out=nullptr;
      do  {
        auto p=pop();
        if (p==nullptr)
          return nullptr;
        p_out=std::dynamic_pointer_cast<FrameType>(p);
      } while (! p_out);
      return p_out;
    }

  }
}
