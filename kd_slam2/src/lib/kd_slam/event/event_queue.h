#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>
#include "event.h"

namespace kd_slam {
  namespace event {

    // std::queue of events
    struct EventQueue: public EventPublisher, protected EventSource{
      void pushEvent(EventBasePtr ev) override; // pushes it to the queue
      void flush();
      void setDone() override ;      
      bool isDone() const override;
      size_t size() const;
    protected:
      std::queue<EventBasePtr> q;
      mutable std::mutex mtx;
      mutable std::condition_variable cv;
      bool is_done=false;
      EventBasePtr popEvent() override; // called internally from flush 
    };
  }
}
