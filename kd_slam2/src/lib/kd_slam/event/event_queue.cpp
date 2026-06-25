#include "event_queue.h"

namespace kd_slam {
  namespace event {

    void EventQueue::pushEvent(EventBasePtr ev)  {
      std::unique_lock<std::mutex> lk(mtx);
      q.push(ev);
      cv.notify_one();
    }
      
    EventBasePtr EventQueue::popEvent()  {
      std::unique_lock<std::mutex> lk(mtx);
      cv.wait(lk,[&]{return ! q.empty() || is_done;});
      EventBasePtr e=nullptr;
      if (! q.empty()) {
        e=q.front();
        q.pop();
      } 
      return e;
    }
    void EventQueue::flush() {
      while (size()) {
        auto e=popEvent();
        EventPublisher::pushEvent(e);
      }
    }
    void EventQueue::setDone()  {
      is_done=true;
      cv.notify_all();
      EventPublisher::setDone();
    }
      
    bool EventQueue::isDone() const  {
      return is_done;
    };

    size_t EventQueue::size() const {
      std::unique_lock<std::mutex> lk(mtx);
      return q.size();
    }
  }
}
