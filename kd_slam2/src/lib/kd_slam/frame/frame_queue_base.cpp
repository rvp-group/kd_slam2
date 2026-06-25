#include "frame_queue_bounded.h"
namespace kd_slam {
  namespace frame {

    void FrameQueueBounded::push(std::shared_ptr<FrameBase> f) {
      std::unique_lock<std::mutex> lk(mtx);
      cv.wait(lk, [&]{ return q.size() < capacity; });
      q.push(std::move(f));
      lk.unlock();
      cv.notify_one();
    }

    std::shared_ptr<FrameBase> FrameQueueBounded::pop()  {
      std::unique_lock<std::mutex> lk(mtx);
      cv.wait(lk, [&]{ return !q.empty() || done; });
      if (q.empty()) return nullptr;
      auto f = std::move(q.front());
      q.pop();
      lk.unlock();
      cv.notify_one();
      return f;
    }

    int FrameQueueBounded::size() const  {return q.size();}

    void FrameQueueBounded::setDone() {
      std::lock_guard<std::mutex> lk(mtx);
      done = true;
      cv.notify_all();
    }

  }
}
