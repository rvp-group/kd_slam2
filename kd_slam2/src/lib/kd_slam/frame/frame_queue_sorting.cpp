#include "frame_queue_sorting.h"

namespace kd_slam {
  namespace frame {

    void FrameQueueSorting::push(std::shared_ptr<FrameBase> f) {
      std::unique_lock<std::mutex> lk(mtx);
      cv.wait(lk, [&]{ return sorted_queue.size() < max_capacity || done; });
      sorted_queue.insert(f);
      //queue too small to compute a range
      if (sorted_queue.size()<2)
        return;
      double most_recent_ts=(*sorted_queue.rbegin())->ts;
      // extract all elements with ts>most_recent_ts-sorting_interval;
      double out_t=most_recent_ts-sorting_interval;
      while (! sorted_queue.empty() && (*sorted_queue.begin())->ts<out_t) {
        out_q.push(*sorted_queue.begin());
        sorted_queue.erase(sorted_queue.begin());
      }
      size_t q_size=out_q.size();
      for (size_t i=0; i<q_size; ++i)
        cv.notify_one();
      lk.unlock();
    }
  
    std::shared_ptr<FrameBase> FrameQueueSorting::pop() {
      std::unique_lock<std::mutex> lk(mtx);
      cv.wait(lk, [&]{ return !out_q.empty() || done; });
      if (out_q.empty()) return nullptr;
      auto f = std::move(out_q.front());
      out_q.pop();
      cv.notify_one();
      return f;
    }

    int FrameQueueSorting::size() const  {
      return out_q.size()+sorted_queue.size();
      // complete
    }

    void FrameQueueSorting::setDone() {
      std::lock_guard<std::mutex> lk(mtx);
      done = true;
      while (! sorted_queue.empty()) {
        out_q.push(*sorted_queue.begin());
        sorted_queue.erase(sorted_queue.begin());
      }
      cv.notify_all();
    }

  }
}
