#include "message_queue.h"

void MessageQueueBounded::push(std::shared_ptr<MessageBase> f) {
  std::unique_lock<std::mutex> lk(mtx);
  cv.wait(lk, [&]{ return q.size() < capacity; });
  q.push(std::move(f));
  lk.unlock();
  cv.notify_one();
}

std::shared_ptr<MessageBase> MessageQueueBounded::pop()  {
  std::unique_lock<std::mutex> lk(mtx);
  cv.wait(lk, [&]{ return !q.empty() || done; });
  if (q.empty()) return nullptr;
  auto f = std::move(q.front());
  q.pop();
  lk.unlock();
  cv.notify_one();
  return f;
}

int MessageQueueBounded::size() const  {return q.size();}

void MessageQueueBounded::setDone() {
  std::lock_guard<std::mutex> lk(mtx);
  done = true;
  cv.notify_all();
}
