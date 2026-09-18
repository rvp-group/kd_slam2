#pragma once
#include <mutex>
#include <condition_variable>
#include <queue>
#include "slam_messages.h"

// multithreaded frame queue of fixed capacity
struct MessageQueueBounded {

  MessageQueueBounded(size_t cap=100): capacity(cap) {}
  // pushes one element in the queue, or stops the caller
  // if the queue exceeds the capacity
  void push(std::shared_ptr<MessageBase> f);
  
  // pops one element in the queue, or stops the caller
  // if queue is empty
  std::shared_ptr<MessageBase> pop();

  // typed return value
  // it scans the queue until an element of MessageType is found,
  // skipping all others
  template <typename MessageType>
  std::shared_ptr<MessageType> popT();

  // size accessor
  int size() const;

  // flushes
  void setDone();

  std::mutex mtx;
  std::condition_variable cv;
  bool done = false;

  std::queue<std::shared_ptr<MessageBase>> q;
  
  size_t capacity;

};


template <typename MessageType>
std::shared_ptr<MessageType> MessageQueueBounded::popT() {
  std::shared_ptr<MessageType> p_out=nullptr;
  do  {
    auto p=pop();
    if (p==nullptr)
      return nullptr;
    p_out=std::dynamic_pointer_cast<MessageType>(p);
  } while (! p_out);
  return p_out;
}

