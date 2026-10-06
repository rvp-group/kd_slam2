#pragma once
#include <thread>
#include "kd_io/message_queue.h"
#include "kd_slam/event/event_queue.h"

struct KDLiveOuster{
  std::string host;
  int port;
  std::shared_ptr<MessageQueueBounded> message_queue;
  std::shared_ptr<kd_slam::event::EventQueue> ev_queue;
  volatile bool& run_flag;
  KDLiveOuster(std::string host,
                    int port,
                    std::shared_ptr<MessageQueueBounded> message_queue,
                    std::shared_ptr<kd_slam::event::EventQueue> ev_queue,
                    volatile bool& run_flag);

  void run();
  void join();
  std::thread run_th;
};
