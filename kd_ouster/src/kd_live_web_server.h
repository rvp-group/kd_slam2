#pragma once
#include <thread>
#include "tiny_http.h"
#include "kd_live_running_context.h"
#include "kd_live_logger.h"

struct KDWebServer {
  KDLiveRunningContext& context;
  KDWebServer(KDLiveRunningContext& ctx, uint16_t port);
  HttpServer http_server;
  uint16_t http_port;
  // spawns a dangling thread
  std::thread runner_th;
  void run();
  void join();
};
