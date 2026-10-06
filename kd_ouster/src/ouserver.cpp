#include <chrono>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <atomic>
#include "tcp_socket.h"
#include "json_parser.h"
#include "ouster_metadata.h"
#include "tiny_http.h"
#include "udp_socket.h"
#include "ouster_legacy_packets.h"
#include "ouster_lidar.h"
#include <sys/select.h>
#include "fstream"
#include "dbuffer.h"
#include <thread>
#include <signal.h>
#include <atomic>

using namespace std;

volatile bool run=true;
// Whole-process shutdown, SIGINT
void handle_sigint(int sig) {
    run=0;
}

std::atomic<OusterLidar*> ouster_lidar=nullptr;


std::string timestampPrefix() {
  auto t=std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::ostringstream os;
  os << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
  return os.str();
}

// Old main()'s body, wrapped so it can be started/stopped as a session.
// session_run is this session's own flag, separate from the process-wide one.
void runSession(std::string host, int port, volatile bool& session_run, bool on_line) {
  OusterLidar oust;
  ouster_lidar=&oust;
  std::string ouster_config;
  oust.on_config = [&](const std::string& conf) {
    ouster_config=conf;
  };

  int open_ok=oust.open(host, port);
  if (open_ok!=1) {
    cerr << "sensor open failed: " << open_ok << endl;
    return;
  }

  size_t capacity=1024*1024*5;
  double swap_th=0.8;
  louster::DBuffer dbuf(capacity, swap_th);
  auto onPacket = [&](uint32_t addr, uint16_t port, const uint8_t* data, size_t size)->void {
    uint8_t *addr_bytes=(uint8_t*)&addr;
    constexpr size_t header_size=sizeof(size_t)+sizeof(addr)+sizeof(port);
    size_t total_size=header_size+size;
    uint8_t* dest= dbuf.getBuffer(total_size);
    if (dest) {
      uint8_t* d=dest;
      memcpy(d,&total_size, sizeof(total_size));
      d+=sizeof(total_size);
      memcpy(d,&addr, sizeof(addr));
      d+=sizeof(addr);
      memcpy(d,&port, sizeof(port));
      d+=sizeof(port);
      memcpy(d,data, size);
      d+=size;

      //checksum
      size_t wb=d-dest;
      if (wb!=total_size) {
        throw std::runtime_error("write size mismatch");
      }
    }
    if (! session_run) {
      oust.run=false;
      // force the trailing partial buffer to the writer before it gives up waiting
      while (! dbuf.done())
        std::this_thread::yield();
      dbuf.finish();
    }
  };
  std::string base_filename=timestampPrefix()+"_oust";
  auto write_fn = [&](std::string base_filename) {
    std::string dat_filename=base_filename+std::string(".dat");
    std::string rec_filename=base_filename+std::string(".json");
    JsonMapItem log_node;
    log_node.insert(std::make_pair("sensor_ip", std::make_unique<JsonSingleItem>(JsonItemBase::JsonItemType::STRING, host)));
    log_node.insert(std::make_pair("config", JsonParser::parse(ouster_config)));
    log_node.insert(std::make_pair("binary_file", std::make_unique<JsonSingleItem>(JsonItemBase::JsonItemType::STRING, dat_filename)));
    ofstream os(rec_filename);
    int lev=0;
    log_node.print(os,lev);
    os.close();
    os.clear();

    ofstream os2(dat_filename);
    while(1) {
      auto wbuf=dbuf.waitSwap();
      if (! wbuf)
        break;
      cerr << "write_block" << endl;
      os2.write((char*)wbuf->data(), wbuf->size());
      wbuf->reset();
    }
    os2.close();
  };

  if (! on_line) {
    std::thread consumer_thr(write_fn, base_filename);
    oust.on_packet=onPacket;
    oust.run=true;
    oust.spinLive();
    ouster_lidar=nullptr;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    consumer_thr.join();
    cerr << "session closed: " << base_filename << endl;
  } else {
    auto oc = [&](const OusterLidar::CloudMessageData& d) {
      cerr << "d: " << d.header.stamp_ns << endl;
      oust.run=session_run;
    };
    oust.on_cloud=oc;
    oust.run=true;
    oust.spinLive();
    ouster_lidar=nullptr;
    while (oust.run){
      sleep(1);
    }
    
  }
}

int main(int argc, char**argv) {
  if (argc<3) {
    return -1;
  }

  std::string host = argv[1];
  int port=atoi(argv[2]);
  uint16_t http_port = argc>3 ? (uint16_t)atoi(argv[3]) : 8080;
  bool live=false;
  if (argc>4)
    live=1;
  volatile bool session_run=false;
  volatile bool logging_active=false;
  std::thread session_thread;

  auto statsJson = [&]() {
    OusterLidar* oust=ouster_lidar;
    JsonMapItem root;
    auto num=[](long long v){ return std::make_unique<JsonSingleItem>(JsonItemBase::JsonItemType::NUMBER, std::to_string(v)); };
    root.insert(std::make_pair("logging", std::make_unique<JsonSingleItem>(JsonItemBase::JsonItemType::BOOLEAN, logging_active ? "true" : "false")));
    root.insert(std::make_pair("lidar_packets", num(oust? oust->lidar_packets_received: 0)));
    root.insert(std::make_pair("imu_packets", num(oust? oust->imus_received: 0)));
    ostringstream os;
    int lev=0;
    root.print(os, lev);
    return os.str();
  };

  // Fire and forget: start spawns the session thread, stop just flags it
  auto startLogging = [&]() {
    if (logging_active)
      return;
    if (session_thread.joinable())
      session_thread.join();
    session_run=true;
    logging_active=true;
    session_thread=std::thread([&]{
      try {
        runSession(host, port, session_run, live);
      } catch (const std::exception& e) {
        cerr << "session error: " << e.what() << endl;
      }
      logging_active=false;
    });
  };
  auto stopLogging = [&]() {
    session_run=false;
  };

  HttpServer http_server;
  if (http_server.listen(http_port)) {
    http_server.on_request = [&](const HTTPRequest& req) {
      HTTPResponse resp;
      if (req.path=="/stats") {
        std::string body=statsJson();
        resp.status=200;
        resp.reason="OK";
        resp.payload.assign(body.begin(), body.end());
        resp.headers.insert({"Content-Type", "application/json"});
      } else if (req.path=="/start") {
        startLogging();
        std::string body="{\"ok\":true}";
        resp.status=200;
        resp.reason="OK";
        resp.payload.assign(body.begin(), body.end());
        resp.headers.insert({"Content-Type", "application/json"});
      } else if (req.path=="/stop") {
        stopLogging();
        std::string body="{\"ok\":true}";
        resp.status=200;
        resp.reason="OK";
        resp.payload.assign(body.begin(), body.end());
        resp.headers.insert({"Content-Type", "application/json"});
      } else if (req.path=="/") {
        static const std::string body =
          "<button onclick=\"fetch('/start',{method:'POST'})\">Start</button>"
          "<button onclick=\"fetch('/stop',{method:'POST'})\">Stop</button>"
          "<pre id=s></pre><script>"
          "setInterval(function(){"
          "fetch('/stats').then(function(r){return r.json()}).then(function(j){"
          "document.getElementById('s').textContent=JSON.stringify(j,null,2)})}"
          ",1000)</script>";
        resp.status=200;
        resp.reason="OK";
        resp.payload.assign(body.begin(), body.end());
        resp.headers.insert({"Content-Type", "text/html"});
      } else {
        resp.status=404;
        resp.reason="Not Found";
      }
      return resp;
    };
    std::thread http_thr([&]{ http_server.spin(); });
    http_thr.detach();
    cerr << "http stats on port " << http_port << endl;
  } else {
    cerr << "failed to bind http port " << http_port << endl;
  }

  signal(SIGINT, handle_sigint);
  while (run) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  stopLogging();
  if (session_thread.joinable())
    session_thread.join();
  cerr << "closing" << endl;
}
