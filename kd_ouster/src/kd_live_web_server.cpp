#include "kd_live_web_server.h"
#include <iostream>
#include <string>

using namespace std;


KDWebServer::KDWebServer(KDLiveRunningContext& ctx, uint16_t port) :
  context(ctx),
  http_port(port){
}

void KDWebServer::run() {
  auto run_fn = [&]() {
    if (http_server.listen(http_port)) {
      http_server.on_request = [&](const HTTPRequest& req) {
        HTTPResponse resp;
        if (req.path=="/stats") {
          std::string body=context.ouster_logger->statsJson();
          resp.status=200;
          resp.reason="OK";
          resp.payload.assign(body.begin(), body.end());
          resp.headers.insert({"Content-Type", "application/json"});
        } else if (req.path=="/start") {
          if (context.proc)
            context.proc->run(true);
          std::string body="{\"ok\":true}";
          resp.status=200;
          resp.reason="OK";
          resp.payload.assign(body.begin(), body.end());
          resp.headers.insert({"Content-Type", "application/json"});
        } else if (req.path=="/stop") {
          if (context.proc)
            context.proc->run(false);
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
  };

  runner_th=std::thread(run_fn);
}  

void KDWebServer::join(){
  if (runner_th.joinable()){
    runner_th.join();
  }
}
