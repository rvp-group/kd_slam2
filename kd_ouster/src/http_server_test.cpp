#include <iostream>
#include "tiny_http.h"

using namespace std;

int main(int argc, char** argv) {
  uint16_t port = argc > 1 ? (uint16_t)stoi(argv[1]) : 8080;

  HttpServer server;
  if (!server.listen(port)) {
    cerr << "failed to listen on port " << port << endl;
    return -1;
  }
  cerr << "listening on port " << port << endl;

  server.on_request = [](const HTTPRequest& req) {
    cerr << req.method << " " << req.path << endl;
    HTTPResponse resp;
    resp.status = 200;
    resp.reason = "OK";
    string body = "hello from louster\n";
    resp.payload.assign(body.begin(), body.end());
    resp.headers.insert({"Content-Type", "text/plain"});
    return resp;
  };

  server.spin();
}
