#pragma once
#include <cstdint>
#include <string>
#include <list>
#include <vector>
#include <map>
#include <functional>
#include "tcp_socket.h"

struct HTTPRequest{
  std::string version = "HTTP/1.1";
  std::string method;        // "GET"
  std::string path;
  std::list<std::pair<std::string, std::string> > headers;// stuff
  std::vector<char> payload;

  std::vector<char> toByteArray() const;
};

struct HTTPResponse{
  std::string version = "HTTP/1.1";
  int status = 0;
  std::string reason;
  std::multimap<std::string, std::string >  headers;// stuff
  std::vector<char> payload;

  std::vector<char> toByteArray() const;
};

HTTPResponse sendRequest(const HTTPRequest& req, std::string host, uint16_t port);

// Reads one request off an accepted connection, method empty on error
HTTPRequest receiveRequest(SimpleTcpSocket& sock);

struct HttpServer {
  bool listen(uint16_t port);
  void spin();  // blocking accept loop, one request per connection
  std::function<HTTPResponse(const HTTPRequest&)> on_request;
  SimpleTcpSocket socket;
};
