#include "tiny_http.h"
#include "tcp_socket.h"
#include <map>
#include <list>
#include <iostream>
#include <sstream>

std::vector<char> HTTPRequest::toByteArray() const {
  std::ostringstream os;
  os << method << " " <<  path << " " << version << "\r\n";
  for (auto& h:headers) {
    os << h.first << ": " << h.second << "\r\n";
  }
  if (payload.size()) {
    os << "Content-Length: " << payload.size() << "\r\n";
    os << "\r\n";
  }
  os <<"\r\n";
  auto http_header=os.str();
  std::vector<char> out;
  out.insert(out.end(), http_header.begin(), http_header.end());
  out.insert(out.end(), payload.begin(), payload.end());
  return out;
}


HTTPResponse sendRequest(const HTTPRequest& req, std::string host, uint16_t port) {
  using namespace std;
  SimpleTcpSocket sock;
  HTTPResponse resp;
  bool result = sock.connectTo(host, port);
  if (! result)
    return resp;
  static constexpr int buffer_size=4096;
  char buffer[buffer_size];
  std::vector<char> request_buffer=req.toByteArray();
  //cerr << request_buffer.data();
  sock.sendData((const uint8_t*)request_buffer.data(), request_buffer.size());
  std::vector<char> response_buffer;
  response_buffer.reserve(1<<20);
  while (1) {
    int bytes_read= sock.receiveData((uint8_t*)buffer, buffer_size-1);
    if (bytes_read<=0)
      break;
    response_buffer.insert(response_buffer.end(), buffer, buffer+bytes_read);
  }
  // parse the response string
  char* b_start=response_buffer.data();
  //cerr << b_start;
  bool first_line=true;
  for (int i=0; i< -1+(int)response_buffer.size(); ++i){
    if (response_buffer[i]=='\r' && response_buffer[i+1]=='\n') {
      response_buffer[i]=0;
      ++i;
      std::string line(b_start);
      std::istringstream is(line);
      if (first_line) {
        is >> resp.version;
        is >> resp.status;
        is >> resp.reason;
        first_line=false;
      } else {
        if (line.empty()) {
          b_start=response_buffer.data()+i+1;
        } else {
          enum State {None,Key, Value};
          State state=None;
          char* k_start=line.data();
          char* v_start=0;
          for (int i=0; i<line.length() && ! v_start; ++i) {
            char c=line[i];
            switch (state) {
            case None:
              if (c==' ') {
                ++k_start;
                continue;
              } else {
                state=Key;
                continue;
              }
              break;
            case Key:
              if (c==' ') {
                line[i]=0;
              }
              if (c==':') {
                line[i]=0;
                state=Value;
              }
              break;
            case Value:
              if (c==' ') {
                line[i]=0;
              } else {
                v_start = line.data()+i;
              }
            }
          }
          std::string key(k_start);
          std::string value;
          if (v_start)
            value = std::string(v_start);
          resp.headers.insert(std::make_pair(key, value));
        } 
      }
      b_start=response_buffer.data()+i+1;
    }
  }
  size_t offset=b_start-response_buffer.data();
  resp.payload.insert(resp.payload.end(),
                      response_buffer.begin()+offset,
                      response_buffer.end());
  return resp;
}

std::vector<char> HTTPResponse::toByteArray() const {
  std::ostringstream os;
  os << version << " " << status << " " << reason << "\r\n";
  for (auto& h:headers) {
    os << h.first << ": " << h.second << "\r\n";
  }
  os << "Content-Length: " << payload.size() << "\r\n";
  os << "\r\n";
  auto http_header=os.str();
  std::vector<char> out;
  out.insert(out.end(), http_header.begin(), http_header.end());
  out.insert(out.end(), payload.begin(), payload.end());
  return out;
}

HTTPRequest receiveRequest(SimpleTcpSocket& sock) {
  using namespace std;
  HTTPRequest req;
  static constexpr int buffer_size=4096;
  char buffer[buffer_size];
  vector<char> buf;
  buf.reserve(1<<16);

  // read until the blank line that ends the headers
  size_t header_end=string::npos;
  while (header_end==string::npos) {
    int n=sock.receiveData((uint8_t*)buffer, buffer_size);
    if (n<=0)
      return req; // closed/error, method stays empty
    size_t search_from = buf.size()>=3 ? buf.size()-3 : 0;
    buf.insert(buf.end(), buffer, buffer+n);
    string s(buf.begin()+search_from, buf.end());
    size_t pos=s.find("\r\n\r\n");
    if (pos!=string::npos)
      header_end=search_from+pos;
  }

  string head(buf.begin(), buf.begin()+header_end);
  istringstream hs(head);
  string line;
  bool first_line=true;
  size_t content_length=0;
  while (getline(hs, line)) {
    if (!line.empty() && line.back()=='\r')
      line.pop_back();
    if (first_line) {
      istringstream is(line);
      is >> req.method >> req.path >> req.version;
      first_line=false;
      continue;
    }
    size_t colon=line.find(':');
    if (colon==string::npos)
      continue;
    string key=line.substr(0, colon);
    size_t vstart=line.find_first_not_of(' ', colon+1);
    string value = vstart==string::npos ? "" : line.substr(vstart);
    req.headers.push_back({key, value});
    if (key=="Content-Length")
      content_length=stoul(value);
  }

  req.payload.assign(buf.begin()+header_end+4, buf.end());
  while (req.payload.size()<content_length) {
    int n=sock.receiveData((uint8_t*)buffer, buffer_size);
    if (n<=0)
      break;
    req.payload.insert(req.payload.end(), buffer, buffer+n);
  }
  return req;
}

bool HttpServer::listen(uint16_t port) {
  return socket.listen(port);
}

void HttpServer::spin() {
  while (true) {
    auto conn=socket.accept();
    if (!conn)
      continue;
    HTTPRequest req=receiveRequest(*conn);
    if (req.method.empty())
      continue;
    HTTPResponse resp;
    if (on_request)
      resp=on_request(req);
    else {
      resp.status=404;
      resp.reason="Not Found";
    }
    auto bytes=resp.toByteArray();
    conn->sendData((const uint8_t*)bytes.data(), bytes.size());
  }
}
