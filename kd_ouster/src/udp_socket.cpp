#include "udp_socket.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
#include <cctype>

using namespace std;
// server on port
void UDPSocket::open(uint16_t port) {
  if (_socket_fd>0)
    close();
  
  _socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (_socket_fd == -1) 
    throw std::runtime_error("error in opening so");
  struct sockaddr_in addr = {0};
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port        = htons(port);
  if (bind(_socket_fd, (struct sockaddr *)&addr, sizeof(addr))<0) 
    throw std::runtime_error("error in binding");
}
void UDPSocket::open() {
  _socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
}

UDPSocket::~UDPSocket() {
  close();
}


// Connect to a server
bool UDPSocket::connectTo(const std::string& host, uint16_t port) {
  if (_socket_fd == -1) {
    std::cerr << "socket non existing" << std::endl;
    return false;
  }
  
  struct sockaddr_in peer_addr;
  // Convert IP address from text to binary form
  if (inet_pton(AF_INET, host.c_str(), &peer_addr.sin_addr) <= 0) {
    std::cerr << "Invalid IP address format." << std::endl;
    close();
    return false;
  }

  // 3. Connect to the server
  if (connect(_socket_fd, (struct sockaddr*)&peer_addr, sizeof(peer_addr)) < 0) {
    std::cerr << "Connection failed." << std::endl;
    close();
    return false;
  }

  return true;
}

// Send data to the server
int UDPSocket::sendData(const uint8_t* src, size_t s) {
  if (_socket_fd == -1)
    return -1;
  return send(_socket_fd, src, s, 0);
}

// Receive data from the server
int UDPSocket::receiveData(uint8_t* dest, size_t capacity) {
  if (_socket_fd == -1) return -1;

  return recv(_socket_fd, dest, capacity, 0);
}

// Explicitly close the connection
void UDPSocket::close() {
  if (_socket_fd != -1) {
    ::close(_socket_fd);
    _socket_fd = -1;
  }
}
