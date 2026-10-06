#include "tcp_socket.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
#include <cctype>

using namespace std;
SimpleTcpSocket::~SimpleTcpSocket() {
  closeConnection();
}


// Connect to a server
bool SimpleTcpSocket::connectTo(const std::string& ipAddress, int port) {
  // 1. Create the TCP socket
  _socket_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (_socket_fd == -1) {
    std::cerr << "Failed to create socket." << std::endl;
    return false;
  }

  // 2. Setup server address structure
  sockaddr_in serverAddr{};
  serverAddr.sin_family = AF_INET;
  serverAddr.sin_port = htons(port);

  // Convert IP address from text to binary form
  if (inet_pton(AF_INET, ipAddress.c_str(), &serverAddr.sin_addr) <= 0) {
    std::cerr << "Invalid IP address format." << std::endl;
    closeConnection();
    return false;
  }

  // 3. Connect to the server
  if (connect(_socket_fd, reinterpret_cast<struct sockaddr*>(&serverAddr), sizeof(serverAddr)) < 0) {
    std::cerr << "Connection failed." << std::endl;
    closeConnection();
    return false;
  }

  return true;
}

// Send data to the server
int SimpleTcpSocket::sendData(const uint8_t* src, size_t s) {
  if (_socket_fd == -1)
    return -1;
  return send(_socket_fd, src, s, 0);
}

int SimpleTcpSocket::sendData(const std::string& src) {
  size_t l=src.length();
  return sendData((const uint8_t*)src.c_str(), l);
}

// Receive data from the server
int SimpleTcpSocket::receiveData(uint8_t* dest, size_t capacity) {
  if (_socket_fd == -1) return -1;

  return recv(_socket_fd, dest, capacity, 0);
}

// Explicitly close the connection
void SimpleTcpSocket::closeConnection() {
  if (_socket_fd != -1) {
    close(_socket_fd);
    _socket_fd = -1;
  }
}

// Bind + listen, this socket becomes the listening socket
bool SimpleTcpSocket::listen(uint16_t port, int backlog) {
  _socket_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (_socket_fd == -1) {
    std::cerr << "Failed to create socket." << std::endl;
    return false;
  }

  int reuse = 1;
  setsockopt(_socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port);

  if (bind(_socket_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    std::cerr << "Bind failed." << std::endl;
    closeConnection();
    return false;
  }

  if (::listen(_socket_fd, backlog) < 0) {
    std::cerr << "Listen failed." << std::endl;
    closeConnection();
    return false;
  }

  return true;
}

// Blocks for one incoming connection, null on error
std::unique_ptr<SimpleTcpSocket> SimpleTcpSocket::accept() {
  int client_fd = ::accept(_socket_fd, nullptr, nullptr);
  if (client_fd < 0) {
    std::cerr << "Accept failed." << std::endl;
    return nullptr;
  }
  return std::unique_ptr<SimpleTcpSocket>(new SimpleTcpSocket(client_fd));
}

