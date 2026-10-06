#pragma once
#include <string>
#include <cstdint>
#include <memory>

struct SimpleTcpSocket {
  // Constructor
  SimpleTcpSocket() = default;

  // Destructor automatically closes the socket
  ~SimpleTcpSocket();

  // Prevent copying to avoid duplicate socket closure
  SimpleTcpSocket(const SimpleTcpSocket&) = delete;
  SimpleTcpSocket& operator=(const SimpleTcpSocket&) = delete;

  // Connect to a server
  bool connectTo(const std::string& ipAddress, int port);
  // Send data to the server
  int sendData(const uint8_t* src, size_t s);
  int sendData(const std::string& s);
  // Receive data from the server
  int receiveData(uint8_t* dest, size_t capacity);
  // Explicitly close the connection
  void closeConnection();
  // Bind + listen, this socket becomes the listening socket
  bool listen(uint16_t port, int backlog = 5);
  // Blocks for one incoming connection, null on error
  std::unique_ptr<SimpleTcpSocket> accept();

  int _socket_fd = -1;

 private:
  explicit SimpleTcpSocket(int fd) : _socket_fd(fd) {}
};
