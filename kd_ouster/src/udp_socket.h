#pragma once
#include <string>
#include <cstdint>
#include <unistd.h>
#include <arpa/inet.h>

struct UDPSocket {
  // Constructor
  UDPSocket()=default;

  // Destructor automatically closes the socket
  ~UDPSocket();
  UDPSocket(const UDPSocket&) = delete;
  UDPSocket& operator=(const UDPSocket&) = delete;

  void open();
  void open(uint16_t port);
  bool connectTo(const std::string& host, uint16_t port);
  void disconnect();
  // Prevent copying to avoid duplicate socket closure
  
  int sendData(const uint8_t* src, size_t s);

  // Receive data from the peer
  int receiveData(uint8_t* dest, size_t capacity);

  // Explicitly close the connection
  void close();

  int _socket_fd=-1;

};
