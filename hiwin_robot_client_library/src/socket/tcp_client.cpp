/*
 * Copyright 2024, HIWIN Technologies Corp. (refactor)
 *
 * Copyright 2019, FZI Forschungszentrum Informatik (refactor)
 *
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <arpa/inet.h>
#include <endian.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <chrono>
#include <cstring>
#include <sstream>
#include <thread>
#include <iostream>

#include <hiwin_robot_client_library/socket/tcp_client.h>

namespace hrsdk
{
namespace socket
{

TCPClient::TCPClient() : socket_fd_(-1), state_(SocketState::Invalid), reconnection_time_(std::chrono::seconds(10))
{
}

TCPClient::~TCPClient()
{
  close();
}

void TCPClient::setupOptions()
{
  int flag = 1;
  setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int));
  setsockopt(socket_fd_, IPPROTO_TCP, TCP_QUICKACK, &flag, sizeof(int));

  if (recv_timeout_ != nullptr)
  {
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, recv_timeout_.get(), sizeof(timeval));
  }
}

bool TCPClient::setup(const std::string& ip_addr, const int port, const std::chrono::milliseconds reconnection_time)
{
  auto reconnection_time_resolved = reconnection_time;

  // Configure server address
  sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  inet_pton(AF_INET, ip_addr.c_str(), &server_addr.sin_addr);

  bool connected = false;
  while (!connected)
  {
    // Create a socket
    socket_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ != -1 && open(socket_fd_, (sockaddr*)&server_addr, sizeof(server_addr)))
    {
      connected = true;
      break;
    }

    if (!connected)
    {
      state_ = SocketState::Invalid;
      std::cout << "Failed to connect to robot." << std::endl;
      std::this_thread::sleep_for(reconnection_time_resolved);
    }
  }
  setupOptions();
  state_ = SocketState::Connected;
  std::cout << "Connection established for " << ip_addr << ":" << port << std::endl;
  return connected;
}

bool TCPClient::read(uint8_t* buf, const size_t buf_len, size_t& read)
{
  read = 0;

  if (state_ != SocketState::Connected)
    return false;

  ssize_t res = ::recv(socket_fd_, buf, buf_len, 0);

  if (res == 0)
  {
    state_ = SocketState::Disconnected;
    return false;
  }
  else if (res < 0)
    return false;

  read = static_cast<size_t>(res);
  return true;
}

bool TCPClient::write(const uint8_t* buf, const size_t buf_len, size_t& written)
{
  written = 0;

  if (state_ != SocketState::Connected)
  {
    std::cout << "Attempt to write on a non-connected socket" << std::endl;
    return false;
  }

  size_t remaining = buf_len;

  // handle partial sends
  while (written < buf_len)
  {
    ssize_t sent = ::send(socket_fd_, buf + written, remaining, 0);

    if (sent <= 0)
    {
      std::cout << "Sending data through socket failed." << std::endl;
      return false;
    }

    written += sent;
    remaining -= sent;
  }

  return true;
}

void TCPClient::close()
{
  if (socket_fd_ >= 0)
  {
    state_ = SocketState::Closed;
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

}  // namespace socket
}  // namespace hrsdk
