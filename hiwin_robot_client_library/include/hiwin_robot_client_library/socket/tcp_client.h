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

#ifndef HIWIN_ROBOT_CLIENT_LIBRARY_SOCKET_TCP_CLIENT_H_
#define HIWIN_ROBOT_CLIENT_LIBRARY_SOCKET_TCP_CLIENT_H_

#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <memory>

namespace hrsdk
{
namespace socket
{

enum class SocketState
{
  Invalid,       ///< Socket is initialized or setup failed
  Connected,     ///< Socket is connected and ready to use
  Disconnected,  ///< Socket is disconnected and cannot be used
  Closed         ///< Connection to socket got closed
};

class TCPClient
{
private:
  std::atomic<int> socket_fd_;
  std::atomic<SocketState> state_;
  std::chrono::milliseconds reconnection_time_;

  void setupOptions();

protected:
  static bool open(int socket_fd, struct sockaddr* address, size_t address_len)
  {
    return ::connect(socket_fd, address, address_len) == 0;
  }

  std::unique_ptr<timeval> recv_timeout_;

public:
  static constexpr std::chrono::milliseconds DEFAULT_RECONNECTION_TIME{ 10000 };

  bool setup(const std::string& host, const int port, const std::chrono::milliseconds reconnection_time);

  TCPClient(/* args */);
  virtual ~TCPClient();

  SocketState getState()
  {
    return state_;
  }

  bool read(uint8_t* buf, const size_t buf_len, size_t& read);
  
  bool write(const uint8_t* buf, const size_t buf_len, size_t& written);

  void close();
};

}  // namespace socket
}  // namespace hrsdk

#endif  // HIWIN_ROBOT_CLIENT_LIBRARY_SOCKET_TCP_CLIENT_H_