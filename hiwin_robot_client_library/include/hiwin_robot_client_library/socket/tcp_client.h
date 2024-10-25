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