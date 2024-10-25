#ifndef HIWIN_ROBOT_CLIENT_LIBRARY_COMMANDER_H_
#define HIWIN_ROBOT_CLIENT_LIBRARY_COMMANDER_H_

#include <hiwin_robot_client_library/socket/tcp_client.h>

namespace hrsdk
{

enum class RobotMode : uint16_t
{
  Manual = 0,
  Auto
};

class Commander : public socket::TCPClient
{
private:
  std::string robot_ip_;

public:
  Commander(const std::string& robot_ip);
  ~Commander();

  bool connect();

  int getPermissions();
  int enableRobot();

  int getActualRPM(double (&velocities)[6]);
  int getActualPosition(double (&positions)[6]);
  int getActualCurrent(double (&efforts)[6]);
  int ptpJoint(double (&positions)[6], double ratio);

  int getHrssMode();

  int setRobotMode(RobotMode mode);
  int getRobotMode(RobotMode& mode);
};

}  // namespace hrsdk

#endif  // HIWIN_ROBOT_CLIENT_LIBRARY_COMMANDER_H_