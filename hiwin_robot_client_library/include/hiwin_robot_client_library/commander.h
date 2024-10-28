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

enum class MotionStatus : uint16_t
{
  ServerOff = 0,
  Waiting = 1,
  Running = 2,
  Hold = 3,
  Delay = 4,
  Moving = 5,
};

enum LogLevels
{
  None = 0,
  Info,
  SetCommand,
  Console,
  Save,
};

class Commander : public socket::TCPClient
{
private:
  std::string robot_ip_;

public:
  Commander(const std::string& robot_ip);
  ~Commander();

  bool connect();
  bool isRemoteMode();

  int getPermissions();
  int setLogLevel(LogLevels level);
  int enableRobot();

  int getActualRPM(double (&velocities)[6]);
  int getActualPosition(double (&positions)[6]);
  int getActualCurrent(double (&efforts)[6]);
  int getMotionState(MotionStatus& status);
  int getErrorCode(std::vector<std::string>& error_code);

  int ptpJoint(double (&positions)[6], double ratio);
  int ptpJointScript(int points_count, double (&positions)[100][6], double ratio);
  int setPtpSpeed(int ratio);
  int getPtpSpeed(int& ratio);
  int setOverrideRatio(int ratio);
  int getOverrideRatio(int& ratio);

  int setRobotMode(RobotMode mode);
  int getRobotMode(RobotMode& mode);
  int GetRobotVersion(std::string& str);
};

}  // namespace hrsdk

#endif  // HIWIN_ROBOT_CLIENT_LIBRARY_COMMANDER_H_