/*
 * -- BEGIN LICENSE BLOCK ----------------------------------------------
 * Copyright 2024 HIWIN Technologies Corp.
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
 *
 * -- END LICENSE BLOCK ------------------------------------------------
 */

#ifndef HIWIN_ROBOT_CLIENT_LIBRARY_COMMANDER_H_
#define HIWIN_ROBOT_CLIENT_LIBRARY_COMMANDER_H_

#include <hiwin_robot_client_library/socket/tcp_client.h>

namespace hrsdk
{

enum class ControlMode : uint16_t
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
  int setServoAmpState(bool& enable);
  int getServoAmpState(bool& enable);

  int getActualRPM(double (&velocities)[6]);
  int getActualPosition(double (&positions)[6]);
  int getActualCurrent(double (&efforts)[6]);
  int getMotionState(MotionStatus& status);
  int getErrorCode(std::vector<std::string>& error_code);

  int ptpJoint(double (&positions)[6], double ratio);
  int motionAbort();
  int setPtpSpeed(int ratio);
  int getPtpSpeed(int& ratio);
  int setOverrideRatio(int ratio);
  int getOverrideRatio(int& ratio);

  int setRobotMode(ControlMode mode);
  int getRobotMode(ControlMode& mode);
  int GetRobotVersion(std::string& str);
};

}  // namespace hrsdk

#endif  // HIWIN_ROBOT_CLIENT_LIBRARY_COMMANDER_H_