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

#ifndef HIWIN_ROBOT_CLIENT_LIBRARY_HIWIN_DRIVER_H_
#define HIWIN_ROBOT_CLIENT_LIBRARY_HIWIN_DRIVER_H_

#include <string>
#include <vector>

#include <hiwin_robot_client_library/socket/tcp_client.h>
#include <hiwin_robot_client_library/commander.h>

namespace hrsdk
{
class HIWINDriver
{
private:
  std::string robot_ip_;  // IP address of the robot
  std::string robot_version_;

  bool servoAmpState_;
  MotionStatus robotStatus_;
  std::vector<std::string> error_list_;

  double prev_target_joint_positions_[6];

  std::unique_ptr<hrsdk::Commander> commander_;
  socket::TCPClient handle_client_;
  socket::TCPClient file_client_;

public:
  HIWINDriver(const std::string& robot_ip);
  ~HIWINDriver();

  bool connect();     // Connect to the robot
  void disconnect();  // Disconnect from the robot

  void writeJointCommand(const std::vector<double>& positions, const float goal_time);
  void motionAbort();

  void getJointVelocity(std::vector<double>& velocities);
  void getJointEffort(std::vector<double>& efforts);
  void getJointPosition(std::vector<double>& positions);
  void getRobotMode(ControlMode& mode);
  bool isEstopped();
  bool isDrivesPowered();
  bool isMotionPossible();
  bool isInMotion();
  bool isInError();
};

}  // namespace hrsdk

#endif  // HIWIN_ROBOT_CLIENT_LIBRARY_HIWIN_DRIVER_H_
