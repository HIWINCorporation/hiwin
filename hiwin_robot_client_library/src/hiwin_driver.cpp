#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <bits/stdc++.h>

#include <ros/ros.h>

#include <hiwin_robot_client_library/hiwin_driver.h>
#include <hiwin_robot_client_library/hiwin_driver.h>

namespace hrsdk
{
HIWINDriver::HIWINDriver(const std::string& robot_ip) : robot_ip_(robot_ip)
{
}

HIWINDriver::~HIWINDriver()
{
  disconnect();  // Ensure disconnection on destruction
}

bool HIWINDriver::connect()
{
  commander_.reset(new hrsdk::Commander(robot_ip_));
  if (!commander_->connect())
  {
    return false;
  }

  handle_client_.setup(robot_ip_, 1504, std::chrono::seconds(10));
  file_client_.setup(robot_ip_, 1505, std::chrono::seconds(10));

  commander_->GetRobotVersion(robot_version_);
  std::cout << robot_version_.c_str() << std::endl;

  commander_->getPermissions();
  commander_->setLogLevel(LogLevels::SetCommand);

  commander_->setRobotMode(ControlMode::Auto);
  commander_->setPtpSpeed(100);
  commander_->setOverrideRatio(100);

  commander_->getActualPosition(prev_target_joint_positions_);

  servoAmpState_ = true;
  commander_->setServoAmpState(servoAmpState_);

  return true;
}

void HIWINDriver::disconnect()
{
}

void HIWINDriver::getRobotMode(ControlMode& mode)
{
  commander_->getRobotMode(mode);
}

bool HIWINDriver::isEstopped()
{
  return false;
}

bool HIWINDriver::isDrivesPowered()
{
  commander_->getServoAmpState(servoAmpState_);
  return servoAmpState_;
}

bool HIWINDriver::isMotionPossible()
{
  if (!servoAmpState_ || error_list_.size() || !commander_->isRemoteMode())
  {
    return false;
  }
  return true;
}

bool HIWINDriver::isInMotion()
{
  commander_->getMotionState(robotStatus_);
  if (robotStatus_ == MotionStatus::Moving)
  {
    return true;
  }
  return false;
}

bool HIWINDriver::isInError()
{
  commander_->getErrorCode(error_list_);
  if (error_list_.empty())
  {
    return false;
  }
  return true;
}

void HIWINDriver::getJointVelocity(std::vector<double>& velocities)
{
  double value[6];
  if (velocities.size() < 6)
  {
    return;
  }

  if (commander_->getActualRPM(value) != 0)
  {
    return;
  }

  velocities.assign(value, value + 6);
  return;
}

void HIWINDriver::getJointEffort(std::vector<double>& efforts)
{
  double value[6];
  if (efforts.size() < 6)
  {
    return;
  }

  if (commander_->getActualCurrent(value) != 0)
  {
    return;
  }

  efforts.assign(value, value + 6);
  return;
}

void HIWINDriver::getJointPosition(std::vector<double>& positions)
{
  double value[6];
  if (positions.size() < 6)
  {
    return;
  }

  if (commander_->getActualPosition(value) != 0)
  {
    return;
  }

  positions.assign(value, value + 6);
  return;
}

void HIWINDriver::writeJointCommand(const std::vector<double>& positions, const float goal_time)
{
  double distance[6];
  double value[6];
  if (positions.size() != 6)
  {
    return;
  }

  for (size_t i = 0; i < 6; i++)
  {
    distance[i] = positions[i] - prev_target_joint_positions_[i];
    (distance[i] < 0) ? distance[i] *= -1 : distance[i] *= 1;
    prev_target_joint_positions_[i] = positions[i];
  }

  double max_distance = *std::max_element(distance, distance + 6);
  double velocity = max_distance / goal_time;

  /*
   * ... TBD.
   */

  std::copy(positions.begin(), positions.end(), value);

  commander_->ptpJoint(value, 100);
}

void HIWINDriver::writeJointTrajectory(const std::vector<std::vector<double> >& positions, const float goal_time)
{
  std::vector<double> point;
  double points[100][6];
  if (positions.size() == 0 || positions.size() > 100)
  {
    return;
  }

  for (size_t i = 0; i < positions.size(); i++)
  {
    if (positions[i].size() != 6)
    {
      return;
    }
    for (size_t j = 0; j < positions[i].size(); j++)
    {
      std::copy(positions[i].begin(), positions[i].end(), points[i]);
    }
  }

  if (commander_->ptpJointScript(positions.size(), points, 100) != 0)
  {
    return;
  }
}

void HIWINDriver::motionAbort()
{
  commander_->motionAbort();
}

}  // namespace hrsdk
