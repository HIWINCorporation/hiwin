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

#include <time.h>

#include <pluginlib/class_list_macros.hpp>

#include <hiwin_driver/hardware_interface.h>

#define NSEC_PER_SEC (1000000000)
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + (B).tv_nsec - (A).tv_nsec)
#define MEASURE_TIMING

using industrial_robot_status_interface::RobotMode;
using industrial_robot_status_interface::TriState;

namespace hiwin_driver
{
HardwareInterface::HardwareInterface()
  : joint_names_(6)
  , joint_positions_({ 0, 0, 0, 0, 0, 0 })
  , joint_velocities_({ 0, 0, 0, 0, 0, 0 })
  , joint_efforts_({ 0, 0, 0, 0, 0, 0 })
  , joint_position_command_({ 0, 0, 0, 0, 0, 0 })
  , joint_trajectory_command_({ 0, 0, 0, 0, 0, 0 })
  , target_joint_positions_({ 0, 0, 0, 0, 0, 0 })
  , target_joint_velocities_({ 0, 0, 0, 0, 0, 0 })
{
}

bool HardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  if (!robot_hw_nh.getParam("robot_ip", robot_ip_))
  {
    ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("robot_ip") << " not given.");
    return false;
  }

  // Names of the joints. Usually, this is given in the controller config file.
  if (!robot_hw_nh.getParam("joints", joint_names_))
  {
    ROS_ERROR_STREAM("Cannot find required parameter " << robot_hw_nh.resolveName("joints")
                                                       << " on the parameter server.");
    return false;
  }

  // Create ros_control interfaces
  for (std::size_t i = 0; i < joint_positions_.size(); ++i)
  {
    ROS_DEBUG_STREAM("Registering handles for joint " << joint_names_[i]);

    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_positions_[i],
                                                                      &joint_velocities_[i], &joint_efforts_[i]));

    // Create joint position control interface
    pj_interface_.registerHandle(
        hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));

    // Create joint trajectory control interface
    jnt_traj_interface_.registerHandle(hardware_interface::JointTrajectoryHandle(
        js_interface_.getHandle(joint_names_[i]), &joint_trajectory_command_[i]));
  }

  jnt_traj_interface_.registerGoalCallback(
      std::bind(&HardwareInterface::startJointInterpolation, this, std::placeholders::_1));
  jnt_traj_interface_.registerCancelCallback(std::bind(&HardwareInterface::abortMotion, this));

  robot_status_interface_.registerHandle(industrial_robot_status_interface::IndustrialRobotStatusHandle(
      "industrial_robot_status_handle", robot_status_resource_));

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&jnt_traj_interface_);
  registerInterface(&robot_status_interface_);

  hiwin_driver_.reset(new hrsdk::HIWINDriver(robot_ip_));
  hiwin_driver_->connect();

  return true;
}

void HardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  hrsdk::ControlMode ctrl_mode;
  hiwin_driver_->getRobotMode(ctrl_mode);
  if (ctrl_mode == hrsdk::ControlMode::Manual)
  {
    robot_status_resource_.mode = RobotMode::MANUAL;
  }
  else if (ctrl_mode == hrsdk::ControlMode::Auto)
  {
    robot_status_resource_.mode = RobotMode::AUTO;
  }
  else
  {
    robot_status_resource_.mode = RobotMode::UNKNOWN;
  }
  robot_status_resource_.drives_powered = (hiwin_driver_->isDrivesPowered()) ? TriState::TRUE : TriState::FALSE;
  robot_status_resource_.in_error = (hiwin_driver_->isInError()) ? TriState::TRUE : TriState::FALSE;
  robot_status_resource_.motion_possible = (hiwin_driver_->isMotionPossible()) ? TriState::TRUE : TriState::FALSE;
  robot_status_resource_.in_motion = (hiwin_driver_->isInMotion()) ? TriState::TRUE : TriState::FALSE;
  robot_status_resource_.e_stopped = (hiwin_driver_->isEstopped()) ? TriState::TRUE : TriState::FALSE;
  robot_status_resource_.error_code = 0;
  hiwin_driver_->getJointPosition(joint_positions_);
  hiwin_driver_->getJointVelocity(joint_velocities_);

  control_msgs::FollowJointTrajectoryFeedback feedback = control_msgs::FollowJointTrajectoryFeedback();
  for (size_t i = 0; i < 6; i++)
  {
    // not provide command return
    target_joint_positions_[i] = joint_positions_[i];
    target_joint_velocities_[i] = joint_velocities_[i];

    feedback.joint_names.push_back(joint_names_[i]);
    feedback.desired.positions.push_back(target_joint_positions_[i]);
    feedback.desired.velocities.push_back(target_joint_velocities_[i]);
    feedback.actual.positions.push_back(joint_positions_[i]);
    feedback.actual.velocities.push_back(joint_velocities_[i]);
    feedback.error.positions.push_back(std::abs(joint_positions_[i] - target_joint_positions_[i]));
    feedback.error.velocities.push_back(std::abs(joint_velocities_[i] - target_joint_velocities_[i]));
  }
  jnt_traj_interface_.setFeedback(feedback);
}

void HardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
}

void HardwareInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                 const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  for (auto& controller_it : stop_list)
  {
    for (auto& resource_it : controller_it.claimed_resources)
    {
    }
  }

  for (auto& controller_it : start_list)
  {
    for (auto& resource_it : controller_it.claimed_resources)
    {
    }
  }
}

bool HardwareInterface::shouldResetControllers()
{
  if (controller_reset_necessary_)
  {
    controller_reset_necessary_ = false;
    return true;
  }
  else
  {
    return false;
  }
}

void HardwareInterface::startJointInterpolation(const control_msgs::FollowJointTrajectoryGoal& trajectory)
{
  ROS_DEBUG("Starting joint-based trajectory forward");

#ifdef MEASURE_TIMING
  struct timespec start_time;
  struct timespec end_time;
  uint32_t exec_ns = 0;
  uint32_t exec_max_ns = 0;
  uint32_t exec_min_ns = 0xFFFFFFFF;
#endif

  size_t point_number = trajectory.trajectory.points.size();
  double last_time = 0.0;

  for (size_t i = 0; i < point_number; i++)
  {
    trajectory_msgs::JointTrajectoryPoint point = trajectory.trajectory.points[i];
    std::vector<double> p;
    p.push_back(point.positions[0]);
    p.push_back(point.positions[1]);
    p.push_back(point.positions[2]);
    p.push_back(point.positions[3]);
    p.push_back(point.positions[4]);
    p.push_back(point.positions[5]);

    double next_time = point.time_from_start.toSec();

#ifdef MEASURE_TIMING
    clock_gettime(CLOCK_MONOTONIC, &start_time);
#endif
    hiwin_driver_->writeJointCommand(p, next_time - last_time);
#ifdef MEASURE_TIMING
    clock_gettime(CLOCK_MONOTONIC, &end_time);
    exec_ns = DIFF_NS(start_time, end_time);
    if (exec_ns > exec_max_ns)
    {
      exec_max_ns = exec_ns;
    }
    if (exec_ns < exec_min_ns)
    {
      exec_min_ns = exec_ns;
    }
#endif

    last_time = next_time;
  }

#ifdef MEASURE_TIMING
  ROS_DEBUG("Trajectory points number: %4d, exec(ns): %10u ... %10u", static_cast<int>(point_number), exec_min_ns,
            exec_max_ns);
#endif
}

void HardwareInterface::abortMotion()
{
  ROS_DEBUG("Abort motion");
  hiwin_driver_->motionAbort();
}

}  // namespace hiwin_driver

PLUGINLIB_EXPORT_CLASS(hiwin_driver::HardwareInterface, hardware_interface::RobotHW);
