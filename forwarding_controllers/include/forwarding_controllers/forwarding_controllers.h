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

#ifndef FORWARDING_CONTROLLERS_FORWARDING_CONTROLLERS_H_
#define FORWARDING_CONTROLLERS_FORWARDING_CONTROLLERS_H_

#include <ros/ros.h>

#include <controller_interface/controller.h>

#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <industrial_msgs/CmdJointTrajectory.h>
#include <industrial_msgs/StopMotion.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <forwarding_controllers/joint_trajectory_interface.h>

namespace forwarding_controllers
{

/**
 * @class JointTrajectoryController
 * @brief A controller to manage joint trajectory execution and feedback.
 *
 * This class handles trajectory commands, executes the motions, and provides
 * feedback on the current state of the joints.
 */
class JointTrajectoryController : public controller_interface::Controller<hardware_interface::JointTrajectoryInterface>
{
public:
  JointTrajectoryController() = default;
  virtual ~JointTrajectoryController() override = default;

  /**
   * @brief Initialize the controller with hardware and node handle.
   * @param hw Pointer to the hardware interface for joint trajectory control.
   * @param root_nh ROS node handle for general topics.
   * @param controller_nh ROS node handle for controller-specific parameters.
   * @return True if initialization succeeds, false otherwise.
   */
  bool init(hardware_interface::JointTrajectoryInterface* hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  /**
   * @brief Start the controller.
   * @param time Current time.
   */
  void starting(const ros::Time& time) override;

  /**
   * @brief Update the controller during each control loop iteration.
   * @param time Current time.
   * @param period Time duration since the last update.
   */
  void update(const ros::Time& time, const ros::Duration& period) override;

  /**
   * @brief Stop the controller.
   * @param time Current time.
   */
  void stopping(const ros::Time& /*time*/) override;

  /**
   * @brief Handle joint trajectory commands via service.
   * @param req The service request containing the trajectory command.
   * @param res The service response with success or failure code.
   * @return True if the command is handled successfully, false otherwise.
   */
  bool jointTrajectoryCB(industrial_msgs::CmdJointTrajectory::Request& req,
                         industrial_msgs::CmdJointTrajectory::Response& res);

  /**
   * @brief Handle joint trajectory commands via topic subscription.
   * @param msg The incoming trajectory message.
   */
  void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr& msg);

  /**
   * @brief Handle stop motion requests.
   * @param req The service request for stopping motion.
   * @param res The service response with success or failure code.
   * @return True if the stop request is handled successfully, false otherwise.
   */
  bool stopMotionCB(industrial_msgs::StopMotion::Request& req, industrial_msgs::StopMotion::Response& res);

private:
  hardware_interface::JointTrajectoryInterface* joint_trajectory_interface_;
  std::vector<hardware_interface::JointTrajectoryHandle> joints_;
  unsigned int num_hw_joints_;

  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::FollowJointTrajectoryFeedback>>
      pub_joint_control_state_;

  control_msgs::FollowJointTrajectoryGoal command_struct_;
  realtime_tools::RealtimeBuffer<control_msgs::FollowJointTrajectoryGoal> command_;

  ros::Subscriber sub_joint_trajectory_;
  ros::ServiceServer srv_joint_trajectory_;
  ros::ServiceServer srv_stop_motion_;

  double state_publisher_rate_;
  bool pub_time_initialized_;
  ros::Time last_state_publish_time_;
};

}  // namespace forwarding_controllers

#endif  // FORWARDING_CONTROLLERS_FORWARDING_CONTROLLERS_H_
