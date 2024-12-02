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

#include <pluginlib/class_list_macros.hpp>

#include <forwarding_controllers/forwarding_controllers.h>

namespace forwarding_controllers
{
bool JointTrajectoryController::init(hardware_interface::JointTrajectoryInterface* hw, ros::NodeHandle& root_nh,
                                     ros::NodeHandle& controller_nh)
{
  if ((joint_trajectory_interface_ = hw) == nullptr)
  {
    ROS_ERROR("JointTrajectoryController: Could not get JointTrajectoryInterface from hardware");
    return false;
  }

  // List of joints to be published
  std::vector<std::string> joint_names;

  if (controller_nh.getParam("joints", joint_names))
  {
    ROS_INFO("Joints parameter specified, publishing specified joints in desired order.");
  }
  else
  {
    ROS_ERROR_STREAM("Could not find 'joints' parameter (namespace: " << controller_nh.getNamespace() << ").");
    return false;
  }

  num_hw_joints_ = joint_names.size();
  for (unsigned int i = 0; i < num_hw_joints_; i++)
  {
    ROS_DEBUG("Got joint %s", joint_names[i].c_str());
  }

  // get publishing period
  state_publisher_rate_ = 50.0;
  controller_nh.getParam("state_publish_rate", state_publisher_rate_);

  // Initialize members
  joints_.resize(num_hw_joints_);
  for (unsigned int i = 0; i < num_hw_joints_; ++i)
  {
    // Joint handle
    try
    {
      joints_[i] = hw->getHandle(joint_names[i]);
    }
    catch (...)
    {
      ROS_ERROR_STREAM("Could not find joint '" << joint_names[i] << "' in '" << this->getHardwareInterfaceType()
                                                << "'.");
      return false;
    }
  }

  pub_joint_control_state_ =
      std::make_unique<realtime_tools::RealtimePublisher<control_msgs::FollowJointTrajectoryFeedback>>(root_nh,
                                                                                                       "feedback_"
                                                                                                       "states",
                                                                                                       4);

  sub_joint_trajectory_ =
      root_nh.subscribe("joint_path_command", 0, &JointTrajectoryController::jointTrajectoryCB, this);

  srv_joint_trajectory_ =
      root_nh.advertiseService("joint_path_command", &JointTrajectoryController::jointTrajectoryCB, this);

  srv_stop_motion_ = root_nh.advertiseService("stop_motion", &JointTrajectoryController::stopMotionCB, this);

  pub_time_initialized_ = false;

  return true;
}

void JointTrajectoryController::starting(const ros::Time& time)
{
  if (!pub_time_initialized_)
  {
    try
    {
      last_state_publish_time_ = time - ros::Duration(1.001 / state_publisher_rate_);  // ensure publish on first cycle
    }
    catch (std::runtime_error& ex)
    {  // negative ros::Time is not allowed
      last_state_publish_time_ = ros::Time::MIN;
    }
    pub_time_initialized_ = true;
  }

  command_.initRT(command_struct_);
}

void JointTrajectoryController::update(const ros::Time& time, const ros::Duration& period)
{
  command_struct_ = *(command_.readFromRT());

  for (unsigned int i = 0; i < num_hw_joints_; i++)
  {
    // joints_[i].getPosition();
  }

  control_msgs::FollowJointTrajectoryFeedback f = joint_trajectory_interface_->getFeedback();

  if (state_publisher_rate_ > 0.0 && last_state_publish_time_ + ros::Duration(1.0 / state_publisher_rate_) < time)
  {
    if (pub_joint_control_state_ && pub_joint_control_state_->trylock())
    {
      last_state_publish_time_ += ros::Duration(1.0 / state_publisher_rate_);
      pub_joint_control_state_->msg_ = f;
      pub_joint_control_state_->msg_.header.stamp = time;
      pub_joint_control_state_->unlockAndPublish();
    }
  }

  for (unsigned int i = 0; i < num_hw_joints_; i++)
  {
    // joints_[i].setCommand();
  }
}

void JointTrajectoryController::stopping(const ros::Time& /*time*/)
{
  joint_trajectory_interface_->setCancel();
}

bool JointTrajectoryController::stopMotionCB(industrial_msgs::StopMotion::Request& req,
                                             industrial_msgs::StopMotion::Response& res)
{
  joint_trajectory_interface_->setCancel();
  res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;
  return true;
}

bool JointTrajectoryController::jointTrajectoryCB(industrial_msgs::CmdJointTrajectory::Request& req,
                                                  industrial_msgs::CmdJointTrajectory::Response& res)
{
  trajectory_msgs::JointTrajectoryPtr traj_ptr(new trajectory_msgs::JointTrajectory);
  *traj_ptr = req.trajectory;
  this->jointTrajectoryCB(traj_ptr);

  res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;
  return true;
}

void JointTrajectoryController::jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr& msg)
{
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names.insert(goal.trajectory.joint_names.end(), msg->joint_names.begin(),
                                     msg->joint_names.end());
  goal.trajectory.points.insert(goal.trajectory.points.end(), msg->points.begin(), msg->points.end());

  if (goal.trajectory.points.empty())
  {
    joint_trajectory_interface_->setCancel();
    return;
  }

  command_.writeFromNonRT(goal);
  joint_trajectory_interface_->setGoal(goal);
}

}  // namespace forwarding_controllers

PLUGINLIB_EXPORT_CLASS(forwarding_controllers::JointTrajectoryController, controller_interface::ControllerBase);
