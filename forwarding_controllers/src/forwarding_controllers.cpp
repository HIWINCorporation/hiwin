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
bool JointTrajectoryController::init(hardware_interface::TrajectoryInterface* hw, ros::NodeHandle& root_node_handle,
                                     ros::NodeHandle& controller_nh)
{
  if ((trajectory_interface_ = hw) == nullptr)
  {
    return false;
  }

  action_server_.reset(new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(
      root_node_handle, "joint_trajectory_action",
      std::bind(&JointTrajectoryController::executeCB, this, std::placeholders::_1), false));

  trajectory_interface_->registerDoneCallback(
      std::bind(&JointTrajectoryController::doneCB, this, std::placeholders::_1));

  action_server_->start();

  sub_cur_pos_ = root_node_handle.subscribe("joint_states", 1, &JointTrajectoryController::jointStateCB, this);
  sub_joint_trajectory_ =
      root_node_handle.subscribe("joint_path_command", 0, &JointTrajectoryController::jointTrajectoryCB, this);
  pub_joint_control_state_ =
      root_node_handle.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);

  srv_joint_trajectory_ =
      root_node_handle.advertiseService("joint_path_command", &JointTrajectoryController::jointTrajectoryCB, this);
  srv_stop_motion_ = root_node_handle.advertiseService("stop_motion", &JointTrajectoryController::stopMotionCB, this);

  return true;
}

void JointTrajectoryController::starting(const ros::Time& time)
{
}

void JointTrajectoryController::update(const ros::Time& time, const ros::Duration& period)
{
  if (action_server_->isActive())
  {
    control_msgs::FollowJointTrajectoryFeedback f = trajectory_interface_->getFeedback();
    action_server_->publishFeedback(f);
  }
}

void JointTrajectoryController::stopping(const ros::Time& /*time*/)
{
  if (action_server_->isActive())
  {
    trajectory_interface_->setCancel();

    control_msgs::FollowJointTrajectoryResult result;
    result.error_string = "Controller stopped.";
    result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
    action_server_->setAborted(result);
  }
}

void JointTrajectoryController::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
  if (!this->isRunning())
  {
    ROS_ERROR("Can't accept new action goals. Controller is not running.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    action_server_->setAborted(result);
    return;
  }

  if (!trajectory_interface_->setGoal(*goal))
  {
    ROS_ERROR("Trajectory goal is invalid.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    action_server_->setAborted(result);
    return;
  }
}

void JointTrajectoryController::doneCB(const hardware_interface::ExecutionState& state)
{
  control_msgs::FollowJointTrajectoryResult result;

  if (!action_server_->isActive())
  {
    return;
  }

  switch (state)
  {
    case hardware_interface::ExecutionState::ABORTED: {
      result.error_string = "Trajectory aborted by the robot. Something unexpected happened.";
      result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      action_server_->setAborted(result);
    }
    break;

    case hardware_interface::ExecutionState::PREEMPTED: {
      result.error_string = "Trajectory preempted. Possible reasons: user request | path tolerances fail.";
      result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      action_server_->setPreempted(result);
    }
    break;

    case hardware_interface::ExecutionState::SUCCESS: {
      result.error_string = "Trajectory execution successful";
      result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      action_server_->setSucceeded(result);
    }
    break;

    default:
      result.error_string = "Trajectory finished in unknown state.";
      result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      action_server_->setAborted(result);
      break;
  }
}

bool JointTrajectoryController::stopMotionCB(industrial_msgs::StopMotion::Request& req,
                                             industrial_msgs::StopMotion::Response& res)
{
  trajectory_interface_->setCancel();

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
  ROS_INFO("Receiving joint trajectory message");

  control_msgs::FollowJointTrajectoryFeedback control_state = trajectory_interface_->getFeedback();
  pub_joint_control_state_.publish(control_state);

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names.insert(goal.trajectory.joint_names.end(), msg->joint_names.begin(),
                                     msg->joint_names.end());
  goal.trajectory.points.insert(goal.trajectory.points.end(), msg->points.begin(), msg->points.end());

  trajectory_interface_->setGoal(goal);
}

void JointTrajectoryController::jointStateCB(const sensor_msgs::JointStateConstPtr& msg)
{
}

}  // namespace forwarding_controllers

PLUGINLIB_EXPORT_CLASS(forwarding_controllers::JointTrajectoryController, controller_interface::ControllerBase);
