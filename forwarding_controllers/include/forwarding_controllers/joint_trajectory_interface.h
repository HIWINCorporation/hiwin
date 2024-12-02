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

#ifndef FORWARDING_CONTROLLERS_JOINT_TRAJECTORY_INTERFACE_H_
#define FORWARDING_CONTROLLERS_JOINT_TRAJECTORY_INTERFACE_H_

#include <functional>

#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace hardware_interface
{

/**
 * @class JointTrajectoryHandle
 * @brief A handle for managing joint trajectory commands and states.
 *
 * This class inherits from JointHandle and provides an interface for interacting
 * with individual joints in a trajectory context.
 */
class JointTrajectoryHandle : public hardware_interface::JointHandle
{
public:
  /**
   * @brief Default constructor. Initializes an empty JointTrajectoryHandle.
   */
  JointTrajectoryHandle() : hardware_interface::JointHandle()
  {
  }

  /**
   * @brief Constructor with joint state and command references.
   * @param js Reference to the JointStateHandle.
   * @param cmd Pointer to the command variable.
   */
  JointTrajectoryHandle(const hardware_interface::JointStateHandle& js, double* cmd)
    : hardware_interface::JointHandle(js, cmd)
  {
  }
};

/**
 * @class JointTrajectoryInterface
 * @brief Interface for managing and executing joint trajectory commands.
 *
 * This interface allows for registering callbacks for goal setting and canceling,
 * handling feedback, and managing hardware resources for joint trajectories.
 */
class JointTrajectoryInterface
  : public hardware_interface::HardwareResourceManager<JointTrajectoryHandle, hardware_interface::ClaimResources>
{
public:
  /**
   * @brief Register a callback to handle incoming trajectory goals.
   * @param f The callback function to be invoked with the trajectory goal.
   */
  void registerGoalCallback(std::function<void(const control_msgs::FollowJointTrajectoryGoal&)> f)
  {
    goal_callback_ = f;
  }

  /**
   * @brief Set a new trajectory goal and trigger the registered callback.
   * @param goal The trajectory goal to be set.
   * @return True if the goal callback is registered and successfully invoked; otherwise, false.
   */
  bool setGoal(control_msgs::FollowJointTrajectoryGoal goal)
  {
    if (goal_callback_ != nullptr)
    {
      goal_callback_(goal);
      return true;
    }
    return false;
  }

  /**
   * @brief Register a callback to handle trajectory cancel requests.
   * @param f The callback function to be invoked upon cancellation.
   */
  void registerCancelCallback(std::function<void()> f)
  {
    cancel_callback_ = f;
  }

  /**
   * @brief Trigger the registered cancel callback.
   */
  void setCancel()
  {
    if (cancel_callback_ != nullptr)
      cancel_callback_();
  }

  /**
   * @brief Set trajectory feedback data.
   * @param feedback The feedback data to be stored.
   */
  void setFeedback(control_msgs::FollowJointTrajectoryFeedback feedback)
  {
    feedback_ = feedback;
  }

  /**
   * @brief Retrieve the current trajectory feedback data.
   * @return The stored feedback data.
   */
  control_msgs::FollowJointTrajectoryFeedback getFeedback() const
  {
    return feedback_;
  }

private:
  std::function<void(const control_msgs::FollowJointTrajectoryGoal&)> goal_callback_;
  std::function<void()> cancel_callback_;
  control_msgs::FollowJointTrajectoryFeedback feedback_;
};

}  // namespace hardware_interface

#endif  // FORWARDING_CONTROLLERS_JOINT_TRAJECTORY_INTERFACE_H_
