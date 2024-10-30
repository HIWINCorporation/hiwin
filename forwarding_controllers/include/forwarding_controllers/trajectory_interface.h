/*
 * Copyright 2024, HIWIN Technologies Corp. (trimming)
 *
 * Copyright 2019, FZI Forschungszentrum Informatik (refactor)
 *
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
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
 */

#ifndef FORWARDING_CONTROLLERS_TRAJECTORY_INTERFACE_H_
#define FORWARDING_CONTROLLERS_TRAJECTORY_INTERFACE_H_

#include <functional>

#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>

#include <hardware_interface/hardware_interface.h>

namespace hardware_interface
{

enum class ExecutionState
{
  SUCCESS = 0,
  PREEMPTED = -1,
  ABORTED = -2,
};

class TrajectoryInterface : public hardware_interface::HardwareInterface
{
public:
  void registerGoalCallback(std::function<void(const control_msgs::FollowJointTrajectoryGoal&)> f)
  {
    goal_callback_ = f;
  }

  void registerCancelCallback(std::function<void()> f)
  {
    cancel_callback_ = f;
  }

  void registerDoneCallback(std::function<void(const ExecutionState&)> f)
  {
    done_callback_ = f;
  }

  void setFeedback(control_msgs::FollowJointTrajectoryFeedback feedback)
  {
    feedback_ = feedback;
  }

  control_msgs::FollowJointTrajectoryFeedback getFeedback() const
  {
    return feedback_;
  }

  bool setGoal(control_msgs::FollowJointTrajectoryGoal goal)
  {
    if (goal_callback_ != nullptr)
    {
      goal_callback_(goal);
      return true;
    }
    return false;
  }

  void setCancel()
  {
    if (cancel_callback_ != nullptr)
      cancel_callback_();
  };

  void setDone(const ExecutionState& state)
  {
    if (done_callback_ != nullptr)
      done_callback_(state);
  }

private:
  std::function<void(const control_msgs::FollowJointTrajectoryGoal&)> goal_callback_;
  std::function<void()> cancel_callback_;
  std::function<void(const ExecutionState&)> done_callback_;
  control_msgs::FollowJointTrajectoryFeedback feedback_;
};

using JointTrajectoryInterface = TrajectoryInterface;

}  // namespace hardware_interface

#endif  // FORWARDING_CONTROLLERS_TRAJECTORY_INTERFACE_H_
