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
