#include <pluginlib/class_list_macros.hpp>

#include <hiwin_driver/hardware_interface.h>

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
  , target_joint_positions_({ 0, 0, 0, 0, 0, 0 })
  , target_joint_velocities_({ 0, 0, 0, 0, 0, 0 })
{
}

bool HardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  // Names of the joints. Usually, this is given in the controller config file.
  if (!robot_hw_nh.getParam("joints", joint_names_))
  {
    ROS_ERROR_STREAM("Cannot find required parameter " << robot_hw_nh.resolveName("joints")
                                                       << " on the parameter server.");
    throw std::runtime_error(
        "Cannot find required parameter "
        "'controller_joint_names' on the parameter server.");
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
  }

  robot_status_interface_.registerHandle(industrial_robot_status_interface::IndustrialRobotStatusHandle(
      "industrial_robot_status_handle", robot_status_resource_));

  jnt_traj_interface_.registerGoalCallback(
      std::bind(&HardwareInterface::startJointInterpolation, this, std::placeholders::_1));
  jnt_traj_interface_.registerCancelCallback(std::bind(&HardwareInterface::cancelInterpolation, this));

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&robot_status_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&jnt_traj_interface_);

  return true;
}

void HardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  // set defaults
  robot_status_resource_.mode = RobotMode::UNKNOWN;
  robot_status_resource_.e_stopped = TriState::UNKNOWN;
  robot_status_resource_.drives_powered = TriState::UNKNOWN;
  robot_status_resource_.motion_possible = TriState::UNKNOWN;
  robot_status_resource_.in_motion = TriState::UNKNOWN;
  robot_status_resource_.in_error = TriState::UNKNOWN;
  robot_status_resource_.error_code = 0;

  // robot connect mock
  for (size_t i = 0; i < 6; i++)
  {
    joint_positions_[i] = target_joint_positions_[i];
    joint_velocities_[i] = target_joint_velocities_[i];
  }

  control_msgs::FollowJointTrajectoryFeedback feedback = control_msgs::FollowJointTrajectoryFeedback();
  for (size_t i = 0; i < 6; i++)
  {
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
  size_t point_number = trajectory.trajectory.points.size();
  ROS_DEBUG("Starting joint-based trajectory forward");

  for (size_t i = 0; i < point_number; i++)
  {
    target_joint_positions_[0] = trajectory.trajectory.points[i].positions[0];
    target_joint_positions_[1] = trajectory.trajectory.points[i].positions[1];
    target_joint_positions_[2] = trajectory.trajectory.points[i].positions[2];
    target_joint_positions_[3] = trajectory.trajectory.points[i].positions[3];
    target_joint_positions_[4] = trajectory.trajectory.points[i].positions[4];
    target_joint_positions_[5] = trajectory.trajectory.points[i].positions[5];

    target_joint_velocities_[0] = trajectory.trajectory.points[i].velocities[0];
    target_joint_velocities_[1] = trajectory.trajectory.points[i].velocities[1];
    target_joint_velocities_[2] = trajectory.trajectory.points[i].velocities[2];
    target_joint_velocities_[3] = trajectory.trajectory.points[i].velocities[3];
    target_joint_velocities_[4] = trajectory.trajectory.points[i].velocities[4];
    target_joint_velocities_[5] = trajectory.trajectory.points[i].velocities[5];
  }

  // robot connect mock
  hardware_interface::ExecutionState final_state;
  final_state = hardware_interface::ExecutionState::SUCCESS;
  jnt_traj_interface_.setDone(final_state);
}

void HardwareInterface::cancelInterpolation()
{
  ROS_DEBUG("Cancelling Trajectory");
}

}  // namespace hiwin_driver

PLUGINLIB_EXPORT_CLASS(hiwin_driver::HardwareInterface, hardware_interface::RobotHW);
