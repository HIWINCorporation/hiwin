#ifndef FORWARDING_CONTROLLERS_FORWARDING_CONTROLLERS_H_
#define FORWARDING_CONTROLLERS_FORWARDING_CONTROLLERS_H_

#include <ros/ros.h>

#include <controller_interface/controller.h>

#include <actionlib/server/simple_action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <industrial_msgs/CmdJointTrajectory.h>
#include <industrial_msgs/StopMotion.h>

#include <forwarding_controllers/trajectory_interface.h>

namespace forwarding_controllers
{
class JointTrajectoryController : public controller_interface::Controller<hardware_interface::TrajectoryInterface>
{
public:
  JointTrajectoryController() = default;
  virtual ~JointTrajectoryController() override{};

  bool init(hardware_interface::TrajectoryInterface* hw, ros::NodeHandle& root_node_handle,
            ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& /*time*/) override;

  void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
  void doneCB(const hardware_interface::ExecutionState& state);
  bool jointTrajectoryCB(industrial_msgs::CmdJointTrajectory::Request& req,
                         industrial_msgs::CmdJointTrajectory::Response& res);
  bool stopMotionCB(industrial_msgs::StopMotion::Request& req, industrial_msgs::StopMotion::Response& res);
  void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr& msg);
  void jointStateCB(const sensor_msgs::JointStateConstPtr& msg);

private:
  hardware_interface::TrajectoryInterface* trajectory_interface_;
  std::unique_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>> action_server_;

  ros::Subscriber sub_cur_pos_;              // handle for joint-state topic subscription
  ros::Subscriber sub_joint_trajectory_;     // handle for joint-trajectory topic subscription
  ros::ServiceServer srv_joint_trajectory_;  // handle for joint-trajectory service
  ros::ServiceServer srv_stop_motion_;       // handle for stop_motion service
  ros::Publisher pub_joint_control_state_;
};

}  // namespace forwarding_controllers

#endif  // FORWARDING_CONTROLLERS_FORWARDING_CONTROLLERS_H_
