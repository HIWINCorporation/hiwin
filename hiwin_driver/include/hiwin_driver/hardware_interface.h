#ifndef HIWIN_DRIVER_HARDWARE_INTERFACE_H_
#define HIWIN_DRIVER_HARDWARE_INTERFACE_H_

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>

#include <forwarding_controllers/trajectory_interface.h>

#include <industrial_robot_status_interface/industrial_robot_status_interface.h>

#include <hiwin_robot_client_library/hiwin_driver.h>

namespace hiwin_driver
{

/*!
 * \brief The HardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class HardwareInterface : public hardware_interface::RobotHW
{
public:
  /*!
   * \brief Creates a new HardwareInterface object.
   */
  HardwareInterface();
  virtual ~HardwareInterface() = default;
  /*!
   * \brief Handles the setup functionality for the ROS interface. This includes parsing ROS
   * parameters, creating interfaces, starting the main driver and advertising ROS services.
   *
   * \param root_nh Root level ROS node handle
   * \param robot_hw_nh ROS node handle for the robot namespace
   *
   * \returns True, if the setup was performed successfully
   */
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /*!
   * \brief Read method of the control loop. Reads a RTDE package from the robot and handles and
   * publishes the information as needed.
   *
   * \param time Current time
   * \param period Duration of current control loop iteration
   */
  virtual void read(const ros::Time& time, const ros::Duration& period) override;
  /*!
   * \brief Write method of the control loop. Writes target joint positions to the robot to be read
   * by its URCaps program.
   *
   * \param time Current time
   * \param period Duration of current control loop iteration
   */
  virtual void write(const ros::Time& time, const ros::Duration& period) override;
  /*!
   * \brief Starts and stops controllers.
   *
   * \param start_list List of controllers to start
   * \param stop_list List of controllers to stop
   */
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list) override;
  /*!
   * \brief Checks if a reset of the ROS controllers is necessary.
   *
   * \returns Necessity of ROS controller reset
   */
  bool shouldResetControllers();

protected:
  void startJointInterpolation(const control_msgs::FollowJointTrajectoryGoal& trajectory);
  void cancelInterpolation();

  hardware_interface::JointStateInterface js_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;
  hardware_interface::JointTrajectoryInterface jnt_traj_interface_;

  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  std::vector<double> joint_position_command_;

  std::vector<double> target_joint_positions_;
  std::vector<double> target_joint_velocities_;

  std::atomic<bool> controller_reset_necessary_;

  industrial_robot_status_interface::RobotStatus robot_status_resource_{};
  industrial_robot_status_interface::IndustrialRobotStatusInterface robot_status_interface_{};

  std::string robot_ip_;
  std::unique_ptr<hrsdk::HIWINDriver> hiwin_driver_;
};

}  // namespace hiwin_driver

#endif  // HIWIN_DRIVER_HARDWARE_INTERFACE_H_
