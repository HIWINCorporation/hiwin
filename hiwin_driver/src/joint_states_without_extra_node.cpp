#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <vector>
#include <algorithm>

class JointStateWithoutExtra
{
public:
  JointStateWithoutExtra(ros::NodeHandle& nh)
  {
    if (!nh.getParam("publish_joint_names", valid_joints_))
    {
      if (!nh.getParam("hiwin_hardware_interface/joints", valid_joints_))
      {
        ROS_ERROR("Failed to initialize joint_names.");
      }
    }

    joint_states_sub_ = nh.subscribe("/joint_states", 10, &JointStateWithoutExtra::jointStateCallback, this);

    filtered_joint_states_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states_without_extra", 10);
  }

private:
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    sensor_msgs::JointState filtered_msg;
    filtered_msg.header = msg->header;

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if (std::find(valid_joints_.begin(), valid_joints_.end(), msg->name[i]) != valid_joints_.end())
      {
        filtered_msg.name.push_back(msg->name[i]);
        filtered_msg.position.push_back(msg->position[i]);
        filtered_msg.velocity.push_back(msg->velocity[i]);
        filtered_msg.effort.push_back(msg->effort[i]);
      }
    }

    filtered_joint_states_pub_.publish(filtered_msg);
  }

  ros::Subscriber joint_states_sub_;
  ros::Publisher filtered_joint_states_pub_;
  std::vector<std::string> valid_joints_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_states_without_extra");
  ros::NodeHandle nh;

  JointStateWithoutExtra filter_node(nh);

  ros::spin();

  return 0;
}
