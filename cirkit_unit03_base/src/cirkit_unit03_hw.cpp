#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class CirkitUnit03HW
  : public hardware_interface::RobotHW
{
public:
  CirkitUnit03HW();
  void register_interface();
  void read(ros::Time time, ros::Duration period);
  void write(ros::Time time, ros::Duration period);
protected:
  ros::NodeHandle nh_;

  hardware_interface::JointStateInterface front_steer_jnt_state_interface_;
  hardware_interface::PositionJointInterface front_steer_jnt_pos_cmd_interface_;
  double front_steer_pos_;
  double front_steer_vel_;
  double front_steer_eff_;
  double front_steer_pos_cmd_;

  hardware_interface::JointStateInterface rear_wheel_jnt_state_interface_;
  hardware_interface::VelocityJointInterface rear_wheel_jnt_vel_cmd_interface_;
  double rear_wheel_pos_;
  double rear_wheel_vel_;
  double rear_wheel_eff_;
  double rear_wheel_vel_cmd_;
};

CirkitUnit03HW::CirkitUnit03HW()
{
  ros::NodeHandle n("~");
  std::vector<std::string> front_steer_name;
  std::vector<std::string> rear_wheel_name;

  hardware_interface::JointStateHandle front_steer_state_handle("front_steer", &front_steer_pos_, &front_steer_vel_, &front_steer_eff_);
  front_steer_jnt_state_interface_.registerHandle(front_steer_state_handle);
  hardware_interface::JointHandle front_steer_pos_cmd_handle(front_steer_jnt_state_interface_.getHandle("front_steer"), &front_steer_pos_cmd_);
  front_steer_jnt_pos_cmd_interface_.registerHandle(front_steer_pos_cmd_handle);

  hardware_interface::JointStateHandle rear_wheel_state_handle("rear_wheel", &rear_wheel_pos_, &rear_wheel_vel_, &rear_wheel_eff_);
  rear_wheel_jnt_state_interface_.registerHandle(rear_wheel_state_handle);
  hardware_interface::JointHandle rear_wheel_vel_cmd_handle(rear_wheel_jnt_state_interface_.getHandle("rear_wheel"), &rear_wheel_vel_cmd_);
  rear_wheel_jnt_vel_cmd_interface_.registerHandle(rear_wheel_vel_cmd_handle);
}

void CirkitUnit03HW::read()
{
}

void CirkitUnit03HW::write()
{
}
















