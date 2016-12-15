#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class CirkitUnit03HardwareInterface
  : public hardware_interface::RobotHW
{
public:
  CirkitUnit03HardwareInterface(const std::string& imcs01_port, const ros::NodeHandle& nh);
  void read();
  void write();
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

  ros::Publisher steer_cmd_publisher_;
};

CirkitUnit03HardwareInterface::CirkitUnit03HardwareInterface(const std::string& imcs01_port, const ros::NodeHandle& nh)
  : nh_(nh),
    steer_cmd_publisher_(nh_.advertise<geometry_msgs::Twist>("/steer_ctrl", 1))
{
  ros::NodeHandle n("~");
  std::string front_steer_joint_name("front_steer");
  std::string rear_wheel_joint_name("rear_wheel");

  hardware_interface::JointStateHandle front_steer_state_handle(front_steer_joint_name, &front_steer_pos_, &front_steer_vel_, &front_steer_eff_);
  front_steer_jnt_state_interface_.registerHandle(front_steer_state_handle);
  hardware_interface::JointHandle front_steer_pos_cmd_handle(front_steer_jnt_state_interface_.getHandle(front_steer_joint_name), &front_steer_pos_cmd_);
  front_steer_jnt_pos_cmd_interface_.registerHandle(front_steer_pos_cmd_handle);

  hardware_interface::JointStateHandle rear_wheel_state_handle(rear_wheel_joint_name, &rear_wheel_pos_, &rear_wheel_vel_, &rear_wheel_eff_);
  rear_wheel_jnt_state_interface_.registerHandle(rear_wheel_state_handle);
  hardware_interface::JointHandle rear_wheel_vel_cmd_handle(rear_wheel_jnt_state_interface_.getHandle(rear_wheel_joint_name), &rear_wheel_vel_cmd_);
  rear_wheel_jnt_vel_cmd_interface_.registerHandle(rear_wheel_vel_cmd_handle);
}

void CirkitUnit03HardwareInterface::read()
{
  
}

void CirkitUnit03HardwareInterface::write()
{
  
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cirkit_unit03_control");

  ros::NodeHandle nh;
  std::string imcs01_port("/dev/urbtc0");
  CirkitUnit03HardwareInterface cirkit_unit03_hardware_interface(imcs01_port, nh);
  controller_manager::ControllerManager cm(&cirkit_unit03_hardware_interface, nh);

  ros::Rate rate(1.0 / cirkit_unit03_hardware.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok()){
    ros::Time now = cirkit_unit03_hardware_interface.getTime();
    ros::Duration dt = cirkit_unit03_hardware_interface.getPeriod();

    cirkit_unit03_hardware_interface.read();
    cm.update(now, dt);
    cirkit_unit03_hardware_interface.write();
    rate.sleep();
  }
  spinner.stop();

  return 0;
}












