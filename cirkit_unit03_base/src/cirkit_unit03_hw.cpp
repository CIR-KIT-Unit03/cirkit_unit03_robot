#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "ixis_imcs01_driver/ixis_imcs01_driver.h"

class CirkitUnit03HardwareInterface
  : public hardware_interface::RobotHW
{
public:
  CirkitUnit03HardwareInterface(const std::string& imcs01_port, const ros::NodeHandle& nh);
  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const {return ros::Duration(0.01); }
  void read();
  void write();
protected:
  void publishSteer(double angle_cmd);
  ros::NodeHandle nh_;

  //hardware_interface::JointStateInterface front_steer_jnt_state_interface_;
  hardware_interface::PositionJointInterface front_steer_jnt_pos_cmd_interface_;
  double front_steer_pos_;
  double front_steer_vel_;
  double front_steer_eff_;
  double front_steer_pos_cmd_;

  //hardware_interface::JointStateInterface rear_wheel_jnt_state_interface_;
  hardware_interface::VelocityJointInterface rear_wheel_jnt_vel_cmd_interface_;
  double rear_wheel_pos_;
  double rear_wheel_vel_;
  double rear_wheel_eff_;
  double rear_wheel_vel_cmd_;

  hardware_interface::JointStateInterface joint_state_interface_;

  ros::Publisher steer_cmd_publisher_;
  IxisImcs01Driver ixis_imcs01_driver_;
};

CirkitUnit03HardwareInterface::CirkitUnit03HardwareInterface(const std::string& imcs01_port, const ros::NodeHandle& nh)
  : nh_(nh),
    ixis_imcs01_driver_(imcs01_port),
    steer_cmd_publisher_(nh_.advertise<geometry_msgs::Twist>("/steer_ctrl", 1))
{
  ros::NodeHandle n("~");
  std::string front_steer_joint_name("front_steer_joint");
  std::string rear_wheel_joint_name("rear_wheel_joint");

  hardware_interface::JointStateHandle front_steer_state_handle(front_steer_joint_name, &front_steer_pos_, &front_steer_vel_, &front_steer_eff_);
  joint_state_interface_.registerHandle(front_steer_state_handle);
  hardware_interface::JointHandle front_steer_pos_cmd_handle(joint_state_interface_.getHandle(front_steer_joint_name), &front_steer_pos_cmd_);
  front_steer_jnt_pos_cmd_interface_.registerHandle(front_steer_pos_cmd_handle);

  hardware_interface::JointStateHandle rear_wheel_state_handle(rear_wheel_joint_name, &rear_wheel_pos_, &rear_wheel_vel_, &rear_wheel_eff_);
  joint_state_interface_.registerHandle(rear_wheel_state_handle);
  hardware_interface::JointHandle rear_wheel_vel_cmd_handle(joint_state_interface_.getHandle(rear_wheel_joint_name), &rear_wheel_vel_cmd_);
  rear_wheel_jnt_vel_cmd_interface_.registerHandle(rear_wheel_vel_cmd_handle);

  registerInterface(&front_steer_jnt_pos_cmd_interface_);
  registerInterface(&rear_wheel_jnt_vel_cmd_interface_);
  registerInterface(&joint_state_interface_);
}

void CirkitUnit03HardwareInterface::read()
{
  ixis_imcs01_driver_.update(); // reading from imcs01.
  sensor_msgs::JointState joints_state = ixis_imcs01_driver_.getJointState();
  front_steer_pos_ = joints_state.position[JOINT_INDEX_FRONT];
  rear_wheel_pos_ = (joints_state.position[JOINT_INDEX_REAR_RIGHT]
                     + joints_state.position[JOINT_INDEX_REAR_LEFT]) / 2.0;
  rear_wheel_vel_ = (joints_state.velocity[JOINT_INDEX_REAR_RIGHT]
                     + joints_state.velocity[JOINT_INDEX_REAR_LEFT]) / 2.0;
}

void CirkitUnit03HardwareInterface::write()
{
  ixis_imcs01_driver_.controlRearWheel(rear_wheel_vel_cmd_);
  this->publishSteer(front_steer_pos_cmd_);
}

void CirkitUnit03HardwareInterface::publishSteer(double angle_cmd)
{
  double limited_angle = angle_cmd * 180.0 / M_PI;
  limited_angle = MAX(limited_angle, -60.0);
  limited_angle = MIN(limited_angle, 60);
  geometry_msgs::Twist steer;
  double angle_diff = limited_angle - (front_steer_pos_*180.0/M_PI); // angle_diff assume [deg]
  if(angle_diff > 0){
    steer.angular.z = 1;
    steer.angular.x = fabs(angle_diff);
  }else if(angle_diff < 0){
    steer.angular.z = -1;
    steer.angular.x = fabs(angle_diff);
  }else{
    steer.angular.z = 0;
    steer.angular.x = 0;
  }
  steer_cmd_publisher_.publish(steer);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cirkit_unit03_control");

  ros::NodeHandle nh;
  std::string imcs01_port("/dev/urbtc0");
  CirkitUnit03HardwareInterface cirkit_unit03_hardware_interface(imcs01_port, nh);
  controller_manager::ControllerManager cm(&cirkit_unit03_hardware_interface, nh);

  ros::Rate rate(1.0 / cirkit_unit03_hardware_interface.getPeriod().toSec());
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
