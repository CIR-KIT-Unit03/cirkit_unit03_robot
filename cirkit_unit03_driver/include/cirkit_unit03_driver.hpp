/******************************************************
 * This is CIR-KIT 3rd robot control driver.
 * Author : Arita Yuta(Kyutech)
 ******************************************************/

// inclusive dependency
#include "ThirdRobotInterface/ThirdRobotInterface.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <tf/transform_broadcaster.h>

#include <string>

namespace cirkit {
class CirkitUnit03Driver {
public:
  CirkitUnit03Driver(const std::string&, const ros::NodeHandle&);
  ~CirkitUnit03Driver();
  void resetCommunication();
  void run();
private:
  // Callback function
  void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel);

  ros::NodeHandle nh_;
  ros::Rate rate_;
  // ROS topic trader
  ros::Publisher odom_pub_;
  ros::Publisher steer_pub_;
  ros::Subscriber cmd_vel_sub_;
  // cirkit unit03 interface object
  cirkit::ThirdRobotInterface cirkit_unit03_;
  // self member
  tf::TransformBroadcaster odom_broadcaster_;
  std::string imcs01_port_;
  ros::Time current_time_, last_time_;
  geometry_msgs::Twist steer_dir_;
};
}
