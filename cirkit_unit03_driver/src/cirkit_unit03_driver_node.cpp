/******************************************************
 * This is CIR-KIT 3rd robot control driver.
 * Author : Arita Yuta(Kyutech)
 ******************************************************/
#include "third_robot_driver.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Third_robot_driver_node");
  ROS_INFO("Third robot driver for ROS.");

  ros::NodeHandle nh;
  
  cirkit::ThirdRobotDriver driver(nh);
  driver.run();

  return 0;
}
