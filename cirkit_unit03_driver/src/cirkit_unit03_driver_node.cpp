/******************************************************
 * This is CIR-KIT 3rd robot control driver.
 * Author : Arita Yuta(Kyutech)
 ******************************************************/
#include "cirkit_unit03_driver.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cirkit_unit03_driver_node");
  ROS_INFO("cirkit unit03 robot driver for ROS.");

  ros::NodeHandle nh;
  
  cirkit::ThirdRobotDriver driver(nh);
  driver.run();

  return 0;
}
