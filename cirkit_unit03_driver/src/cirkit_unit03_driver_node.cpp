/******************************************************
 * This is CIR-KIT 3rd robot control driver.
 * Author : Arita Yuta(Kyutech)
 ******************************************************/
#include "cirkit_unit03_driver.hpp"

#include <string>
#include <utility>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cirkit_unit03_driver_node");
  ROS_INFO("cirkit unit03 robot driver for ROS.");

  ros::NodeHandle n {"~"};
  std::string imcs01_port {"/dev/urbtc0"};
  n.param<std::string>("imcs01_port", imcs01_port, imcs01_port);
  
  cirkit::CirkitUnit03Driver driver(std::move(imcs01_port), ros::NodeHandle {});
  driver.run();

  return 0;
}
