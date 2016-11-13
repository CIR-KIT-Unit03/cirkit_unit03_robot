cmake_minimum_required(VERSION 2.8.3)
project(cirkit_unit03_driver)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  tf
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(third_robot_driver_node 
  src/third_robot_driver_node.cpp
  src/third_robot_driver.cpp
  src/ThirdRobotInterface/ThirdRobotInterface.cpp
)
add_dependencies(third_robot_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(third_robot_driver_node
  ${catkin_LIBRARIES}
)

