cmake_minimum_required(VERSION 2.8.3)
project(cirkit_unit03_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  roslaunch
  sensor_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs tf
)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cirkit_unit03_driver_node
  src/cirkit_unit03_driver_node.cpp
  src/cirkit_unit03_driver.cpp
  src/ThirdRobotInterface/ThirdRobotInterface.cpp
)
add_dependencies(cirkit_unit03_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(cirkit_unit03_driver_node ${catkin_LIBRARIES})

roslaunch_add_file_check(test/cirkit_unit03_launch.test.xml)
