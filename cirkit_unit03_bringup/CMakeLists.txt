cmake_minimum_required(VERSION 2.8.3)
project(cirkit_unit03_bringup)
find_package(catkin REQUIRED COMPONENTS roslaunch)

roslaunch_add_file_check(test/cirkit_unit03_bringup_launch.test.xml)
roslaunch_add_file_check(test/sensor_bringup_launch.test.xml)
