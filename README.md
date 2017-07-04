# cirkit_unit03_robot [![Build Status](https://travis-ci.org/CIR-KIT-Unit03/cirkit_unit03_robot.svg?branch=kinetic-devel)](https://travis-ci.org/CIR-KIT-Unit03/cirkit_unit03_robot) [![Slack](https://img.shields.io/badge/Slack-CIR--KIT-blue.svg)](http://cir-kit.slack.com/messages/unit03_robot)

## Summary
Robot packages for CIR-KIT-Unit03, for operating robot hardware.

For CIR-KIT-Unit03 instructions and tutorials, please see http://wiki.ros.org/Robots/CIR-KIT-Unit03.

- husky_bringup : Bringup launch files and scripts.
- husky_base : Hardware driver for communicating with the onboard MCU.

## Installation
#### **!! CAUTION !!  DO NOT INSTALL** this repository **ALONE**.  
We highly recommend install this repository with [cirkit_unit03_pkgs](https://github.com/CIR-KIT-Unit03/cirkit_unit03_pkgs).

#### For the developers only
##### 1. Create **catkinized**  workspace.
##### 2. Clone this repository.
```bash
$ cd <catkin_ws>/src
$ git clone https://github.com/CIR-KIT-Unit03/cirkit_unit03_robot.git
```
##### 3. Download required packages by wstool.
```bash
$ cd <catkin_ws>
$ wstool init src
$ wstool merge -t src src/cirkit_unit03_robot/cirkit_unit03_robot.rosinstall
$ wstool update -t src
```
##### 4. Download depended packages by rosdep.
```bash
$ cd <catkin_ws>
$ rosdep install -i -r -y --from-paths src --ignore-src
```
##### 5. Build packages, and set the path for the packages.
```bash
$ cd <catkin_ws>
$ catkin_make
$ source devel/setup.bash
```
