#ifndef IXIS_IMCS01_DRIVER_H
#define IXIS_IMCS01_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <vector>
#include <string>

enum RunningState {
    FORWARD,
    FORWARD_STOP,
    BACK,
    BACK_STOP,
    OTHERWISE
};

// RunningMode means switch of Handle of car.
enum RunningMode {
  FORWARD,
  BACK
};

class IxisImcs01Driver
{
public:
  IxisImcs01Driver(std::string port_name);
  int update();
  sensor_msgs::JointState getJointState();
  int controlRearWheel(double rear_speed);
protected:
  int openPort(std::string port_name);
  int closePort();
  int parseEncoderTime();
  int parseFrontEncoderCounts();
  int parseRearEncoderCounts();
  int writeOffsetCmd(RunningMode mode,
                     unsigned short duty);
  struct uin received_data_;
  std::vector<int> rear_last_encoder_counts_;
  std::vector<int> rear_delta_encoder_counts_;
  double delta_encoder_time_;
  double last_encoder_time_;
  sensor_msgs::JointState state_;
  int imcs01_fd_;
  RunningState running_state_;
  struct ccmd cmd_ccmd_;
  struct uin cmd_uin_;
};

#endif /* IXIS_IMCS01_DRIVER_H */
