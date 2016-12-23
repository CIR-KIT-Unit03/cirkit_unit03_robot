#ifndef IXIS_IMCS01_DRIVER_H
#define IXIS_IMCS01_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <vector>
#include <string>
#include <mutex>

#include <termios.h>
#include "urbtc.h"
#include "urobotc.h"

#define JOINT_INDEX_REAR_RIGHT 0
#define JOINT_INDEX_REAR_LEFT 1
#define JOINT_INDEX_FRONT 2

template<typename N, typename M>
inline double MIN(const N& a, const M& b)
{
  return a < b ? a : b;
}

template<typename N, typename M>
inline double MAX(const N& a, const M& b)
{
  return a > b ? a : b;
}

template<typename T>
inline double NORMALIZE(const T& z)
{
  return atan2(sin(z), cos(z));
}

enum class RunningState {
    FORWARD,
    FORWARD_STOP,
    BACK,
    BACK_STOP,
    OTHERWISE
};

// RunningMode means switch of Handle of car.
enum class RunningMode {
  FORWARD,
  BACK
};


class IxisImcs01Driver
{
public:
  IxisImcs01Driver(std::string port_name);
  ~IxisImcs01Driver();
  int update();
  sensor_msgs::JointState getJointState();
  int controlRearWheel(double rear_speed);
protected:
  int openPort(std::string port_name);
  int closePort();
  int setImcs01();
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
  termios oldtio_imcs01_;
  termios newtio_imcs01_;
  std::mutex communication_mutex_;
};

#endif /* IXIS_IMCS01_DRIVER_H */
