#include "ixis_imcs01_driver/ixis_imcs01_driver.h"


#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>

#include <ros/ros.h>

static constexpr int ROBOT_MAX_ENCODER_COUNTS {65535};
static constexpr double MAX_LIN_VEL {1.11}; // 1.11[m/s] => 4.0[km/h]

IxisImcs01Driver::IxisImcs01Driver(std::string port_name)
  : imcs01_fd_(-1)
{
  rear_last_encoder_counts_.resize(2);
  rear_delta_encoder_counts_.resize(2);
  state_.position.resize(3);
  state_.velocity.resize(3);
  state_.effort.resize(3);
  state_.name.resize(3);
  for (size_t i; i < 3; ++i) {
    rear_last_encoder_counts_[i] = 0;
    rear_delta_encoder_counts_[i] = -1;
    state_.position[i] = 0.0;
    state_.velocity[i] = 0.0;
    state_.effort[i] = 0;
  }
  state_.name[0] = "rear_right";
  state_.name[1] = "rear_left";
  state_.name[2] = "front";
  this->openPort(port_name);
}

IxisImcs01Driver::~IxisImcs01Driver()
{
  this->closePort();
}

int IxisImcs01Driver::openPort(std::string port_name)
{
  imcs01_fd_ = open(port_name.c_str(), O_RDWR);
  if(imcs01_fd_ < 0){
    ROS_ERROR_STREAM("iMCs01 Open Error : " << port_name);
    exit(-1);
  }else{
    this->setImcs01();
  }
  return 0;
}

int IxisImcs01Driver::closePort()
{
  cmd_ccmd_.offset[0] = 65535; // iMCs01 CH101 PIN2 is 5[V]. Forwarding flag.
  cmd_ccmd_.offset[1] = 32767; // STOP
  cmd_ccmd_.offset[2] = 32767;
  cmd_ccmd_.offset[3] = 32767;
  //! if iMCs01 is opened
  if (imcs01_fd_ > 0){
    write(imcs01_fd_, &cmd_ccmd_, sizeof(cmd_ccmd_));
    tcsetattr(imcs01_fd_, TCSANOW, &oldtio_imcs01_);
    close(imcs01_fd_);
  }
}

int IxisImcs01Driver::setImcs01()
{
  tcgetattr(imcs01_fd_, &oldtio_imcs01_);
  if(ioctl(imcs01_fd_, URBTC_CONTINUOUS_READ) < 0){
    ROS_ERROR_STREAM("iMCs01 ioctl URBTC_CONTINUOUS_READ error");
    exit(-1);
  }
  if(ioctl(imcs01_fd_, URBTC_BUFREAD) < 0){
    ROS_ERROR_STREAM("iMCs01 ioctl URBTC_BUFREAD error");
    //exit(-1);
  }

  cmd_ccmd_.selout     = SET_SELECT | CH0 | CH1 | CH2 | CH3; // All PWM.
  cmd_ccmd_.selin      = SET_SELECT; // All input using for encoder count.
  cmd_ccmd_.setoffset  = CH0 | CH1 | CH2 | CH3;
  cmd_ccmd_.offset[0]  = 58981;
  cmd_ccmd_.offset[1]  = 58981;
  cmd_ccmd_.offset[2]  = 58981;
  cmd_ccmd_.offset[3]  = 58981; // 1/2
  cmd_ccmd_.setcounter = CH0 | CH1 | CH2 | CH3;
  cmd_ccmd_.counter[1] = -3633; //-67[deg]*(1453/27), initialize.
  cmd_ccmd_.counter[2] = 0;
  cmd_ccmd_.posneg     = SET_POSNEG | CH0 | CH1 | CH2 | CH3; //POS PWM out.
  cmd_ccmd_.breaks     = SET_BREAKS | CH0 | CH1 | CH2 | CH3; //No Brake;
  cmd_ccmd_.magicno    = 0x00;

  if (ioctl(imcs01_fd_, URBTC_COUNTER_SET) < 0){
    ROS_ERROR_STREAM("Faild to ioctl: URBTC_COUNTER_SET");
    exit(-1);
  }
  if (write(imcs01_fd_, &cmd_ccmd_, sizeof(cmd_ccmd_)) < 0){
    ROS_ERROR_STREAM("Faild to ioctl: Faild to write");
    exit(-1);
  }
  cmd_ccmd_.setcounter = 0; // 次の書き込み時にエンコーダのカウンタを上書きしない
  ROS_INFO_STREAM( "iMCs01 is Connected.");
  return 0;
}

int IxisImcs01Driver::update()
{
  std::lock_guard<std::mutex> lck {communication_mutex_};
  if(read(imcs01_fd_, &received_data_, sizeof(received_data_))
     != sizeof(received_data_)){
    //ROS_WARN_STREAM("iMCs01 update() read error");
    return -1;
  }else{
    this->parseEncoderTime();
    this->parseFrontEncoderCounts();
    this->parseRearEncoderCounts();
    return 0;
  }
}

int IxisImcs01Driver::parseEncoderTime()
{
  delta_encoder_time_ = (double)(received_data_.time) - last_encoder_time_;

  if(delta_encoder_time_ < 0){
    delta_encoder_time_ = 65535 - (last_encoder_time_ - received_data_.time);
  }
  delta_encoder_time_ = delta_encoder_time_ / 1000.0; // [ms] -> [s]
  last_encoder_time_ = (double)(received_data_.time);
  return 0;
}

int IxisImcs01Driver::parseFrontEncoderCounts()
{
  int steer_encoder_counts = (int)received_data_.ct[1];
  state_.position[JOINT_INDEX_FRONT] = steer_encoder_counts*M_PI*67.0/3633.0/180.0;
  // cirkit_unit03_driverでの実装だとここで移動平均をしてる
  return 0;
}

int IxisImcs01Driver::parseRearEncoderCounts()
{
  int rear_encoder_counts[2]{(int)received_data_.ct[2], -(int)received_data_.ct[3]};

  for(int i = 0; i < 2; ++i){
    if(rear_delta_encoder_counts_[i] == -1
       || rear_encoder_counts[i] == rear_last_encoder_counts_[i]){ // First time.

     rear_delta_encoder_counts_[i] = 0;

    }else{
      rear_delta_encoder_counts_[i] = rear_encoder_counts[i] - rear_last_encoder_counts_[i];

      // checking iMCs01 counter overflow.
      if(rear_delta_encoder_counts_[i] > ROBOT_MAX_ENCODER_COUNTS/10){
        rear_delta_encoder_counts_[i] = rear_delta_encoder_counts_[i] - ROBOT_MAX_ENCODER_COUNTS;
      }
      if(rear_delta_encoder_counts_[i] < -ROBOT_MAX_ENCODER_COUNTS/10){
        rear_delta_encoder_counts_[i] = rear_delta_encoder_counts_[i] + ROBOT_MAX_ENCODER_COUNTS;
      }
    }
    rear_last_encoder_counts_[i] = rear_encoder_counts[i];

    // Update rear state
    // pulse rate : 40.0, gear_rate : 33.0
    // position = (counts / (40.0 * 33.0)) * pi = counts * 0.00237999...
    state_.position[i] += rear_delta_encoder_counts_[i]*0.0238;
    state_.velocity[i] = (rear_delta_encoder_counts_[i]*0.0238/delta_encoder_time_);
  }

  return 0;
}

sensor_msgs::JointState IxisImcs01Driver::getJointState()
{
  return state_;
}

int IxisImcs01Driver::controlRearWheel(double rear_speed)
{
  static int forward_stop_cnt = 0;
  static int back_stop_cnt = 0;
  static int max_spped_x = 3.0;
  static double average_spped_x = 0;
  static double u = 32767.0;
  double duty = 0;

  if (0.0 <= rear_speed && rear_speed <= 0.05){
    u = 32767.0;
    average_spped_x = 0;
    this->writeOffsetCmd(RunningMode::FORWARD, (unsigned short)u);
    running_state_ = RunningState::FORWARD_STOP;
    return 0;
  }

  // Forward
  if (rear_speed >= 0.0){
    double rear_speed_m_s = MIN(rear_speed, MAX_LIN_VEL); // return smaller
    if ((running_state_ == RunningState::FORWARD
         || running_state_ == RunningState::FORWARD_STOP) &&
        (state_.velocity[JOINT_INDEX_REAR_LEFT] >= 0
         || state_.velocity[JOINT_INDEX_REAR_RIGHT] >= 0)){
      // Now Forwarding
      average_spped_x = (average_spped_x + rear_speed)/2.0;
      u = 32767.0 + 32767.0 * average_spped_x *1.0;
      duty = MIN(u, 60000);
      duty = MAX(duty, 32767);
      this->writeOffsetCmd(RunningMode::FORWARD, (unsigned short)duty);
      running_state_ = RunningState::FORWARD;
      //ROS_INFO("ROBOT_STASIS_FORWARD");
    }else{
      // Now Backing
      // Need to stop once.
      duty = 32767; // STOP
      this->writeOffsetCmd(RunningMode::FORWARD, duty);
      average_spped_x = 0;

      if ((state_.velocity[JOINT_INDEX_REAR_LEFT] == 0.0
           && state_.velocity[JOINT_INDEX_REAR_RIGHT] == 0.0 )){
        running_state_ = RunningState::FORWARD_STOP;
        forward_stop_cnt = 0;
        duty = 32767;
        this->writeOffsetCmd(RunningMode::FORWARD, duty);
        for (int i = 0; i < 300; ++i){ usleep(1000); }
        //ROS_INFO("ROBOT_STASIS_FORWARD_STOP");
      }else{
        running_state_ = RunningState::OTHERWISE;
        forward_stop_cnt++;
        //ROS_INFO("ROBOT_STASIS_OTHERWISE");
      }
    }
  }else{
    // (rear_speed < 0) -> Back
    average_spped_x = 0;
    if (running_state_ == RunningState::BACK_STOP ||
        running_state_ == RunningState::BACK){
      // Now backing
      duty = 60000; // Back is constant speed
      this->writeOffsetCmd(RunningMode::BACK, duty);
      running_state_ = RunningState::BACK;
      //ROS_INFO("ROBOT_STASIS_BACK");
    }else{
      // Now forwarding
      if (back_stop_cnt >= 10){
        running_state_ = RunningState::BACK_STOP;
        back_stop_cnt = 0;
        duty = 32767; // STOP
        this->writeOffsetCmd(RunningMode::BACK, duty);
        for (int i = 0; i < 300; ++i){ usleep(1000); }
        //ROS_INFO("ROBOT_STASIS_BACK_STOP");
      }else{
        usleep(50000);
        duty = 32767; // STOP
        this->writeOffsetCmd(RunningMode::FORWARD, duty);
        running_state_ = RunningState::OTHERWISE;
        back_stop_cnt++;
        //ROS_INFO("ROBOT_STASIS_OTHERWISE");
      }
    }
  }
}

int IxisImcs01Driver::writeOffsetCmd(RunningMode mode,
                                     unsigned short duty)
{
  switch (mode) {
    case RunningMode::FORWARD : {
      //ROS_INFO_STREAM("RunningMode::FORWARD");
      cmd_ccmd_.offset[0] = 65535;
      //cmd_ccmd_.offset[0] = 32767;
      break;
    }
    case RunningMode::BACK : {
      //ROS_INFO_STREAM("RunningMode::BACK");
      cmd_ccmd_.offset[0] = 32767;
      //cmd_ccmd_.offset[1] = 65535;
      break;
    }
    default:
      break;
  }
  cmd_ccmd_.offset[1] = duty;
  //ROS_INFO_STREAM("duty : " << duty);
  std::lock_guard<std::mutex> lck {communication_mutex_};
  if (write(imcs01_fd_, &cmd_ccmd_, sizeof(cmd_ccmd_)) < 0){
    ROS_ERROR_STREAM("iMCs01 write fail.");
    return -1;
  }else{
    return 0;
  }
}
