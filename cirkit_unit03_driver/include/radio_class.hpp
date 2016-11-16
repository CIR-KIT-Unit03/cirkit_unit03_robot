#include <ros/ros.h>

#include <numeric>
#include <string>
#include <stdint.h> // What this?
#include <termios.h>

// iMCs01
#include "ThirdRobotInterface/imcs01_driver/driver/urbtc.h"
#include "ThirdRobotInterface/imcs01_driver/driver/urobotc.h"

enum {
  FRONT,
  REAR
};

enum {
  ROBOT_STASIS_FORWARD,
  ROBOT_STASIS_FORWARD_STOP,
  ROBOT_STASIS_BACK,
  ROBOT_STASIS_BACK_STOP,
  ROBOT_STASIS_OTHERWISE
};

enum {
  FORWARD_MODE,
  FORWARD_STOP_MODE,
  BACK_MODE,
  BACK_STOP_MODE,
  STOP_MODE
};

//! Robot max encoder counts  
#define ROBOT_MAX_ENCODER_COUNTS 65535

//! Length of between front wheel and rear wheel [m]
#define WHEELBASE_LENGTH 0.94

//! Width of tread [m]
#define TREAD_WIDTH 0.53


//! Max linear velocity [m/s]
#define MAX_LIN_VEL 1.11 // 1.11[m/s] => 4.0[km/h]

//! Send packet size for ctrl stepping motor to arduino
#define SENDSIZE 7

#ifndef MIN
template<typedef T>
T MIN(const T& a, const T& b) { return a < b ? a : b };
#endif
#ifndef MAX
template<typedef T>
T MAX(const T& a, const T& b) { return a > b ? a : b };
#endif
#ifndef NORMALIZE
template<typedef T>
double NORMALIZE(const T& z) { return atan2(sin(z), cos(z)) };
#endif

class radio_class{

public:
  //! Constructor
  radio_class(const std::string& new_serial_port_imcs01, int new_baudrate_imcs01, const std::string& new_serial_port_arduino, int new_baudrate_arduino);

  //! Destructor
  ~radio_class();

  //! Open the serial port
  virtual int openSerialPort();

  //! Setting the serial port
  virtual int setSerialPort();

  //! Close the serial port
  virtual int closeSerialPort();

  virtual int radio_drive(double);

  //! Drive
  virtual int drive(double linear_speed, double angular_speed);

  //! Drive direct
  virtual int driveDirect(double front_angular, double rear_speed);// front_angular in [deg]

  //! Read the encoder pulses from iMCs01
  virtual int getEncoderPacket();

  //! Calculate Third robot odometry. Call after reading encoder pulses.
  virtual void calculateOdometry();

  //! Reset Third robot odometry.
  virtual void resetOdometry();

  //! Set new odometry.
  virtual void setOdometry(double new_x, double new_y, double new_yaw);

  //! Send stepping motor operating code to Arduino
  int sendOpcode(const char code);


  //! robot odometry x[m]
  double odometry_x_;
  //! robot odometry y[m]
  double odometry_y_;
  //! robot odometry yaw[rad]
  double odometry_yaw_;
  //! Front steer angle[deg].
  double steer_angle;
  //! Robot running status
  int stasis_;

protected:
  //! Parse data
  /*!
   * Data parsing function. Parses data comming from iMCs01.
   * \param buffer 			Data to be parsed.
   *
   * \return 0 if ok, -1 otherwise.
   */
  int parseEncoderPackets();
  int parseFrontEncoderCounts();
  int parseRearEncoderCounts();


  //! For access to iMCs01
  struct uin cmd_uin;
  //struct uout cmd_uout;
  struct ccmd cmd_ccmd;

  //! Serial port to which the robot is connected
  std::string imcs01_port_name;
  std::string arduino_port_name;

  //! File descriptor
  int fd_imcs01;
  int fd_arduino;

  //! Baudrate
  int baudrate_imcs01;
  int baudrate_arduino;

  //! Old and new termios struct
  termios oldtio_imcs01;
  termios newtio_imcs01;
  termios oldtio_arduino;
  termios newtio_arduino;

  //! Delta rear encoder counts. 
  int delta_rear_encoder_counts;

  //! Last rear encoder counts reading. For odometry calculation.
  int last_rear_encoder_counts;

  //! Last time reading encoder
  double last_rear_encoder_time;

  //! Delta time
  double delta_rear_encoder_time;

  //! Linear velocity
  double linear_velocity;

  //! Send packet data to Arduino.
  char sendPacket[SENDSIZE];

  //! Forward or Back mode flag
  int runmode;
};

