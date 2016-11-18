#include "cirkit_unit03_driver.hpp"

// inclusive dependency
#include "ThirdRobotInterface/ThirdRobotInterface.h"

// dependency to ROS
#include <nav_msgs/Odometry.h> // odom

// dependency to Boost
#include <boost/thread.hpp>

// dependency to std
#include <iostream>
#include <stdexcept>


using namespace std; // FIXME: this software is library to cirkit_unit03_driver_node, don't erosion grobal area.

cirkit::CirkitUnit03Driver::CirkitUnit03Driver(const ros::NodeHandle& nh)
: nh_(nh),
  rate_(100),
  odom_pub_(nh_.advertise<nav_msgs::Odometry>("/odom", 1)),
  steer_pub_(nh_.advertise<geometry_msgs::Twist>("/steer_ctrl", 1)),
  cmd_vel_sub_(nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind(&cirkit::CirkitUnit03Driver::cmdVelReceived, this, _1))),
  odom_broadcaster_(),
  imcs01_port_(), // over write by rosparam
  current_time_(),
  last_time_(),
  access_mutex_(),
  steer_dir_(),
  cirkit_unit03_(nullptr) // FIXME: init in constructor function
{
  ros::NodeHandle n("~");
  n.param<std::string>("imcs01_port", imcs01_port_, "/dev/urbtc0");
  double pulse_rate = 40.0;
  double geer_rate = 33.0;
  double wheel_diameter_right = 0.275;
  double wheel_diameter_left = 0.275;
  double tred_width = 0.595;
  n.param("pulse_rate", pulse_rate, pulse_rate);
  n.param("geer_rate", geer_rate, geer_rate);
  n.param("wheel_diameter_right", wheel_diameter_right, wheel_diameter_right);
  n.param("wheel_diameter_left", wheel_diameter_left, wheel_diameter_left);
  n.param("tred_width", tred_width, tred_width);

  cirkit_unit03_ = new cirkit::ThirdRobotInterface(imcs01_port_, 0); // FIXME: path received by argument for constructor
  cirkit_unit03_->setParams(pulse_rate, geer_rate, wheel_diameter_right, wheel_diameter_left, tred_width);

  resetCommunication();
}

cirkit::CirkitUnit03Driver::~CirkitUnit03Driver() {
  cirkit_unit03_->closeSerialPort();
  delete cirkit_unit03_;
}

void cirkit::CirkitUnit03Driver::resetCommunication() {
  if(cirkit_unit03_->openSerialPort() == 0) {
	  ROS_INFO("Connected to cirkit unit03.");
	  cirkit_unit03_->driveDirect(0, 0);
	} else {
	  ROS_FATAL("Could not connect to cirkit unit03.");
    delete cirkit_unit03_;
    throw runtime_error("Could not connect to cirkit unit03");
	}

  cirkit_unit03_->resetOdometry();
  cirkit_unit03_->setOdometry(0, 0, 0);
}

void cirkit::CirkitUnit03Driver::run() {
  double last_x, last_y, last_yaw;
  double vel_x, vel_y, vel_yaw;
  double dt;

  while(nh_.ok()) {
    current_time_ = ros::Time::now();
    last_x = cirkit_unit03_->odometry_x_;
    last_y = cirkit_unit03_->odometry_y_;
    last_yaw = cirkit_unit03_->odometry_yaw_;
    {
      boost::mutex::scoped_lock(access_mutex_); // why use mutex?
      if( cirkit_unit03_->getEncoderPacket() == -1) ROS_ERROR("Could not retrieve encoder packet.");
      else cirkit_unit03_->calculateOdometry();
    }
    dt = (current_time_ - last_time_).toSec();
    vel_x = (cirkit_unit03_->odometry_x_ - last_x)/dt;
    vel_y = (cirkit_unit03_->odometry_y_ - last_y)/dt;
    vel_yaw = (cirkit_unit03_->odometry_yaw_ - last_yaw)/dt;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(cirkit_unit03_->odometry_yaw_);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = cirkit_unit03_->odometry_x_;
    odom_trans.transform.translation.y = cirkit_unit03_->odometry_y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster_.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time_;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = cirkit_unit03_->odometry_x_;
    odom.pose.pose.position.y = cirkit_unit03_->odometry_y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vel_x;
    odom.twist.twist.linear.y = vel_y;
    odom.twist.twist.angular.z = vel_yaw;

    //publish the message
    odom_pub_.publish(odom);

    ros::spinOnce();
    rate_.sleep();
  }
}

void cirkit::CirkitUnit03Driver::cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
  static int steer = 0;
  {
    boost::mutex::scoped_lock(access_mutex_); // why use mutex?
    steer_dir_ = cirkit_unit03_->drive(cmd_vel->linear.x, cmd_vel->angular.z);
  }
  steer_pub_.publish(steer_dir_);
}
