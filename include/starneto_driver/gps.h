/**
 *****************************************************************************
 * COPYRIGHT STATEMENT
 * Copyright (c) 2018, Robosense Co.,Ltd. - www.robosense.ai
 * All Rights Reserved.
 *
 * You can not use, copy or spread without official authorization.
 *****************************************************************************
 *
 * Author: Robosense Localization Group
 * Version: 1.0.0
 * Date: 2018.5
 *
 * DESCRIPTION
 *
 * Robosense RTK-GPS driver.
 *
 */

#ifndef GPS_H
#define GPS_H

#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <time.h>
#include <ros/ros.h>
#include <sstream>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iomanip>
#include <boost/thread.hpp>
#include <sys/time.h>
#include <boost/lambda/lambda.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <unistd.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <starneto_driver/gps.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatStatus.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "starneto_driver/inspva.h"
#include <std_msgs/Bool.h>
#include "starneto_driver/time_conversion.h"

using namespace std;

struct STime
{
  unsigned char ucYear;
  unsigned char ucMonth;
  unsigned char ucDay;
  unsigned char ucHour;
  unsigned char ucMinute;
  unsigned char ucSecond;
  unsigned short usMiliSecond;
};

struct SLonLat
{
  int lLon;
  int lLat;
};

struct SGPSV
{
  short sGPSHeight;
  short sGPSYaw;
  int lGPSVelocity;
};

struct SGPSPrecision
{
  short sGPSNumber;
  short sGPSPDOP;
  short sGPSHDOP;
  short sGPSVDOP;
};

class GPSDriver
{
public:
  GPSDriver(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~GPSDriver();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher navsat_pub_;
  ros::Publisher navimu_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher inspva_pub_;
  ros::Publisher heartbeat_pub_;

  sensor_msgs::NavSatFix fix_;
  sensor_msgs::Imu imu_;
  nav_msgs::Odometry odom_;
  starneto_driver::inspva inspva_;

  struct STime stcTime_;
  struct SLonLat stcLonLat_;
  struct SGPSV stcGPSV_;
  struct SGPSPrecision stcGPSPrecision_;
  string s_frame_id_;
  string s_topic_gps_;
  string s_topic_imu_;
  string s_topic_odom_;
  string s_topic_inspva_;
  string s_port_;
  string sLog_path_;
  string sPackage_path_;
  int n_baudrate_;
  string s_frame_id_default_ = "/gps";
  string s_topic_default_ = "/gps";
  string s_port_default_ = "/dev/ttyACM0";
  int n_baudrate_default_ = 115200;

  bool if_log_;
  ofstream log_data_;

  void initParams();
  void CopeSerialData(char ucData[], unsigned short usLength);
  void runGPS(const string& rsPort, const int nBaudRate);
  string scanPort();
  string intToString(int value);

};



#endif  // PROJECT_GPS_H
