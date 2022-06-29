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
#ifndef VISUALIZE_RTK
#define VISUALIZE_RTK

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <math.h>
#include <iostream>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

class RTKVisualizer
{
public:
  RTKVisualizer(ros::NodeHandle nh);

private:
  ros::NodeHandle nh_;
  nav_msgs::Path gps_path;
  pcl::PointCloud<pcl::PointXYZI> pcl_path;
  ros::Publisher gps_path_pub;
  ros::Publisher gps_path_cloud_pub;
  ros::Publisher GPS_origin_pub;

  bool initialization = false;
  bool use_imu = true;
  Eigen::Vector3d XYZ;

  double ori_lat;
  double ori_lon;
  double ori_ati;
  geometry_msgs::Quaternion odom_quat;

  Eigen::Vector3d WGS84toECEF(Eigen::Vector3d gps);
  void IMUCallback(const sensor_msgs::Imu& imu_msg);
  void GPSCallback(const sensor_msgs::NavSatFix& gps_msg);
};

#endif  // !VISUALIZE_RTK
