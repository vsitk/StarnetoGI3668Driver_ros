#include "rs_rtk/visualize_rtk.h"

RTKVisualizer::RTKVisualizer(ros::NodeHandle nh)
{
  nh_=nh;
  ros::Subscriber sub_gps = nh_.subscribe("/gps", 10, &RTKVisualizer::GPSCallback,this);
  ros::Subscriber sub_imu = nh_.subscribe("/imu_gpfpd", 10, &RTKVisualizer::IMUCallback,this);
  gps_path_pub = nh_.advertise<nav_msgs::Path>("/gps_path_rtk", 1, true);
  gps_path_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_point_path",1,true);
  gps_path.header.frame_id = "rs_odom";
  GPS_origin_pub = nh_.advertise<sensor_msgs::NavSatFix>("/rs_GPS_origin_gpfpd", 1, true);
}

Eigen::Vector3d RTKVisualizer::WGS84toECEF(Eigen::Vector3d gps)
{
  double SEMI_MAJOR_AXIS = 6378137.0;
  double RECIPROCAL_OF_FLATTENING = 298.257223563;
  double SEMI_MINOR_AXIS = 6356752.3142;
  double FIRST_ECCENTRICITY_SQUARED = 6.69437999014e-3;
  double SECOND_ECCENTRICITY_SQUARED = 6.73949674228e-3;

  double lon = gps[0];
  double lat = gps[1];
  double ati = gps[2];

  double rad_lon = lon / 180 * M_PI;
  double rad_lat = lat / 180 * M_PI;

  double sin_lon = sin(rad_lon);
  double cos_lon = cos(rad_lon);
  double sin_lat = sin(rad_lat);
  double cos_lat = cos(rad_lat);

  double chi = sqrt(1.0 - FIRST_ECCENTRICITY_SQUARED * sin_lat * sin_lat);
  double N = SEMI_MAJOR_AXIS / chi + ati;

  Eigen::Vector3d ret;
  ret << N * cos_lat * cos_lon, N * cos_lat * sin_lon,
      (SEMI_MAJOR_AXIS * (1.0 - FIRST_ECCENTRICITY_SQUARED) / chi + ati) * sin_lat;

  return ret;
}

void RTKVisualizer::IMUCallback(const sensor_msgs::Imu& imu_msg)
{
  odom_quat = imu_msg.orientation;
}

void RTKVisualizer::GPSCallback(const sensor_msgs::NavSatFix& gps_msg)
{
  if (!initialization)
  {
    initialization = true;
    sensor_msgs::NavSatFix fix;
    fix.header.stamp = ros::Time().now();
    fix.header.frame_id = "base_link2_rtk";
    fix.latitude = ori_lat;
    fix.longitude = ori_lon;
    GPS_origin_pub.publish(fix);
  }

  if(fabs(gps_msg.longitude)<0.1 || fabs(gps_msg.latitude)<0.1)
  {
    std::cerr << "Abnormal Data Discarded. " << '\n';
    return;
  }

  Eigen::Vector3d gps;
  gps << gps_msg.longitude, gps_msg.latitude, gps_msg.altitude;

  Eigen::Vector3d ret;
  ret = WGS84toECEF(gps);

  double rad_lon = ori_lon / 180 * M_PI;
  double rad_lat = ori_lat / 180 * M_PI;
  double sin_lon = sin(rad_lon);
  double cos_lon = cos(rad_lon);
  double sin_lat = sin(rad_lat);
  double cos_lat = cos(rad_lat);

  Eigen::Matrix3d rot;

  rot<<-sin_lon,                 cos_lon,       0,
       -sin_lat*cos_lon, -sin_lat*sin_lon, cos_lat,
        cos_lat*cos_lon,  cos_lat*sin_lon,       0;

  Eigen::Vector3d z0;
  Eigen::Vector3d wgs0;
  wgs0 << ori_lon, ori_lat, ori_ati;
  //
  z0 = WGS84toECEF(wgs0);
  Eigen::Vector3d l;
  l = ret - z0;
  XYZ = rot * l;

  std::cout << "XYZ: " << XYZ.transpose() << std::endl;

  //---------------------------------------------
  // visualize the GPS path...

  geometry_msgs::PoseStamped odom2;
  sensor_msgs::PointCloud2 cloud_path;
  pcl::PointXYZI pose_pcl;

  odom2.header.stamp = gps_msg.header.stamp;
  odom2.header.frame_id = "rs_odom";

  // set the position
  pose_pcl.x = XYZ[0];
  pose_pcl.y = XYZ[1];
  pose_pcl.z = 0.0;
  if(gps_msg.status.service >= 0)
  {
   pose_pcl.intensity = 1000*gps_msg.status.service;
   std::cerr << "[intensity]" << pose_pcl.intensity << std::endl;
  }
  else
  {
     std::cerr << "[intensity]: Error" << pose_pcl.intensity << std::endl;
     pose_pcl.intensity = 0;
   }

  pcl_path.push_back(pose_pcl);
  pcl::toROSMsg	(pcl_path, cloud_path);

  cloud_path.header.frame_id = "rs_odom";
  odom2.pose.position.x = XYZ[0];
  odom2.pose.position.y = XYZ[1];
  odom2.pose.position.z = 0.0;

  gps_path.poses.push_back(odom2);
  gps_path_pub.publish(gps_path);
  gps_path_cloud_pub.publish(cloud_path);

  static tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = gps_msg.header.stamp;
  odom_trans.header.frame_id = "rs_odom";
  // odom_trans.header.frame_id = frame_id_;
  odom_trans.child_frame_id = "base_link2_rtk";  // child_frame_id_
  odom_trans.transform.translation.x = XYZ[0];
  odom_trans.transform.translation.y = XYZ[1];
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //std::cout << "XYZ[] =" << XYZ[0] << "," << XYZ[1] << "," << std::endl;

  odom_broadcaster.sendTransform(odom_trans);
}
