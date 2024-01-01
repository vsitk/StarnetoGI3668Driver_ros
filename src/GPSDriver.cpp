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
#include <starneto_driver/gps.h>

#define BUFF_SIZE 256

using namespace boost::lambda;
using namespace boost::asio;
using namespace std;

GPSDriver::GPSDriver(ros::NodeHandle nh, ros::NodeHandle nh_private)
{
  ROS_INFO("GPS Driver Started");
  nh_ = nh;
  nh_private_ = nh_private;
  // default value
  s_frame_id_default_ = "/ins";
  s_topic_default_ = "/rs/gps";
  s_port_default_ = "/dev/ttyUSB0";
  n_baudrate_default_ = 230400;
  if_log_ = false;
  initParams();

  navsat_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(s_topic_gps_, 1);
  navimu_pub_ = nh_.advertise<sensor_msgs::Imu>(s_topic_imu_, 1);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(s_topic_odom_, 1);
  inspva_pub_ = nh_.advertise<starneto_driver::inspva>(s_topic_inspva_, 1);
  heartbeat_pub_ = nh_.advertise<std_msgs::Bool>(s_topic_inspva_ + "/heartbeat", 1);
  
  runGPS(s_port_, n_baudrate_);
}

GPSDriver::~GPSDriver()
{
  if (if_log_)
  {
    log_data_.close();
  }
}

void GPSDriver::initParams()
{
  nh_private_.param("frame", s_frame_id_, s_frame_id_default_);
  nh_private_.param("topic_gps", s_topic_gps_, s_topic_default_);
  nh_private_.param("topic_imu", s_topic_imu_, s_topic_default_);
  nh_private_.param("topic_odom", s_topic_odom_, s_topic_default_);
  nh_private_.param("topic_inspva", s_topic_inspva_, s_topic_default_);
  nh_private_.param("baudrate", n_baudrate_, n_baudrate_default_);
  nh_private_.param("debug_mode", if_log_, false);

  if (nh_private_.getParam("package_name", sPackage_path_))
  {
    if_log_ = true;
    time_t t = time(0);  // get time now
    struct tm* now = localtime(&t);
    char buffer[80];
    strftime(buffer, 80, "%Y%m%d-%H%M", now);

    std::string folder_path1 = sPackage_path_ + "/logs/";
    std::string folder_path2 = std::string(buffer) + "/";
    std::string folder_path = folder_path1 + folder_path2;

    if (!fopen(folder_path.c_str(), "wb"))
    {
      mkdir(folder_path.c_str(), S_IRWXU);
    }

    std::string fileName;
    fileName = folder_path + "gps_data" + ".txt";

    log_data_.open(fileName, std::ios::out | std::ios::app);
  }

  if (!nh_private_.getParam("port_rtk", s_port_))
  {
    s_port_ = scanPort();
  }
}

string GPSDriver::scanPort()
{
  cout << "[Searching Serial Port]";
  string result = "nan";
  vector<string> v_port;

  for (int i = 0; i < 10; i++)
  {
    v_port.push_back("/dev/ttyUSB" + intToString(i));
  }

  for (int i = 0; i < 10; i++)
  {
    v_port.push_back("/dev/ttyACM" + intToString(i));
  }

  vector<string>::iterator it_i;
  for (it_i = v_port.begin(); it_i != v_port.end(); ++it_i)
  {
    try
    {
      io_service io;
      serial_port port(io, *it_i);
      port.set_option(serial_port_base::baud_rate(n_baudrate_));
      port.set_option(serial_port_base::character_size(8));
      port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
      port.set_option(serial_port_base::parity(serial_port_base::parity::none));
      port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));

      char chr_buf[256];
      boost::array<unsigned char, 256> data_buf;
      size_t usLength = boost::asio::read(port, buffer(data_buf), transfer_at_least(256));
      boost::array<unsigned char, 256>::iterator itr = data_buf.begin();
      for (int i = 0; itr != data_buf.end(); itr++, i++)
      {
        chr_buf[i] = *itr;
      }
      vector<string> serial_input_line;
      boost::split(serial_input_line, chr_buf, boost::is_any_of("\n"));
      int flag = 0;
      for (int l = 0; l < serial_input_line.size(); l++)
      {
        std::cout << serial_input_line[l] << std::endl;
        if (boost::starts_with(serial_input_line[l], "$GP"))
        {
          flag = 1;
          break;
        }
      }

      if (flag)
      {
        cout << endl << "[Scan Port Success]: " << *it_i << endl;
        port.close();
        break;
      }
    }
    catch (std::exception& e)
    {
      cout << ">";
    }
  }

  if (it_i != v_port.end())
    result = *it_i;
  return result;
}

void GPSDriver::runGPS(const string& rsPort, const int nBaudRate)
{
  // char chr_buffer[BUFF_SIZE];
  try
  {
    io_service io;
    serial_port port(io, rsPort);
    port.set_option(serial_port_base::baud_rate(nBaudRate));
    port.set_option(serial_port_base::character_size(8));
    port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
    port.set_option(serial_port_base::parity(serial_port_base::parity::none));
    port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));

    // boost::array<unsigned char, BUFF_SIZE> data_buf;
    string prev_log_remain_data;

    ros::Rate loop_rate(100);

    uint32_t nCounter = 0;
    while (ros::ok())
    {
      // memset(chr_buffer, 0, BUFF_SIZE);
      try
      {
        boost::array<unsigned char, BUFF_SIZE> data_buf;
        size_t usLength = boost::asio::read(port, buffer(data_buf, BUFF_SIZE), transfer_exactly(BUFF_SIZE));

        // boost::array<unsigned char, BUFF_SIZE>::iterator itr = data_buf.begin();
        // for (int i = 0; itr != data_buf.end(); itr++, i++)
        // {
        //   chr_buffer[i] = *itr;
        // }
        if (usLength > 0)
        {
          vector<string> serial_input_line;
          boost::split(serial_input_line, (char *)data_buf.c_array(), boost::is_any_of("\n"));

          for (unsigned int l = 0; l < serial_input_line.size() - 1; l++)
          {
            if (l == 0 && !boost::starts_with(serial_input_line[l], "$GP")) {
              serial_input_line[l] = prev_log_remain_data.substr(0, prev_log_remain_data.length() - 6) + serial_input_line[l];
            }
            // std::cout << "[" << l << "]" << serial_input_line[l] << std::endl;

            #if 1
            vector<string> aryNMEAstring;
            boost::split(aryNMEAstring, serial_input_line[l], boost::is_any_of(","));

            if (if_log_ && boost::starts_with(serial_input_line[l], "$GP"))
            {
              log_data_ << serial_input_line[l] << std::endl;
              // std::cout << l << " " << serial_input_line[l] << " | " << aryNMEAstring.size() << std::endl;
            }

            // std::cout << serial_input_line[l] << std::endl;

            // if (boost::starts_with(serial_input_line[l], "$GPCHC") && aryNMEAstring.size() >= 24)
            if (boost::starts_with(serial_input_line[l], "$GPCHC"))
            {
              uint16_t gps_week = atoi(aryNMEAstring[1].c_str());
              double gps_time = atof(aryNMEAstring[2].c_str());

              double yaw = (atof(aryNMEAstring[3].c_str()));
              double pitch = atof(aryNMEAstring[4].c_str()) / 180.0 * M_PI;
              double roll = atof(aryNMEAstring[5].c_str()) / 180.0 * M_PI;

              double gyro_x = atof(aryNMEAstring[6].c_str()) / 180.0 * M_PI;
              double gyro_y = atof(aryNMEAstring[7].c_str()) / 180.0 * M_PI;
              double gyro_z = atof(aryNMEAstring[8].c_str()) / 180.0 * M_PI;

              double acc_x = (atof(aryNMEAstring[9].c_str())) * 9.780;
              double acc_y = atof(aryNMEAstring[10].c_str()) * 9.780;
              double acc_z = atof(aryNMEAstring[11].c_str()) * 9.780;
              
              double latitude = atof(aryNMEAstring[12].c_str());
              double longitude = atof(aryNMEAstring[13].c_str());
              double altitude = atof(aryNMEAstring[14].c_str());
             
              double velocity_x = atof(aryNMEAstring[15].c_str());
              double velocity_y = atof(aryNMEAstring[16].c_str());
              double velocity_z = atof(aryNMEAstring[17].c_str());

              double velocity_v = atof(aryNMEAstring[18].c_str());

              if (!(isnan(latitude) || isnan(longitude) || isnan(altitude) || isnan(pitch) || isnan(roll) ||
                  isnan(yaw) || isnan(velocity_x) || isnan(velocity_y) || isnan(velocity_z)) )
              {
                const uint8_t status = atoi(aryNMEAstring[21].c_str());
                
                double yaw_ECEF;
                if (yaw >= 0.0 && yaw <= 270.0)
                  yaw_ECEF = (90.0 - yaw) / 180.0 * M_PI;
                else
                  yaw_ECEF = (450.0 - yaw) / 180.0 * M_PI;

                geometry_msgs::Quaternion quat;
                tf::Quaternion quat_tf = tf::createQuaternionFromRPY(roll, pitch, yaw_ECEF);
                tf::quaternionTFToMsg(quat_tf, quat);
                // odom_.pose.pose.orientation = quat;
                // odom_.twist.twist.linear.x = velocity_x;
                // odom_.twist.twist.linear.y = velocity_y;
                // odom_.twist.twist.linear.z = velocity_z;
                // odom_.header.stamp = ros::Time().now();
                // odom_.header.frame_id = s_frame_id_;

                // fix_.latitude = latitude;
                // fix_.longitude = longitude;
                // fix_.altitude = altitude;
                // fix_.header.stamp = odom_.header.stamp;
                // fix_.header.frame_id = s_frame_id_;
                
                inspva_.gps_week_number       =     gps_week;
                inspva_.gps_week_milliseconds =     static_cast<uint32_t>(gps_time * 1000);
                inspva_.latitude              =     latitude;
                inspva_.longitude             =     longitude;
                inspva_.height                =     altitude;
                inspva_.north_velocity        =     velocity_y;
                inspva_.east_velocity         =     velocity_x;
                inspva_.up_velocity           =     velocity_z;
                inspva_.roll                  =     roll;
                inspva_.pitch                 =     pitch;
                inspva_.azimuth               =     yaw;
                inspva_.status                =     status;
                inspva_.header.stamp          =     ros::Time(gps2unix(inspva_.gps_week_number * 604800 + inspva_.gps_week_milliseconds * 0.001));
                inspva_.header.frame_id       =     s_frame_id_;
                inspva_.header.seq            =     nCounter;

                imu_.angular_velocity.x       =     velocity_x;
                imu_.angular_velocity.y       =     velocity_y;
                imu_.angular_velocity.z       =     velocity_z;
                imu_.linear_acceleration.x    =     acc_x;
                imu_.linear_acceleration.y    =     acc_y;
                imu_.linear_acceleration.z    =     acc_z;
                imu_.orientation              =     quat;
                imu_.header.frame_id          =     s_frame_id_;
                imu_.header.stamp             =     inspva_.header.stamp;
                imu_.header.seq               =     nCounter;

                // navsat_pub_.publish(fix_);
                // odom_pub_.publish(odom_);
                inspva_pub_.publish(inspva_);
                navimu_pub_.publish(imu_);
              }
              else
              {
                std::cout << "\033[1;31m [Abnormal GPS Data]:\033[0m [lat,lon,alt]= [" << latitude
                          << "," << longitude << "," << altitude << "]"
                          << std::endl;
              }
            }
            // else if (boost::starts_with(serial_input_line[l], "$GTIMU") && aryNMEAstring.size() > 9) {
            //    uint32_t gps_week = atoi(aryNMEAstring[1].c_str());
            //    double gps_time = atof(aryNMEAstring[2].c_str());
            //    double gyro_x = atof(aryNMEAstring[3].c_str()) / 180.0 * M_PI;
            //    double gyro_y = atof(aryNMEAstring[4].c_str()) / 180.0 * M_PI;
            //    double gyro_z = atof(aryNMEAstring[5].c_str()) / 180.0 * M_PI;
            //    double acc_x = (atof(aryNMEAstring[6].c_str())) * 9.780;
            //    double acc_y = atof(aryNMEAstring[7].c_str()) * 9.780;
            //    double acc_z = atof(aryNMEAstring[8].c_str()) * 9.780;

            //    if (!(isnan(gps_week) || isnan(gps_time) || isnan(gyro_x) || isnan(gyro_y) || isnan(gyro_z) ||
            //        isnan(acc_x) || isnan(acc_y) || isnan(acc_z))) {
            //      imu_.orientation = odom_.pose.pose.orientation;
            //      imu_.angular_velocity.x = gyro_x;
            //      imu_.angular_velocity.y = gyro_y;
            //      imu_.angular_velocity.z = gyro_z;
            //      imu_.linear_acceleration.x = acc_x;
            //      imu_.linear_acceleration.y = acc_y;
            //      imu_.linear_acceleration.z = acc_z;
            //      imu_.header.stamp = ros::Time().now();
            //      imu_.header.frame_id = s_frame_id_;
            //      navimu_pub_.publish(imu_);
            //    }
            //    else
            //    {
            //      std::cout << "\033[1;31m [Abnormal IMU Data]:\033[0m [lat,lon,alt]= [" << atof(aryNMEAstring[1].c_str())
            //                << "," << atof(aryNMEAstring[2].c_str()) << "," << atof(aryNMEAstring[3].c_str()) << "]"
            //                << std::endl;
            //    }
            //  }

            ++nCounter;
            if (nCounter % 100 == 0) {
              std_msgs::Bool heartbeat;
              heartbeat.data = 1;
              heartbeat_pub_.publish(heartbeat);
            }
            #endif
          }
          prev_log_remain_data = serial_input_line[serial_input_line.size() - 1]; 
        }
      }

      catch (boost::system::system_error& e)
      {
        if (e.code().message() == "Interrupted system call")
        {  // ctrl+c
          ros::shutdown();
        }
        else
        {
          std::cerr << " \033[1;31m  ReadData Error:" << e.what()<< "\033[0m" << endl;
        }
      }

      //ros::spinOnce();
      // loop_rate.sleep();
    }

    port.close();
  }
  catch (std::exception& e)
  {
    std::cerr << " \033[1;31m [runRTK Error]: "
          << s_port_ << " "
          << e.what() <<"\033[0m" << std::endl;
  }
}

string GPSDriver::intToString(int value)
{
  ostringstream stream;
  stream << value;
  return stream.str();
}
