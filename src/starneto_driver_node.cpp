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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "starneto_driver");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  GPSDriver gps(n, n_private);
  ros::spin();
  return 0;
}
