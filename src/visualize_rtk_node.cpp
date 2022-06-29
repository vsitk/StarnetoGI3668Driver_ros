#include <ros/ros.h>
#include <rs_rtk/visualize_rtk.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize_rtk");
  ros::NodeHandle n;
  RTKVisualizer v(n);
  ros::spin();
  return 0;
}




