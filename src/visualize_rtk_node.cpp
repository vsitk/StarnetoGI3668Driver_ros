#include <ros/ros.h>
#include <starneto_driver/visualize_rtk.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize_rtk");
  ros::NodeHandle n;
  RTKVisualizer v(n);
  ros::spin();
  return 0;
}




