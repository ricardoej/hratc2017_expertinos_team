#include "convert_odom.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "convert_odom_node");

  ros::NodeHandle n;

  ConvertOdom codom(n);

  codom.spin();

  return 0;

}
