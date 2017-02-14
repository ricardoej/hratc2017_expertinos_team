#ifndef CONVERT_ODOM_H
#define CONVERT_ODOM_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


class ConvertOdom
{
public:
  ConvertOdom(ros::NodeHandle n);
  virtual ~ConvertOdom();

public:

  void ekfOdomCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& ekf_odom);
  void spin();

private:

  ros::NodeHandle n_;

  ros::Subscriber ekf_odom_sub_;
  ros::Publisher convertOdom_pub_;

  nav_msgs::Odometry odom_converted_;


};


#endif // CONVERT_ODOM_H
