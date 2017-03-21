/**
 *  This header file defines the ObstacleAvoider class, which is based
 *on the ROSNode class. It controls the obstacle_avoider_node.
 *
 *  Version: 1.1.3
 *  Created on: 20/03/2017
 *  Modified on: 21/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_ENTRIES_OBSTACLE_AVOIDER_H_
#define _HRATC2017_ENTRIES_OBSTACLE_AVOIDER_H_

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include "utilities/ros_node.h"

namespace hratc2017
{
class ObstacleAvoider : public utilities::ROSNode
{
public:
  ObstacleAvoider(ros::NodeHandle* nh);
  virtual ~ObstacleAvoider();

private:
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> scan_notifier_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher avoiding_pub_;
  bool avoiding_;
  virtual void controlLoop();
  void setVelocity(double vx, double wz);
  void setAvoiding(bool avoiding);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void reset();
};
}

#endif /* _HRATC2017_ENTRIES_OBSTACLE_AVOIDER_H_ */
