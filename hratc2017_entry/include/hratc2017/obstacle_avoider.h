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
#include <visualization_msgs/Marker.h>
#include "utilities/ros_node.h"

#define LINEAR_VELOCITY_X 0.1
#define ANGULAR_VELOCITY_Z 0.3
#define ANGLE M_PI / 4
#define INNER_MINOR_RADIUS 0.55
#define INNER_MAJOR_RADIUS 0.75
#define OUTER_MINOR_RADIUS 0.65
#define OUTER_MAJOR_RADIUS 0.80

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
  ros::Publisher inner_ellipse_viz_pub_;
  ros::Publisher outer_ellipse_viz_pub_;
  visualization_msgs::Marker inner_ellipse_msg_;
  visualization_msgs::Marker outer_ellipse_msg_;
  sensor_msgs::PointCloud cloud_;
  bool avoiding_;
  double vx_;
  double wz_;
  double inner_minor_radius_;
  double inner_major_radius_;
  double outer_minor_radius_;
  double outer_major_radius_;
  double angle_;
  unsigned int  angle_index_size_;
  unsigned int  center_index_;
  virtual void controlLoop();
  void setVelocity(double vx, double wz);
  void setAvoiding(bool avoiding);
  void publishMarkers();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  virtual void reset();
  bool isInsideInnerEllipse(geometry_msgs::Point32 p) const;
  bool isInsideOuterEllipse(geometry_msgs::Point32 p) const;
  void populateEllipseMsgs();
};
}

#endif /* _HRATC2017_ENTRIES_OBSTACLE_AVOIDER_H_ */
