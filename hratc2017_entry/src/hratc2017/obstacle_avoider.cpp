/**
 *  This source file implements the ObstacleAvoider class, which is
 *based on the ROSNode helper class. It controls the obstacle_avoider_node.
 *
 *  Version: 1.1.3
 *  Created on: 20/03/2017
 *  Modified on: 23/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/obstacle_avoider.h"

namespace hratc2017
{

/**
 * @brief ObstacleAvoider::ObstacleAvoider
 * @param nh
 */
ObstacleAvoider::ObstacleAvoider(ros::NodeHandle* nh)
    : ROSNode(nh, 30), scan_sub_(*nh, "scan", 10),
      scan_notifier_(scan_sub_, listener_, "base_link", 10), center_index_(0),
      angle_index_size_(0)
{
  ros::NodeHandle pnh("~");
  pnh.param("linear_velocity_x", vx_, LINEAR_VELOCITY_X);
  ROS_INFO("   Linear velocity x: %lf", vx_);
  pnh.param("angular_velocity_z", wz_, ANGULAR_VELOCITY_Z);
  ROS_INFO("   Angular velocity z: %lf", wz_);
  pnh.param("angle", angle_, ANGLE);
  ROS_INFO("   Angle: %lf", angle_);
  pnh.param("inner_minor_radius", inner_minor_radius_, INNER_MINOR_RADIUS);
  ROS_INFO("   Inner minor radius: %lf", inner_minor_radius_);
  pnh.param("inner_major_radius", inner_major_radius_, INNER_MAJOR_RADIUS);
  ROS_INFO("   Inner major radius: %lf", inner_major_radius_);
  pnh.param("outer_minor_radius", outer_minor_radius_, OUTER_MINOR_RADIUS);
  ROS_INFO("   Outer minor radius: %lf", outer_minor_radius_);
  pnh.param("outer_major_radius", outer_major_radius_, OUTER_MAJOR_RADIUS);
  ROS_INFO("   Outer major radius: %lf", outer_major_radius_);
  cmd_vel_pub_ = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
  avoiding_pub_ = nh->advertise<std_msgs::Bool>("avoiding_obstacle", 1);
  scan_notifier_.registerCallback(
      boost::bind(&ObstacleAvoider::scanCallback, this, _1));
  scan_notifier_.setTolerance(ros::Duration(0.001));
  inner_ellipse_viz_pub_ =
      nh->advertise<visualization_msgs::Marker>("inner_ellipse_marker", 10);
  outer_ellipse_viz_pub_ =
      nh->advertise<visualization_msgs::Marker>("outer_ellipse_marker", 10);
  reset();
  populateEllipseMsgs();
}

/**
 * @brief ObstacleAvoider::~ObstacleAvoider
 */
ObstacleAvoider::~ObstacleAvoider()
{
  cmd_vel_pub_.shutdown();
  avoiding_pub_.shutdown();
  inner_ellipse_viz_pub_.shutdown();
  outer_ellipse_viz_pub_.shutdown();
}

/**
 * @brief ObstacleAvoider::controlLoop
 */
void ObstacleAvoider::controlLoop()
{
  if (angle_index_size_ == 0)
  {
    return;
  }
  bool avoiding(false);
  double scale, vx, wz;
  double x(0.0), y(0.0);
  for (int i(0); i < cloud_.points.size(); i++)
  {
    if (isInsideOuterEllipse(cloud_.points[i]))
    {
      avoiding = true;
      scale = !isInsideInnerEllipse(cloud_.points[i])
                  ? fabs(i - center_index_) / angle_index_size_
                  : 0;
      x += (1 - scale) * cloud_.points[i].x;
      y += (1 - scale) * cloud_.points[i].y;
    }
  }
  if (avoiding)
  {
    setAvoiding(true);
    double theta(atan2(y, x)); //, r(sqrt(pow(x, 2) + pow(y, 2)));
    scale = fabs(theta) / angle_;
    ROS_INFO("theta: %lf, scale: %lf", theta * 180 / M_PI, scale);
    vx = scale * vx_;
    wz = (theta < 0.0 ? 1 : -1) * (1 - scale) * wz_;
    setVelocity(vx, wz);
    ROS_INFO("vx: %lf, wz: %lf", vx, wz);
  }
  else if (avoiding != avoiding_)
  {
    setAvoiding(false);
  }
  publishMarkers();
}

/**
 * @brief ObstacleAvoider::setVelocity
 * @param vx
 * @param wz
 */
void ObstacleAvoider::setVelocity(double vx, double wz)
{
  geometry_msgs::Twist msg;
  msg.linear.x = vx;
  msg.angular.z = wz;
  cmd_vel_pub_.publish(msg);
}

/**
 * @brief ObstacleAvoider::setAvoiding
 * @param avoiding
 */
void ObstacleAvoider::setAvoiding(bool avoiding)
{
  avoiding_ = avoiding;
  std_msgs::Bool msg;
  msg.data = avoiding;
  avoiding_pub_.publish(msg);
}

/**
 * @brief ObstacleAvoider::publishMarkers
 */
void ObstacleAvoider::publishMarkers()
{
  inner_ellipse_msg_.header.stamp = ros::Time::now();
  inner_ellipse_viz_pub_.publish(inner_ellipse_msg_);
  outer_ellipse_msg_.header.stamp = ros::Time::now();
  outer_ellipse_viz_pub_.publish(outer_ellipse_msg_);
}

/**
 * @brief ObstacleAvoider::scanCallback
 * @param msg
 */
void ObstacleAvoider::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if (msg->ranges.empty())
  {
    return;
  }
  unsigned int angle_size(ceil(angle_ / msg->angle_increment));
  unsigned int center(ceil(msg->ranges.size() / 2));
  unsigned int begin(center > angle_size ? center - angle_size : 0);
  unsigned int end(center + angle_size < msg->ranges.size()
                       ? center + angle_size
                       : msg->ranges.size());
  angle_size = floor((end - begin) / 2);
  sensor_msgs::LaserScan valid_readings;
  valid_readings.header = msg->header;
  valid_readings.angle_increment = msg->angle_increment;
  valid_readings.angle_min = -angle_;
  valid_readings.angle_max = angle_;
  valid_readings.intensities = msg->intensities;
  valid_readings.range_min = msg->range_min;
  valid_readings.range_max = msg->range_max;
  valid_readings.ranges.assign(msg->ranges.begin() + begin,
                               msg->ranges.begin() + end + 1);
  try
  {
    projector_.transformLaserScanToPointCloud("base_link", valid_readings,
                                              cloud_, listener_);
    angle_index_size_ = angle_size;
    center_index_ = center;
  }
  catch (tf::TransformException& e)
  {
    ROS_WARN("Exception catched: %s", e.what());
  }
}

/**
 * @brief ObstacleAvoider::reset
 */
void ObstacleAvoider::reset()
{
  ROSNode::reset();
  setAvoiding(false);
}

/**
 * @brief ObstacleAvoider::isInsideInnerEllipse
 * @param p
 * @return
 */
bool ObstacleAvoider::isInsideInnerEllipse(geometry_msgs::Point32 p) const
{
  return sqrt(pow(p.x / inner_major_radius_, 2) +
              pow(p.y / inner_minor_radius_, 2)) < 1;
}

/**
 * @brief ObstacleAvoider::isInsideOuterEllipse
 * @param p
 * @return
 */
bool ObstacleAvoider::isInsideOuterEllipse(geometry_msgs::Point32 p) const
{
  return sqrt(pow(p.x / outer_major_radius_, 2) +
              pow(p.y / outer_minor_radius_, 2)) < 1;
}

/**
 * @brief ObstacleAvoider::populateEllipseMsgs
 */
void ObstacleAvoider::populateEllipseMsgs()
{
  inner_ellipse_msg_.header.frame_id = "/base_link";
  inner_ellipse_msg_.header.stamp = ros::Time::now();
  inner_ellipse_msg_.ns = "inner_ellipse";
  inner_ellipse_msg_.action = visualization_msgs::Marker::ADD;
  inner_ellipse_msg_.type = visualization_msgs::Marker::POINTS;
  inner_ellipse_msg_.color.r = 1.0;
  inner_ellipse_msg_.color.a = 1.0;
  inner_ellipse_msg_.scale.x = 0.05;
  inner_ellipse_msg_.scale.y = 0.05;
  outer_ellipse_msg_.header.frame_id = "/base_link";
  outer_ellipse_msg_.header.stamp = ros::Time::now();
  outer_ellipse_msg_.ns = "outer_ellipse";
  outer_ellipse_msg_.action = visualization_msgs::Marker::ADD;
  outer_ellipse_msg_.type = visualization_msgs::Marker::POINTS;
  outer_ellipse_msg_.color.r = 1.0;
  outer_ellipse_msg_.color.b = 1.0;
  outer_ellipse_msg_.color.a = 1.0;
  outer_ellipse_msg_.scale.x = 0.05;
  outer_ellipse_msg_.scale.y = 0.05;
  geometry_msgs::Point p;
  for (double t(-M_PI / 2); t <= M_PI / 2; t += 0.2)
  {
    p.x = inner_major_radius_ * cos(t);
    p.y = inner_minor_radius_ * sin(t);
    inner_ellipse_msg_.points.push_back(p);
    p.x = outer_major_radius_ * cos(t);
    p.y = outer_minor_radius_ * sin(t);
    outer_ellipse_msg_.points.push_back(p);
  }
}
}
