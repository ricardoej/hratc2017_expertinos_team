/**
 *  This source file implements the ObstacleAvoider class, which is
 *based on the ROSNode helper class. It controls the obstacle_avoider_node.
 *
 *  Version: 1.1.3
 *  Created on: 20/03/2017
 *  Modified on: 21/03/2017
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
ObstacleAvoider::ObstacleAvoider(ros::NodeHandle* nh) : ROSNode(nh, 30),
  scan_sub_(*nh, "scan", 10),
  scan_notifier_(scan_sub_, listener_, "base_link", 10)
{
  cmd_vel_pub_ = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
  avoiding_pub_ = nh->advertise<std_msgs::Bool>("avoiding_obstacle", 1);
  scan_notifier_.registerCallback(
        boost::bind(&ObstacleAvoider::scanCallback, this, _1));
  scan_notifier_.setTolerance(ros::Duration(0.001));
  reset();
}

/**
 * @brief ObstacleAvoider::~ObstacleAvoider
 */
ObstacleAvoider::~ObstacleAvoider()
{
  cmd_vel_pub_.shutdown();
  avoiding_pub_.shutdown();
}

/**
 * @brief ObstacleAvoider::controlLoop
 */
void ObstacleAvoider::controlLoop() {}

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
 * @brief ObstacleAvoider::scanCallback
 * @param msg
 */
void ObstacleAvoider::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  ROS_INFO("[scanCB] size: %ld", msg->ranges.size());////////////
  double sum(0.0);
  for (int i(0); i < msg->ranges.size(); i++)
  {
    sum += msg->ranges[i];
  }
  ROS_INFO("[scanCB] mean: %lf", sum / msg->ranges.size());
  sensor_msgs::PointCloud cloud;
  try
  {
    projector_.transformLaserScanToPointCloud("base_link", *msg, cloud, listener_);
  }
  catch (tf::TransformException& e)
  {
    ROS_WARN("Exception catched: %s", e.what());
    return;
  }
}

/**
 * @brief ObstacleAvoider::reset
 */
void ObstacleAvoider::reset()
{
  ROS_INFO("   Resetting %s!!!", ROSNode::getName().c_str());
  setAvoiding(false);
}
}
