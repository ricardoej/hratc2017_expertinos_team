/**
 *  This source file implements the MapController class, which is
 *based on the ROSNode helper class. It controls the map_Controller_node.
 *
 *  Version: 1.0.0
 *  Created on: 21/03/2017
 *  Modified on: 21/03/2017
 *  Author: Ricardo Emerson Julio (ricardoej@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/map_controller.h"

namespace hratc2017
{

/**
 * @brief MapController::MapController
 * @param nh
 */
MapController::MapController(ros::NodeHandle* nh) 
  : ROSNode(nh, 30), has_utm_initial_pose_(false), initial_pose_sent_(false)
{
  gps_odom_sub_ =
      nh->subscribe("/gps/odom", 1, &MapController::gpsOdomCallback, this);
  initial_pose_pub_ =
      nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);
}

/**
 * @brief MapController::~MapController
 */
MapController::~MapController()
{
  gps_odom_sub_.shutdown();
  initial_pose_pub_.shutdown();
}

/**
 * @brief MapController::controlLoop
 */
void MapController::controlLoop() 
{
  if (!initial_pose_sent_ && has_utm_initial_pose_)
  {
    geometry_msgs::PoseWithCovarianceStamped initial_pose_msg;
    initial_pose_msg.pose = utm_initial_pose_.pose;

    double centerX = 483238.6184840562;
    double centerY = 6674530.097688958;

    initial_pose_msg.pose.pose.position.x = utm_initial_pose_.pose.pose.position.x - centerX;
    initial_pose_msg.pose.pose.position.y = utm_initial_pose_.pose.pose.position.y - centerY;

    initial_pose_pub_.publish(initial_pose_msg);
    initial_pose_sent_ = true;

    ROS_INFO("Published initial pose (x=%f, y=%f, z=%f)", 
      initial_pose_msg.pose.pose.position.x, initial_pose_msg.pose.pose.position.y, initial_pose_msg.pose.pose.position.z);
  }
}

/**
 * @brief MapController::gpsOdomCallback receives gps odom from robot and publishes initialpose
 * @param msg gps odom
 */
void MapController::gpsOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (!has_utm_initial_pose_)
  {
    utm_initial_pose_ = *msg;
    has_utm_initial_pose_ = true;

    // Unsubscribe??
    gps_odom_sub_.shutdown();
  }
}

}
