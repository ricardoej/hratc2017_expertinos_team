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

#include "hratc2017/pose_estimator.h"

namespace hratc2017
{

/**
 * @brief MapController::MapController
 * @param nh
 */
PoseEstimator::PoseEstimator(ros::NodeHandle* nh)
  : ROSNode(nh, 30), pose_estimated_sent_(false), current_state_(states::S1_READING1),
    has_utm_reading1_(false), has_utm_reading2_(false), isMoving_(false)
{
  ros::NodeHandle pnh("~");
  pnh.param("centerX", centerX_, CENTER_X);
  ROS_INFO("   center X: %f", centerX_);
  pnh.param("centerY", centerY_, CENTER_Y);
  ROS_INFO("   center Y: %f", centerY_);

  gps_odom_sub_ = nh->subscribe("/gps/odom", 100, &PoseEstimator::gpsOdomCallback, this);
  odom_p3at_sub_ = nh->subscribe("/p3at/odom", 100, &PoseEstimator::odomP3atCallback, this);
  pose_estimated_pub_ = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/poseEstimated", 100, true);
  cmd_vel_pub_ = nh->advertise<geometry_msgs::Twist>("p3at/cmd_vel", 100, true);
}

/**
 * @brief MapController::~MapController
 */
PoseEstimator::~PoseEstimator()
{
  gps_odom_sub_.shutdown();
  cmd_vel_pub_.shutdown();
  pose_estimated_pub_.shutdown();
  odom_p3at_sub_.shutdown();
}

/**
 * @brief MapController::controlLoop
 */
void PoseEstimator::controlLoop()
{ 
  if (!pose_estimated_sent_)
  {
    if(has_utm_reading1_ && current_state_ == states::S1_READING1){
      ROS_INFO("Getting p1");
      p1_.pose.pose.position.x = utm_reading1_.pose.pose.position.x - centerX_;
      p1_.pose.pose.position.y = utm_reading1_.pose.pose.position.y - centerY_;
      current_state_ = states::S2_MOVING;
    }
    if(has_utm_reading2_ && current_state_ == states::S3_READING2){
      ROS_INFO("Getting p2");
      p2_.pose = utm_reading2_.pose;
      p2_.pose.pose.position.x = utm_reading2_.pose.pose.position.x - centerX_;
      p2_.pose.pose.position.y = utm_reading2_.pose.pose.position.y - centerY_;
      current_state_ = states::S4_FINISHED;
      //get angle z by atan
      p2_.pose.pose.position.z = atan((p2_.pose.pose.position.y - p1_.pose.pose.position.y) / (p2_.pose.pose.position.x - p1_.pose.pose.position.x));
      pose_estimated_pub_.publish(p2_);
      pose_estimated_sent_ = true;
      ROS_INFO("p1 - (%f,%f)", p1_.pose.pose.position.x, p1_.pose.pose.position.y);
      ROS_INFO("p2 - (%f,%f)", p2_.pose.pose.position.x, p2_.pose.pose.position.y);
      ROS_INFO("Published pose estimated (x=%f, y=%f, z=%f)", p2_.pose.pose.position.x, p2_.pose.pose.position.y, p2_.pose.pose.position.z);
    }
  }
}

/**
 * @brief MapController::gpsOdomCallback receives gps odom from robot
 * @param msg gps odom
 */
void PoseEstimator::gpsOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  switch (current_state_) {
  case states::S1_READING1:
    ROS_INFO("reading 1");
    utm_reading1_ = *msg;
    has_utm_reading1_ = true;
    break;
  case states::S2_MOVING:
    ROS_INFO("moving....");
    if(!isMoving_)
    {
      ROS_INFO("starting moving");
      setVelocity(0.5, 0);
      isMoving_ = true;
    }
    if(odom_p3at_.pose.pose.position.x > 1){
      ROS_INFO("stopped moving");
      setVelocity(0, 0);
      current_state_ = states::S3_READING2;
    }
    break;
  case states::S3_READING2:
    ROS_INFO("reading 2");
    utm_reading2_ = *msg;
    has_utm_reading2_ = true;
    ROS_INFO("finished");
    break;

  }

}

void PoseEstimator::odomP3atCallback(const nav_msgs::Odometry::ConstPtr& msg){
  odom_p3at_.pose.pose.position.x = msg->pose.pose.position.x;

}

void PoseEstimator::setVelocity(double vx, double wz)
{
  geometry_msgs::Twist msg;
  msg.linear.x = vx;
  msg.angular.z = wz;
  cmd_vel_pub_.publish(msg);
}


}
