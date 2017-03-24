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
    has_utm_reading1_(false), has_utm_reading2_(false), isMoving_(false), has_imu_initial_(false), has_odom_initial_(false)
{
  ros::NodeHandle pnh("~");
  pnh.param("centerX", centerX_, CENTER_X);
  ROS_INFO("   center X: %f", centerX_);
  pnh.param("centerY", centerY_, CENTER_Y);
  ROS_INFO("   center Y: %f", centerY_);

  gps_odom_sub_ = nh->subscribe("/gps/odom", 100, &PoseEstimator::gpsOdomCallback, this);
  odom_p3at_sub_ = nh->subscribe("/p3at/odom", 100, &PoseEstimator::odomP3atCallback, this);
  //odom_p3at_sub_ = nh->subscribe("/RosAria/pose", 100, &PoseEstimator::odomP3atCallback, this);
  imu_sub_ = nh->subscribe("imu/data", 100, &PoseEstimator::imuCallback, this);
  initial_pose_sub_ = nh->subscribe("initialpose", 100, &PoseEstimator::initialPoseCallback, this);
  pose_estimated_pub_ = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/poseEstimated", 100, true);
  cmd_vel_pub_ = nh->advertise<geometry_msgs::Twist>("p3at/cmd_vel", 100, true);
  //cmd_vel_pub_ = nh->advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 100, true);
  imu_pub_ = nh->advertise<sensor_msgs::Imu>("imu_w_offset", 100, true);
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
      p2_.pose.pose.position.z = atan2((p2_.pose.pose.position.y - p1_.pose.pose.position.y) , (p2_.pose.pose.position.x - p1_.pose.pose.position.x));
      pose_estimated_pub_.publish(p2_);
      pose_estimated_sent_ = true;
      ROS_INFO("p1 - (%f,%f)", p1_.pose.pose.position.x, p1_.pose.pose.position.y);
      ROS_INFO("p2 - (%f,%f)", p2_.pose.pose.position.x, p2_.pose.pose.position.y);
      ROS_INFO("Published pose estimated (x=%f, y=%f, z=%f)", p2_.pose.pose.position.x, p2_.pose.pose.position.y, p2_.pose.pose.position.z);
    }
  }else{
    yaw_ekf_ = yaw_data_ + p2_.pose.pose.position.z - yaw_initial_;
    quat_ekf_ = tf::createQuaternionFromYaw(yaw_ekf_);

    imu_ekf_ = imu_data_;
    //imu_ekf_.orientation.x = quat_ekf_.getX();
    //imu_ekf_.orientation.y = quat_ekf_.getY();
    imu_ekf_.orientation.z = quat_ekf_.getZ();
    imu_ekf_.orientation.w = quat_ekf_.getW();

    imu_pub_.publish(imu_ekf_);
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
    //timer_ = ros::Time::now();
    //while((ros::Time::now() - timer_).toSec() < 2){
 //     mean_filter_.add(*msg);
   // }
    //utm_reading1_ = mean_filter_.getFilteredValue();
    //has_utm_reading1_ = true;
    //break;
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

  if(!has_odom_initial_)
  {
    odom_initial_.pose.pose.position.x = msg->pose.pose.position.x;
    has_odom_initial_ = true;
  }

  odom_p3at_.pose.pose.position.x = msg->pose.pose.position.x - odom_initial_.pose.pose.position.x;



}

void PoseEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
  if(!has_imu_initial_)
  {
    imu_initial_ = *msg;
    has_imu_initial_ = true;
    quat_initial_.setX(msg->orientation.x);
    quat_initial_.setY(msg->orientation.y);
    quat_initial_.setZ(msg->orientation.z);
    quat_initial_.setW(msg->orientation.w);

    yaw_initial_ = tf::getYaw(quat_initial_);
    std::cout << "yaw initial: " << yaw_initial_ << std::endl;
  }
  imu_data_ = *msg;
  quat_data_.setX(msg->orientation.x);
  quat_data_.setY(msg->orientation.y);
  quat_data_.setZ(msg->orientation.z);
  quat_data_.setW(msg->orientation.w);
  yaw_data_ = tf::getYaw(quat_data_);
  std::cout << "yaw data: " << yaw_data_ << std::endl;
}


void PoseEstimator::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){


}

void PoseEstimator::setVelocity(double vx, double wz)
{
  geometry_msgs::Twist msg;
  msg.linear.x = vx;
  msg.angular.z = wz;
  cmd_vel_pub_.publish(msg);
}


}
