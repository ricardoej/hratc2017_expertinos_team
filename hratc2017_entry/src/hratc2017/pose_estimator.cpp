/**
 *  This source file implements the PoseEstimator class, which is
 *based on the ROSNode helper class. It controls the pose_estimador_node.
 *
 *  Version: 1.0.0
 *  Created on: 21/03/2017
 *  Modified on: 21/03/2017
 *  Author: Luiz Fernando Nunes (luizfernandolfn@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/pose_estimator.h"

namespace hratc2017
{

/**
 * @brief PoseEstimator::PoseEstimador
 * @param nh
 */
PoseEstimator::PoseEstimator(ros::NodeHandle* nh)
    : ROSNode(nh, 30), has_pose_estimated_(false),
      current_state_(states::S1_READING1), has_utm_reading1_(false),
      has_utm_reading2_(false), isMoving_(false), has_imu_initial_(false),
      has_odom_initial_(false), mean_filter_x_(new utilities::MeanFilter(NUMBER_OF_SAMPLES)),
      mean_filter_y_(new utilities::MeanFilter(NUMBER_OF_SAMPLES)), reading_count_(0)
{
  ros::NodeHandle pnh("~");
  pnh.param("centerX", centerX_, CENTER_X);
  ROS_INFO("   center X: %f", centerX_);
  pnh.param("centerY", centerY_, CENTER_Y);
  ROS_INFO("   center Y: %f", centerY_);
  gps_odom_sub_ =
      nh->subscribe("/gps/odom", 100, &PoseEstimator::gpsOdomCallback, this);
  odom_p3at_sub_ =
      nh->subscribe("odom", 100, &PoseEstimator::odomP3atCallback, this);
  imu_sub_ = nh->subscribe("/imu/data", 100, &PoseEstimator::imuCallback, this);
  initial_pose_sub_ = nh->subscribe("/initialpose", 100,
                                    &PoseEstimator::initialPoseCallback, this);
  pose_estimated_pub_ =
      nh->advertise<nav_msgs::Odometry>("/odom_w_offset", 100, true);
  cmd_vel_pub_ = nh->advertise<geometry_msgs::Twist>("cmd_vel", 100, true);
  imu_pub_ = nh->advertise<sensor_msgs::Imu>("/imu_w_offset", 100, true);
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
  imu_sub_.shutdown();
  initial_pose_sub_.shutdown();
}

/**
 * @brief MapController::controlLoop
 */
void PoseEstimator::controlLoop()
{
  if (!has_pose_estimated_){
    calcPoseEstimated();
  }
  else if(has_imu_initial_){
    sendImuEkf();
  }
  if (has_utm_reading1_ && has_imu_initial_){
    sendOdomWithOffset();
  }
}

/**
 * @brief MapController::gpsOdomCallback receives gps odom from robot
 * @param msg gps odom
 */
void PoseEstimator::gpsOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  switch (current_state_)
  {
  case states::S1_READING1:
    if(reading_count_< NUMBER_OF_SAMPLES){
      mean_filter_x_->add(msg->pose.pose.position.x);
      mean_filter_y_->add(msg->pose.pose.position.y);
      std::cout << "Leitura "<< reading_count_<<": ["<< msg->pose.pose.position.x<< ", " << msg->pose.pose.position.y << "]\n";
      reading_count_++;
//      ros::Duration(0.25).sleep();
    }else{
      ROS_INFO("Reading 1 filtered");
      utm_reading1_.pose.pose.position.x = mean_filter_x_->getFilteredValue();
      utm_reading1_.pose.pose.position.y = mean_filter_y_->getFilteredValue();
      std::cout << "Final Values "<< utm_reading1_.pose.pose.position.x<< ", " << utm_reading1_.pose.pose.position.y << "]\n";
      has_utm_reading1_ = true;
      reading_count_ = 0;
    }
    break;
  case states::S2_MOVING:
    ROS_INFO("moving....");
    if (!isMoving_)
    {
      ROS_INFO("starting moving");
      setVelocity(0.5, 0);
      isMoving_ = true;
      timer_ = ros::Time::now();
    }
    if ((ros::Time::now() - timer_).toSec() > 2)
    {
      ROS_INFO("stopped moving");
      setVelocity(0, 0);
      current_state_ = states::S3_READING2;
    }
    break;
  case states::S3_READING2:
    if(reading_count_ < NUMBER_OF_SAMPLES)
    {
      mean_filter_x_->add(msg->pose.pose.position.x);
      mean_filter_y_->add(msg->pose.pose.position.y);
      std::cout << "Leitura "<< reading_count_<<": ["<< msg->pose.pose.position.x<< ", " << msg->pose.pose.position.y << "]\n";
      reading_count_++;
//      ros::Duration(0.25).sleep();
    }else{
      utm_reading2_.pose.pose.position.x = mean_filter_x_->getFilteredValue();
      utm_reading2_.pose.pose.position.y = mean_filter_y_->getFilteredValue();
      has_utm_reading2_ = true;
      ROS_INFO("Reading 2 filtered");
      std::cout << "Final Values "<< utm_reading2_.pose.pose.position.x<< ", " << utm_reading2_.pose.pose.position.y << "]\n";
      ROS_INFO("finished");
    }
    break;
  }
}

void PoseEstimator::odomP3atCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (!has_odom_initial_ && has_utm_reading1_)
  {
    odom_initial_.pose.pose.position.x = msg->pose.pose.position.x;
    odom_initial_.pose.pose.position.y = msg->pose.pose.position.y;
    has_odom_initial_ = true;
  }
  odom_p3at_ = *msg;
}

void PoseEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  if (!has_imu_initial_)
  {
    imu_initial_ = *msg;
    has_imu_initial_ = true;
    quat_initial_.setX(msg->orientation.x);
    quat_initial_.setY(msg->orientation.y);
    quat_initial_.setZ(msg->orientation.z);
    quat_initial_.setW(msg->orientation.w);

    yaw_initial_ = tf::getYaw(quat_initial_);
    // std::cout << "yaw initial: " << yaw_initial_ << std::endl;
  }
  imu_data_ = *msg;
  quat_data_.setX(msg->orientation.x);
  quat_data_.setY(msg->orientation.y);
  quat_data_.setZ(msg->orientation.z);
  quat_data_.setW(msg->orientation.w);
  yaw_data_ = tf::getYaw(quat_data_);
}

void PoseEstimator::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  initial_pose_ = *msg;
}


/**
 * @brief PoseEstimator::calcPoseEstimated
 */
void PoseEstimator::calcPoseEstimated(){
  if (has_utm_reading1_ && current_state_ == states::S1_READING1)
  {
    ROS_INFO("Getting p1");
    p1_.pose.pose.position.x = utm_reading1_.pose.pose.position.x - centerX_;
    p1_.pose.pose.position.y = utm_reading1_.pose.pose.position.y - centerY_;
    current_state_ = states::S2_MOVING;
  }
  if (has_utm_reading2_ && current_state_ == states::S3_READING2)
  {
    ROS_INFO("Getting p2");
    p2_.pose = utm_reading2_.pose;
    p2_.pose.pose.position.x = utm_reading2_.pose.pose.position.x - centerX_;
    p2_.pose.pose.position.y = utm_reading2_.pose.pose.position.y - centerY_;
    current_state_ = states::S4_FINISHED;
    // get angle by atan2
    p2_.pose.pose.position.z = atan2((p2_.pose.pose.position.y - p1_.pose.pose.position.y),(p2_.pose.pose.position.x - p1_.pose.pose.position.x));
    // pose_estimated_pub_.publish(p2_);
    has_pose_estimated_ = true;
    ROS_INFO("p1 - (%f,%f)", p1_.pose.pose.position.x, p1_.pose.pose.position.y);
    ROS_INFO("p2 - (%f,%f)", p2_.pose.pose.position.x, p2_.pose.pose.position.y);
    ROS_INFO("Published pose estimated (x=%f, y=%f, z=%f)", p2_.pose.pose.position.x, p2_.pose.pose.position.y, p2_.pose.pose.position.z);
  }
}

/**
 * @brief PoseEstimator::sendImuEkf
 */
void PoseEstimator::sendImuEkf(){
  yaw_ekf_ = yaw_data_ + p2_.pose.pose.position.z - yaw_initial_;
  quat_ekf_ = tf::createQuaternionFromYaw(yaw_ekf_);
  imu_ekf_ = imu_data_;
  imu_ekf_.orientation.z = quat_ekf_.getZ();
  imu_ekf_.orientation.w = quat_ekf_.getW();
  imu_pub_.publish(imu_ekf_);
}

/**
 * @brief PoseEstimator::sendOdomWithOffset
 * Calculate position estimate with offset and initial odom.
 * Position_estimate[t] = position[t] + initialpose - position[0]
 */
void PoseEstimator::sendOdomWithOffset(){
    odom_w_offset_ = odom_p3at_;
    odom_w_offset_.pose.pose.position.x += p1_.pose.pose.position.x - odom_initial_.pose.pose.position.x;
    odom_w_offset_.pose.pose.position.y += p1_.pose.pose.position.y - odom_initial_.pose.pose.position.y;
    pose_estimated_pub_.publish(odom_w_offset_);
}

void PoseEstimator::setVelocity(double vx, double wz)
{
  geometry_msgs::Twist msg;
  msg.linear.x = vx;
  msg.angular.z = wz;
  cmd_vel_pub_.publish(msg);
}
}
