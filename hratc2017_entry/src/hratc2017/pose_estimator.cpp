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
    : ROSNode(nh, 30), started_hratc2017_(false), has_pose_estimated_(false),
      current_state_(states::S1_READING1), has_utm_reading1_(false),
      has_utm_reading2_(false), moving_(false), has_imu_initial_(false),
      has_odom_initial_(false), reading_count_(0), angle_(0.0),
      mean_filter_(new utilities::PointMeanFilter(NUMBER_OF_SAMPLES))
{
  ros::NodeHandle pnh("~");
  double aux;
  pnh.param("center_x", aux, CENTER_X);
  p0_.x = aux;
  pnh.param("center_y", aux, CENTER_Y);
  p0_.y = aux;
  ROS_INFO("   p0: (%lf, %lf)", p0_.x, p0_.y);
  gps_odom_sub_ =
      nh->subscribe("/gps/odom", 1, &PoseEstimator::gpsOdomCallback, this);
  odom_p3at_sub_ =
      nh->subscribe("odom", 1, &PoseEstimator::odomP3atCallback, this);
  imu_sub_ = nh->subscribe("/imu/data", 1, &PoseEstimator::imuCallback, this);
  pose_estimated_pub_ =
      nh->advertise<nav_msgs::Odometry>("/odom_w_offset", 10, true);
  cmd_vel_pub_ = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  imu_pub_ = nh->advertise<sensor_msgs::Imu>("/imu_w_offset", 10, true);
  start_hratc2017_cli_ =
      nh->serviceClient<std_srvs::Trigger>("/start_hratc2017");
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
  start_hratc2017_cli_.shutdown();
  if (mean_filter_)
  {
    delete mean_filter_;
    mean_filter_ = NULL;
  }
}

/**
 * @brief MapController::controlLoop
 */
void PoseEstimator::controlLoop()
{
  if (!has_pose_estimated_)
  {
    calcPoseEstimated();
  }
  else if (has_imu_initial_)
  {
    sendImuEkf();
    startHRATC2017();
  }
  if (has_utm_reading1_ && has_imu_initial_)
  {
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
    if (reading_count_ < NUMBER_OF_SAMPLES)
    {
      if ((ros::Time::now() - last_timestamp_).toSec() > SAMPLING_DURATION)
      {
        last_timestamp_ = ros::Time::now();
        mean_filter_->add(msg->pose.pose.position);
        std::cout << "Leitura " << reading_count_ << ": ["
                  << msg->pose.pose.position.x << ", "
                  << msg->pose.pose.position.y << "]\n";
        reading_count_++;
      }
    }
    else
    {
      ROS_INFO("Reading 1 filtered");
      p1_ = mean_filter_->getFilteredValue();
      std::cout << "Final Values " << p1_.x << ", " << p1_.y << "]\n";
      has_utm_reading1_ = true;
      reading_count_ = 0;
    }
    break;
  case states::S2_MOVING:
    ROS_INFO("moving....");
    if (!moving_)
    {
      ROS_INFO("starting moving");
      setVelocity(0.5, 0);
      moving_ = true;
      timer_ = ros::Time::now();
    }
    if ((ros::Time::now() - timer_).toSec() > 2)
    {
      ROS_INFO("stopped moving");
      setVelocity(0, 0);
      current_state_ = states::S3_READING2;
      last_timestamp_ = ros::Time::now();
    }
    break;
  case states::S3_READING2:
    if (reading_count_ < NUMBER_OF_SAMPLES)
    {
      if ((ros::Time::now() - last_timestamp_).toSec() > SAMPLING_DURATION)
      {
        last_timestamp_ = ros::Time::now();
        mean_filter_->add(msg->pose.pose.position);
        std::cout << "Leitura " << reading_count_ << ": ["
                  << msg->pose.pose.position.x << ", "
                  << msg->pose.pose.position.y << "]\n";
        reading_count_++;
      }
    }
    else
    {
      has_utm_reading2_ = true;
      ROS_INFO("Reading 2 filtered");
      p2_ = mean_filter_->getFilteredValue();
      std::cout << "Final Values " << p2_.x << ", " << p2_.y << "]\n";
      ROS_INFO("Finished");
    }
    break;
  }
}

/**
 * @brief PoseEstimator::odomP3atCallback
 * @param msg
 */
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

/**
 * @brief PoseEstimator::imuCallback
 * @param msg
 */
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
  }
  imu_data_ = *msg;
  quat_data_.setX(msg->orientation.x);
  quat_data_.setY(msg->orientation.y);
  quat_data_.setZ(msg->orientation.z);
  quat_data_.setW(msg->orientation.w);
  yaw_data_ = tf::getYaw(quat_data_);
}

/**
 * @brief PoseEstimator::calcPoseEstimated
 */
void PoseEstimator::calcPoseEstimated()
{
  if (has_utm_reading1_ && current_state_ == states::S1_READING1)
  {
    ROS_INFO("Getting p1");
    current_state_ = states::S2_MOVING;
  }
  if (has_utm_reading2_ && current_state_ == states::S3_READING2)
  {
    ROS_INFO("Getting p2");
    current_state_ = states::S4_FINISHED;
    angle_ = atan2(p2_.y - p1_.y, p2_.x - p1_.x);
    has_pose_estimated_ = true;
    ROS_INFO("p1 - (%f,%f)", p1_.x, p1_.y);
    ROS_INFO("p2 - (%f,%f)", p2_.x, p2_.y);
    ROS_INFO("Published pose estimated (x=%f, y=%f, z=%f)", p2_.x, p2_.y,
             angle_);
  }
}

/**
 * @brief PoseEstimator::sendImuEkf
 */
void PoseEstimator::sendImuEkf()
{
  yaw_ekf_ = yaw_data_ + angle_ - yaw_initial_;
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
void PoseEstimator::sendOdomWithOffset()
{
  odom_w_offset_ = odom_p3at_;
  odom_w_offset_.pose.pose.position.x +=
      p1_.x - odom_initial_.pose.pose.position.x;
  odom_w_offset_.pose.pose.position.y +=
      p1_.y - odom_initial_.pose.pose.position.y;
  pose_estimated_pub_.publish(odom_w_offset_);
}

/**
 * @brief PoseEstimator::setVelocity
 * @param vx
 * @param wz
 */
void PoseEstimator::setVelocity(double vx, double wz)
{
  geometry_msgs::Twist msg;
  msg.linear.x = vx;
  msg.angular.z = wz;
  cmd_vel_pub_.publish(msg);
}

/**
 * @brief PoseEstimator::startHRATC2017
 */
void PoseEstimator::startHRATC2017()
{
  if (started_hratc2017_)
  {
    return;
  }
  std_srvs::Trigger srv;
  if (start_hratc2017_cli_.call(srv))
  {
    if (srv.response.success)
    {
      ROS_INFO("%s", srv.response.message.c_str());
    }
    else
    {
      ROS_WARN("%s", srv.response.message.c_str());
    }
    started_hratc2017_ = true;
  }
}
}
