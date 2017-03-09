/**
 *  This source file implements the MetalScanner class, which is
 *based on the ROSNode helper class. It controls the metal_scanner_node.
 *
 *  Version: 1.0.3
 *  Created on: 09/02/2017
 *  Modified on: 10/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *          Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *          Luiz Fernando Nunes (luizfernandolfn@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/metal_scanner.h"

namespace hratc2017
{

/**
 * @brief MetalScanner::MetalScanner
 * @param nh
 */
MetalScanner::MetalScanner(ros::NodeHandle* nh)
    : ROSNode(nh, 30), current_state_(states::S0_SETTING_UP), error_(0),
      s3_timer_(0), scanning_(false)
{
  ros::NodeHandle pnh("~");
  coils_.setParameters(pnh);
  pnh.param("linear_velocity_x", vx_, LINEAR_VELOCITY_X);
  ROS_INFO("   Linear velocity x: %f", vx_);
  pnh.param("angular_velocity_z", wz_, ANGULAR_VELOCITY_Z);
  ROS_INFO("   Angular velocity z: %f", wz_);
  pnh.param("Kp", Kp_, KP);
  ROS_INFO("   Kp: %f", Kp_);
  pnh.param("min_coil_signal", min_coil_signal_, MIN_COIL_SIGNAL);
  ROS_INFO("   Minimum coil signal: %f", min_coil_signal_);
  pnh.param("max_coil_signal", max_coil_signal_, MAX_COIL_SIGNAL);
  ROS_INFO("   Maximum coil signal: %f", max_coil_signal_);
  pnh.param("coil_signal_increment", coil_signal_increment_,
            COIL_SIGNAL_INCREMENT);
  ROS_INFO("   Coil signal increment: %f", coil_signal_increment_);
  pnh.param("coil_signal_tolerance", coil_signal_tolerance_,
            COIL_SIGNAL_TOLERANCE);
  ROS_INFO("   Coil signal tolerance: %f", coil_signal_tolerance_);
  pnh.param("safe_coil_signal", safe_coil_signal_, SAFE_COIL_SIGNAL);
  ROS_INFO("   Safe coil signal %f", safe_coil_signal_);
  pnh.param("sample_time", sample_time_, SAMPLE_TIME);
  ROS_INFO("   Sample time: %f", sample_time_);
  pnh.param("safe_time", safe_time_, SAFE_TIME);
  ROS_INFO("   Safe_time %f", safe_time_);
  pnh.param("spin_time", spin_time_, SPIN_TIME);
  ROS_INFO("   Spin_time %f", spin_time_);
  cmd_vel_pub_ = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
  moving_away_pub_ = nh->advertise<geometry_msgs::Twist>("moving_away", 1);
  coils_sub_ = nh->subscribe("/coils", 10, &Coils::coilsCallback, &coils_);
  scanning_sub_ =
      nh->subscribe("scanning", 1, &MetalScanner::scanningCallback, this);
  sampler_ = nh->createTimer(ros::Duration(sample_time_),
                             &MetalScanner::timerCallback, this);
}

/**
 * @brief MetalScanner::~MetalScanner
 */
MetalScanner::~MetalScanner()
{
  cmd_vel_pub_.shutdown();
  moving_away_pub_.shutdown();
  coils_sub_.shutdown();
  scanning_sub_.shutdown();
}

/**
 * @brief MetalScanner::controlLoop
 */
void MetalScanner::controlLoop()
{
  if (!scanning_)
  {
    ROS_DEBUG("   not scanning!!!");
    return;
  }
  setNextState();
  setVelocity();
}

/**
 * @brief MetalScanner::setNextState
 * @return
 */
void MetalScanner::setNextState()
{
  switch (current_state_)
  {
  case states::S0_SETTING_UP:
    ref_coil_signal_ = min_coil_signal_ - coil_signal_increment_;
    current_state_ = states::S1_ALIGNING;
    break;
  case states::S1_ALIGNING:
    if (fabs(error_) <= coil_signal_tolerance_)
    {
      ref_coil_signal_ += coil_signal_increment_;
      current_state_ = ref_coil_signal_ <= max_coil_signal_
                           ? states::S2_SCANNING
                           : states::S3_MOVING_AWAY;
    }
    break;
  case states::S2_SCANNING:
    if (coils_.getLeftValue() >= ref_coil_signal_ ||
        coils_.getRightValue() >= ref_coil_signal_)
    {
      current_state_ = states::S1_ALIGNING;
    }
    break;
  case states::S3_MOVING_AWAY:
    if (coils_.isBothLow())
    {
      s3_timer_ = ros::Time::now();
    }
    else if (!s3_timer_.isZero() &&
             ((ros::Time::now() - s3_timer_).toSec() > safe_time_))
    {
      current_state_ = states::S4_RESETTING;
    }
    break;
  case states::S4_RESETTING:
    reset();
    break;
  }
}

/**
 * @brief MetalScanner::setVelocity
 */
void MetalScanner::setVelocity()
{
  double wz;
  switch (current_state_)
  {
  case states::S0_SETTING_UP:
    ROS_DEBUG("   S0 - Setting up!");
    setVelocity(0, 0);
    break;
  case states::S1_ALIGNING: // P controller is implemented here
    ROS_DEBUG("   S1 - Aligning!");
    error_ = coils_.getLeftValue() - coils_.getRightValue();
    wz = error_ * Kp_;
    setVelocity(0, wz * (fabs(wz) > wz_ ? wz_ / fabs(wz) : 1));
    break;
  case states::S2_SCANNING:
    ROS_DEBUG("   S2 - Scanning foward!");
    setVelocity(vx_, 0);
    break;
  case states::S3_MOVING_AWAY:
    ROS_DEBUG("   S3 - Moving away!");
    setMovingAway(true);
    setVelocity(-vx_, 0);
    break;
  case states::S4_RESETTING:
    ROS_DEBUG("   S5 - Resetting!");
    break;
  }
}

/**
 * @brief MetalScanner::setVelocity
 * @param vx
 * @param wz
 */
void MetalScanner::setVelocity(double vx, double wz)
{
  geometry_msgs::Twist msg;
  msg.linear.x = vx;
  msg.angular.z = wz;
  cmd_vel_pub_.publish(msg);
}

/**
 * @brief MetalScanner::setMovingAway
 * @param moving_away
 */
void MetalScanner::setMovingAway(bool moving_away)
{
  std_msgs::Bool msg;
  msg.data = moving_away;
  moving_away_pub_.publish(msg);
}

/**
 * @brief MetalScanner::pauseCallback
 * @param msg
 */
void MetalScanner::scanningCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (scanning_ != msg->data)
  {
    scanning_ = msg->data;
    ROS_INFO("   scanning: %s", scanning_ ? "true" : "false");
    if (!scanning_)
    {
      reset();
    }
  }
}

/**
 * @brief MetalScanner::timerCallback
 * @param event
 */
void MetalScanner::timerCallback(const ros::TimerEvent& event)
{
  derivative_ = coils_.getMeanDerivedValue();
}

/**
 * @brief MetalScanner::reset
 */
void MetalScanner::reset()
{
  scanning_ = false;
  current_state_ = states::S0_SETTING_UP;
  setMovingAway(false);
  setVelocity(0, 0);
}
}
