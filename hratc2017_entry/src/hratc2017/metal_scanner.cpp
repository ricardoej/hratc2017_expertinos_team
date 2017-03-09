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
    : ROSNode(nh, 30), current_state_(states::S0_SETTING_UP), angular_error_(0),
      timer_(0), scanning_(false), moving_away_(false)
{
  ros::NodeHandle pnh("~");
  coils_.setParameters(pnh);
  pnh.param("linear_velocity_x", vx_, LINEAR_VELOCITY_X);
  ROS_INFO("   Linear velocity x: %f", vx_);
  pnh.param("angular_velocity_z", wz_, ANGULAR_VELOCITY_Z);
  ROS_INFO("   Angular velocity z: %f", wz_);
  pnh.param("linear_Kp", linear_Kp_, LINEAR_KP);
  ROS_INFO("   Linear Kp: %f", linear_Kp_);
  pnh.param("angular_Kp", angular_Kp_, ANGULAR_KP);
  ROS_INFO("   Angular Kp: %f", angular_Kp_);
  pnh.param("linear_tolerance", linear_tolerance_,
            LINEAR_TOLERANCE);
  ROS_INFO("   Linear tolerance: %f", linear_tolerance_);
  pnh.param("angular_tolerance", angular_tolerance_,
            ANGULAR_TOLERANCE);
  ROS_INFO("   Angular tolerance: %f", angular_tolerance_);
  pnh.param("min_coil_signal", min_coil_signal_, MIN_COIL_SIGNAL);
  ROS_INFO("   Minimum coil signal: %f", min_coil_signal_);
  pnh.param("max_coil_signal", max_coil_signal_, MAX_COIL_SIGNAL);
  ROS_INFO("   Maximum coil signal: %f", max_coil_signal_);
  pnh.param("sample_time", sample_time_, SAMPLE_TIME);
  ROS_INFO("   Sample time: %f", sample_time_);
  pnh.param("safe_time", safe_time_, SAFE_TIME);
  ROS_INFO("   Safe time %f", safe_time_);
  pnh.param("rotation_time", rotation_time_, ROTATION_TIME);
  ROS_INFO("   Rotation time %f", rotation_time_);
  pnh.param("moving_away_time", moving_away_time_, MOVING_AWAY_TIME);
  ROS_INFO("   Moving away time %f", moving_away_time_);
  cmd_vel_pub_ = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
  moving_away_pub_ = nh->advertise<std_msgs::Bool>("moving_away", 1);
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
  if (!scanning_ && !moving_away_)
  {
    ROS_DEBUG("   Not scanning and not moving away!!!");
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
    current_state_ = states::S1_ALIGNING;
    break;
  case states::S1_ALIGNING:
    if (fabs(angular_error_) <= angular_tolerance_)
    {
      current_state_ = states::S2_SCANNING;
      linear_reference_ = max_coil_signal_;
    }
    break;
  case states::S2_SCANNING:
    if (coils_.getLeftValue() >= max_coil_signal_ ||
        coils_.getRightValue() >= max_coil_signal_)
    {
      current_state_ = states::S3_MOVING_BACK;
      linear_reference_ = min_coil_signal_;
    }
    break;
  case states::S3_MOVING_BACK:
    if (!coils_.isBothLow())
    {
      timer_ = ros::Time::now();
    }
    else if ((ros::Time::now() - timer_).toSec() > safe_time_)
    {
      timer_ = ros::Time::now();
      current_state_ = states::S4_CHANGING_DIRECTION;
    }
    break;
  case states::S4_CHANGING_DIRECTION:
    if ((ros::Time::now() - timer_).toSec() > rotation_time_)
    {
      timer_ = ros::Time::now();
      current_state_ = states::S5_MOVING_AWAY;
    }
    break;
  case states::S5_MOVING_AWAY:
    if ((ros::Time::now() - timer_).toSec() > rotation_time_)
    {
      reset();
    }
    break;
  }
}

/**
 * @brief MetalScanner::setVelocity
 */
void MetalScanner::setVelocity()
{
  // P controller for linear velocity
  linear_error_ = linear_reference_ - coils_.getMeanValue();
  double vx(linear_error_ * linear_Kp_);
  vx *= fabs(vx) > vx_ ? vx_ / fabs(vx) : 1;
  // P controller for angular velocity
  angular_error_ = coils_.getLeftValue() - coils_.getRightValue();
  double wz(angular_error_ * angular_Kp_);
  wz *= fabs(wz) > wz_ ? wz_ / fabs(wz) : 1;
  switch (current_state_)
  {
  case states::S0_SETTING_UP:
    ROS_INFO("   S0 - Setting up!");
    setVelocity(0, 0);
    break;
  case states::S1_ALIGNING:
    ROS_INFO("   S1 - Aligning!");
    setVelocity(0, wz);
    break;
  case states::S2_SCANNING:
    ROS_INFO("   S2 - Scanning!");
    setVelocity(vx, wz);
    break;
  case states::S3_MOVING_BACK:
    ROS_INFO("   S3 - Moving back!");
    setMovingAway(true);
    setVelocity(vx, wz);
    break;
  case states::S4_CHANGING_DIRECTION:
    ROS_INFO("   S4 - Changing direction!");
    setMovingAway(true);
    setVelocity(0, wz_);
    break;
  case states::S5_MOVING_AWAY:
    ROS_INFO("   S5 - Moving away!");
    setMovingAway(true);
    setVelocity(vx_, 0);
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
  moving_away_ = moving_away;
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
    //whenever emergent stop is needed while scanning or
    //whenever need to start scanning again while moving away
    if (scanning_ == moving_away_)
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
