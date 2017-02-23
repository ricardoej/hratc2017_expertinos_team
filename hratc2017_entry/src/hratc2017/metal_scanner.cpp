/**
 *  This source file implements the MetalScanner class, which is
 *based on the ROSNode helper class. It controls the metal_scanner_node.
 *
 *  Version: 0.0.1
 *  Created on: 09/02/2017
 *  Modified on: 13/02/2017
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
      scanning_(false)
{
  ros::NodeHandle pnh("~");
  double threshold;
  pnh.param("coil_signal_threshold", threshold, COIL_SIGNAL_THRESHOLD);
  coils_.setThreshold(threshold);
  int number_of_observations;
  pnh.param("coil_signal_filter_number_of_observations", number_of_observations, COIL_SIGNAL_FILTER_NUMBER_OF_OBSERVATIONS);
  coils_.setNumberOfObservations(number_of_observations);
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
  pnh.param("safe_time", safe_time_, SAFE_TIME);
  ROS_INFO("   Safe_time %f", safe_time_);
  cmd_vel_pub_ = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
  coils_sub_ = nh->subscribe("/coils", 10, &Coils::coilsCallback, &coils_);
  scanning_sub_ =
      nh->subscribe("start_scanning", 1, &MetalScanner::scanningCallback, this);
}

/**
 * @brief MetalScanner::~MetalScanner
 */
MetalScanner::~MetalScanner()
{
  cmd_vel_pub_.shutdown();
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
    ROS_INFO("   S0 - Change State!");
    current_state_ = states::S1_ALINGING;
    break;
  case states::S1_ALINGING:
    if (fabs(error_) <= coil_signal_tolerance_)
    {
      ref_coil_signal_ += coil_signal_increment_;
      ROS_INFO("   S1 - State change!");
      current_state_ = ref_coil_signal_ <= max_coil_signal_
                           ? states::S2_SCANNING_FOWARD
                           : states::S5_MOVING_AWAY;
    }
    break;
  case states::S2_SCANNING_FOWARD:
    if (coils_.getLeft() >= ref_coil_signal_ ||
        coils_.getRight() >= ref_coil_signal_)
    {
      ROS_INFO("   S2 - State change!");
      current_state_ = states::S3_SCANNING_LEFT;
    }
    break;
  case states::S3_SCANNING_LEFT:
    if (!coils_.isHighCoilSignalOnLeft())
    {
      ROS_INFO("   S3 - State change!");
      current_state_ = states::S4_SCANNING_RIGHT;
    }
    break;
  case states::S4_SCANNING_RIGHT:
    if (!coils_.isHighCoilSignalOnRight())
    {
      ROS_INFO("   S4 - State change!");
      current_state_ = states::S1_ALINGING;
    }
    break;
  case states::S5_MOVING_AWAY:
    if (coils_.getLeft() < safe_coil_signal_ && coils_.getRight() < safe_coil_signal_)
    {
      s6_timer_ = ros::Time::now();
      current_state_ = states::S0_SETTING_UP;
      //current_state_ = states::S6_CHANGING_DIRECTION;
      ROS_INFO("   S5 - State change!");
      reset();
    }
    break;
  case states::S6_CHANGING_DIRECTION:
    if((ros::Time::now() - s6_timer_).toSec() > safe_time_)
    {
      ROS_INFO("   S6 - State change!");
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
  double wz;

  switch (current_state_)
  {
  case states::S0_SETTING_UP:
    ROS_DEBUG("   S0 - Setting up!");
    setVelocity(0, 0);
    break;
  case states::S1_ALINGING: // P controller is implemented here
    ROS_DEBUG("   S1 - Alinging!");
    error_ = coils_.getLeft() - coils_.getRight();
    wz = error_ * Kp_;
    setVelocity(0, wz * (fabs(wz) > wz_ ? wz_ / fabs(wz) : 1));
    break;
  case states::S2_SCANNING_FOWARD:
    ROS_DEBUG("   S2 - Scanning foward!");
    setVelocity(vx_, 0);
    break;
  case states::S3_SCANNING_LEFT:
    ROS_DEBUG("   S3 - Scanning left!");
    setVelocity(0, wz_);
    break;
  case states::S4_SCANNING_RIGHT:
    ROS_DEBUG("   S4 - Scanning right!");
    setVelocity(0, -wz_);
    break;
  case states::S5_MOVING_AWAY:
    ROS_DEBUG("   S5 - Moving away!");
    setVelocity(-vx_, 0);
    break;
   case states::S6_CHANGING_DIRECTION:
    ROS_DEBUG("   S6 - Moving away!");
    setVelocity(0, wz_);
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
 * @brief MetalScanner::pauseCallback
 * @param msg
 */
void MetalScanner::scanningCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if(scanning_ != msg->data)
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
 * @brief MetalScanner::reset
 */
void MetalScanner::reset()
{
  scanning_ = false;
  current_state_ = states::S0_SETTING_UP;
  setVelocity(0,0);
}
}
