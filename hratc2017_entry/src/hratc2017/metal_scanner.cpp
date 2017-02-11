/**
 *  This source file implements the MetalScanner class, which is
 *based on the ROSNode helper class. It controls the metal_scanner_node.
 *
 *  Version: 0.0.1
 *  Created on: 09/02/2017
 *  Modified on: 09/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *          Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/metal_scanner.h"

namespace hratc2017
{

/**
 * @brief MetalScanner::MetalScanner
 * @param nh
 */
MetalScanner::MetalScanner(ros::NodeHandle* nh) : ROSNode(nh, 30), current_state_(states::S0_SETTING_UP), error_(0), paused_(true)
{
  ros::NodeHandle pnh("~");
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
  pnh.param("coil_signal_increment", coil_signal_increment_, COIL_SIGNAL_INCREMENT);
  ROS_INFO("   Coil signal increment: %f", coil_signal_increment_);
  pnh.param("coil_signal_tolerance", coil_signal_tolerance_, COIL_SIGNAL_TOLERANCE);
  ROS_INFO("   Coil signal tolerance: %f", coil_signal_tolerance_);
  pnh.param("safe_coil_signal", safe_coil_signal_, SAFE_COIL_SIGNAL);
  ROS_INFO("   Safe coil signal %f", safe_coil_signal_);
  pnh.param("threshold", threshold_, THRESHOLD);
  ROS_INFO("   Threshold %f", threshold_);
  pnh.param("safe_time", safe_time_, SAFE_TIME);
  ROS_INFO("   Safe_time %f", safe_time_);
  cmd_vel_pub_ =
      nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
  coils_sub_ =
      nh->subscribe("/coils", 10, &MetalScanner::coilsCallback, this);
  pause_pub_ =
      nh->advertise<std_msgs::Bool>("pause_scanning", 1);
  pause_sub_=
      nh->subscribe("pause_scanning", 1, &MetalScanner::pauseCallback, this);
}

/**
 * @brief MetalScanner::~MetalScanner
 */
MetalScanner::~MetalScanner()
{
  cmd_vel_pub_.shutdown();
  coils_sub_.shutdown();
  pause_pub_.shutdown();
  pause_sub_.shutdown();
}

/**
 * @brief MetalScanner::controlLoop
 */
void MetalScanner::controlLoop()
{
  if(paused_)
  {
    ROS_INFO("   Paused!!!");
    return;
  }
  setNextState();
  setVelocity();
}

StateEnum MetalScanner::setNextState()
{
  switch (current_state_)
  {
  case states::S0_SETTING_UP:
    ref_coil_signal_ = min_coil_signal_ - coil_signal_increment_;
    ROS_INFO("   S0 - Change State!");
    current_state_ = states::S1_ALINGING;
    break;
  case states::S1_ALINGING:
    if(fabs(error_) <= coil_signal_tolerance_)
    {
      ref_coil_signal_ += coil_signal_increment_;
      ROS_INFO("   S1 - State change!");
      current_state_ = ref_coil_signal_ <= max_coil_signal_ ? states::S2_SCANNING_FOWARD : states::S5_MOVING_AWAY;
    }
    break;
  case states::S2_SCANNING_FOWARD:
    if(coils_.getLeft() >= ref_coil_signal_ || coils_.getRight() >= ref_coil_signal_)
    {
      ROS_INFO("   S2 - State change!");
      current_state_ = states::S3_SCANNING_LEFT;
    }
    break;
  case states::S3_SCANNING_LEFT:
    if(coils_.getLeft() <= threshold_)
    {
      ROS_INFO("   S3 - State change!");
      current_state_ = states::S4_SCANNING_RIGHT;
    }
    break;
  case states::S4_SCANNING_RIGHT:
    if(coils_.getRight() <= threshold_)
    {
      ROS_INFO("   S4 - State change!");
      current_state_ = states::S1_ALINGING;
    }
    break;
  case states::S5_MOVING_AWAY:
    if(coils_.getLeft() < safe_coil_signal_ && coils_.getRight() < safe_coil_signal_)
    {
      ROS_INFO("   S5 - Waiting safe time!");
      ros::Duration(safe_time_).sleep();
      setVelocity(0, 0);
      ROS_INFO("   S5 - State change!");
      current_state_ = states::S0_SETTING_UP;
      setPause(true);
    }
    break;
  }
}

void MetalScanner::setVelocity()
{
  double wz;

  switch (current_state_)
  {
  case states::S0_SETTING_UP:
    ROS_INFO("   S0 - Setting up!");
    setVelocity(0, 0);
    break;
  case states::S1_ALINGING:  //P controller is implemented here
    ROS_INFO("   S1 - Alinging!");
    error_ = coils_.getLeft() - coils_.getRight();
    wz = error_ * Kp_;
    setVelocity(0, wz * (fabs(wz) > wz_ ? wz_ / fabs(wz) : 1));
    break;
  case states::S2_SCANNING_FOWARD:
    ROS_INFO("   S2 - Scanning foward!");
    setVelocity(vx_, 0);
    break;
  case states::S3_SCANNING_LEFT:
    ROS_INFO("   S3 - Scanning left!");
    setVelocity(0, wz_);
    break;
  case states::S4_SCANNING_RIGHT:
    ROS_INFO("   S4 - Scanning right!");
    setVelocity(0, -wz_);
    break;
  case states::S5_MOVING_AWAY:
    ROS_INFO("   S5 - Moving away!");
    setVelocity(-vx_, 0);
    break;
  }
}

void MetalScanner::setVelocity(double vx, double wz)
{
  geometry_msgs::Twist msg;
  msg.linear.x = vx;
  msg.angular.z = wz;
  cmd_vel_pub_.publish(msg);
}

void MetalScanner::pauseCallback(const std_msgs::Bool::ConstPtr &msg)
{
  paused_ = msg->data;
  ROS_INFO("   pauseCallBack: %s", paused_ ? "true" : "false");
}

void MetalScanner::coilsCallback(const metal_detector_msgs::Coil::ConstPtr &msg)
{
  coils_ = msg;
}

void MetalScanner::setPause(bool paused)
{
  std_msgs::Bool msg;
  msg.data = paused;
  paused_ = paused;
}
}
