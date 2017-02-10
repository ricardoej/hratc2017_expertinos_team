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
MetalScanner::MetalScanner(ros::NodeHandle* nh) : ROSNode(nh, 30)
{
  ros::NodeHandle pnh("~");

  pnh.param("linear_velocity_x", vx_, LINEAR_VELOCITY_X);
  ROS_INFO("   Linear velocity x: %f", vx_);

  pnh.param("angular_velocity_z", wx_, ANGULAR_VELOCITY_Z);
  ROS_INFO("   Angular velocity z: %f", wz_);

  pnh.param("Kp", Kp_, KP);
  ROS_INFO("   Kp: %f", Kp_);

  pnh.param("min_coil_signal", min_coil_signal_, MIN_COIL_SIGNAL);
  ROS_INFO("   Minimum coil signal: %f", min_coil_signal_);

  pnh.param("max_coil_signal", max_coil_signal_, MAX_COIL_SIGNAL);
  ROS_INFO("   Maximum coil signal: %f", max_coil_signal_);

  pnh.param("coil_signal_increment", coil_signal_increment_, COIL_SIGNAL_INCREMENT);
  ROS_INFO("   Coil signal increment: %f", coil_signal_increment_);

  cmd_vel_pub_ =
      nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
  coils_sub_ =
      nh->subscribe("/coils", 10, &MetalScanner::coilsCallback, this);
}

/**
 * @brief MetalScanner::~MetalScanner
 */
MetalScanner::~MetalScanner()
{
  cmd_vel_pub_.shutdown();
  coils_sub_.shutdown();
}

/**
 * @brief MetalScanner::controlLoop
 */
void MetalScanner::controlLoop()
{

  setVelocity(1,0);
}

StateEnum MetalScanner::getNextState()
{
  current_state_ =
}

void MetalScanner::setVelocity()
{
  switch (current_state_) {
  case states::S0:  //P controller is implemented here
    double wz((coils_.getLeft() - coils_.getRight()) * Kp_);
    setVelocity(0, wz * (fabs(wz) > wz_ ? wz_ / fabs(wz) : 1));
    break;
  case states::S1:
    setVelocity(vx_, 0);
    break;
  case states::S2:
    setVelocity(0, wz_);
    break;
  case states::S3:
    setVelocity(0, -wz_);
    break;
  case states::S4:
    setVelocity(0, wz_);
    break;
  case states::S5:
    setVelocity(-vx_, 0);
  default:
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

void MetalScanner::coilsCallback(const metal_detector_msgs::Coil_::ConstPtr &msg)
{
  coils_ = msg;
}
}
