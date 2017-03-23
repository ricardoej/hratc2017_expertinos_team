/**
 *  This source file implements the MetalScanner class, which is
 *based on the ROSNode helper class. It controls the metal_scanner_node.
 *
 *  Version: 1.1.4
 *  Created on: 09/02/2017
 *  Modified on: 21/03/2017
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
    : ROSNode(nh, 30), coils_(nh), disp_monitor_(nh),
      current_state_(states::S0_SETTING_UP), angular_error_(0), timer_(0),
      scanning_(false), moving_away_(false)
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
  pnh.param("linear_tolerance", linear_tolerance_, LINEAR_TOLERANCE);
  ROS_INFO("   Linear tolerance: %f", linear_tolerance_);
  disp_monitor_.setLinearTolerance(linear_tolerance_);
  pnh.param("angular_tolerance", angular_tolerance_, ANGULAR_TOLERANCE);
  ROS_INFO("   Angular tolerance: %f", angular_tolerance_);
  disp_monitor_.setAngularTolerance(angular_tolerance_);
  pnh.param("min_coil_signal", min_coil_signal_, MIN_COIL_SIGNAL);
  ROS_INFO("   Minimum coil signal: %f", min_coil_signal_);
  pnh.param("max_coil_signal", max_coil_signal_, MAX_COIL_SIGNAL);
  ROS_INFO("   Maximum coil signal: %f", max_coil_signal_);
  pnh.param("pause_time", pause_time_, PAUSE_TIME);
  ROS_INFO("   Pause time %f", pause_time_);
  pnh.param("std_radius", std_radius_, STANDARD_RADIUS);
  ROS_INFO("   Standard radius of separation: %f", std_radius_);
  cmd_vel_pub_ = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
  moving_away_pub_ = nh->advertise<std_msgs::Bool>("moving_away", 1);
  known_landmine_pub_ = nh->advertise<std_msgs::Bool>("known_mine", 1, true);
  coils_sub_ = nh->subscribe("/coils", 10, &Coils::coilsCallback, &coils_);
  scanning_sub_ =
      nh->subscribe("scanning", 1, &MetalScanner::scanningCallback, this);
  mines_sub_ = nh->subscribe("/HRATC_FW/set_mine", 10,
                             &MetalScanner::minesCallback, this);
  fake_mines_sub_ = nh->subscribe("/HRATC_FW/set_fake_mine", 10,
                                  &MetalScanner::fakeMinesCallback, this);
  reset();
}

/**
 * @brief MetalScanner::~MetalScanner
 */
MetalScanner::~MetalScanner()
{
  cmd_vel_pub_.shutdown();
  moving_away_pub_.shutdown();
  known_landmine_pub_.shutdown();
  coils_sub_.shutdown();
  scanning_sub_.shutdown();
  mines_sub_.shutdown();
  fake_mines_sub_.shutdown();
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
  bool known(isKnownMine());
  publishKnownMine(known);
  setNextState();
  setVelocity();
}

/**
 * @brief MetalScanner::isSetted
 * @return
 */
bool MetalScanner::isSettedUp()
{
  return disp_monitor_.isSettedUp() && coils_.isSettedUp();
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
    ROS_INFO("   S0_SETTING_UP  -->  S1_ALIGNING");
    break;
  case states::S1_ALIGNING:
    if (fabs(angular_error_) <= angular_tolerance_)
    {
      linear_reference_ = max_coil_signal_;
      current_state_ = states::S2_SCANNING_FORWARD;
      ROS_INFO("   S1_ALIGNING  -->  S2_SCANNING_FORWARD");
    }
    break;
  case states::S2_SCANNING_FORWARD:
    if (coils_.isLeftHigh() || coils_.isRightHigh())
    {
      linear_reference_ = min_coil_signal_;
      timer_ = ros::Time::now();
      current_state_ = states::S3_HOLDING_ON;
      ROS_INFO("   S2_SCANNING  -->  S3_HOLDING_ON");
      bool known(isKnownMine());
      if (known)
      {
        ROS_INFO("   Known Mine  -->  S4_SCANNING_BACK");
        current_state_ = states::S4_SCANNING_BACK;
      }
    }
    break;
  case states::S3_HOLDING_ON:
    if ((ros::Time::now() - timer_).toSec() > pause_time_)
    {
      current_state_ = states::S4_SCANNING_BACK;
      ROS_INFO("   S3_HOLDING_ON  -->  S4_SCANNING_BACK");
    }
    break;
  case states::S4_SCANNING_BACK:
    if (coils_.isBothLow())
    {
      disp_monitor_.reset();
      disp_monitor_.setGoal(S5_MOVING_BACK_X);
      current_state_ = states::S5_MOVING_BACK;
      ROS_INFO("   S4_SCANNING_BACK  -->  S5_MOVING_BACK");
    }
    break;
  case states::S5_MOVING_BACK:
    if (disp_monitor_.goalAchieved())
    {
      disp_monitor_.reset();
      disp_monitor_.setGoal(0.0, S6_CHANGING_DIRECTION_PHI);
      current_state_ = states::S6_CHANGING_DIRECTION;
      ROS_INFO("   S5_MOVING_BACK  -->  S6_CHANGING_DIRECTION");
    }
    break;
  case states::S6_CHANGING_DIRECTION:
    if (disp_monitor_.goalAchieved())
    {
      disp_monitor_.reset();
      disp_monitor_.setGoal(0.0, S7_MOVING_AWAY_X);
      current_state_ = states::S7_MOVING_AWAY;
      ROS_INFO("   S5_CHANGING_DIRECTION  -->  S6_MOVING_AWAY");
    }
    break;
  case states::S7_MOVING_AWAY:
    if (disp_monitor_.goalAchieved())
    {
      reset();
      ROS_INFO("   S6_MOVING_AWAY  -->  S0_SETTING_UP");
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
    ROS_DEBUG("   S0 - Setting up!");
    setVelocity(0, 0);
    break;
  case states::S1_ALIGNING:
    ROS_DEBUG("   S1 - Aligning!");
    setVelocity(0, wz);
    break;
  case states::S2_SCANNING_FORWARD:
    ROS_DEBUG("   S2 - Scanning forward!");
    setVelocity(vx, wz);
    break;
  case states::S3_HOLDING_ON:
    ROS_DEBUG("   S3 - Holding on!");
    setVelocity(0, 0);
    break;
  case states::S4_SCANNING_BACK:
    ROS_DEBUG("   S4 - Scanning back!");
    // setMovingAway(true);
    setVelocity(vx, wz);
    break;
  case states::S5_MOVING_BACK:
    ROS_DEBUG("   S5 - Moving back!");
    // setMovingAway(true);
    setVelocity(-vx_, 0);
    break;
  case states::S6_CHANGING_DIRECTION:
    ROS_DEBUG("   S6 - Changing direction!");
    setMovingAway(true);
    setVelocity(0, -wz_);
    break;
  case states::S7_MOVING_AWAY:
    ROS_DEBUG("   S7 - Moving away!");
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
    // whenever emergent stop is needed while scanning or
    // whenever need to start scanning again while moving away
    if (scanning_ == moving_away_)
    {
      reset();
    }
  }
}

/**
 * @brief MetalScanner::minesCallback
 * @param msg
 */
void MetalScanner::minesCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (!isKnownMine(msg->pose.position))
  {
    mines_.push_back(msg->pose.position);
  }
  for (int i(0); i < mines_.size(); i++)
  {
    if (utilities::Points::getEuclidianDistance(
            mines_[i], msg->pose.position) <= std_radius_)
    {
      mines_[i] =
          utilities::Points::getMidstPoint(mines_[i], msg->pose.position);
      return;
    }
  }
}

/**
 * @brief MetalScanner::fakeMinesCallback
 * @param msg
 */
void MetalScanner::fakeMinesCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (!isKnownFakeMine(msg->pose.position))
  {
    fake_mines_.push_back(msg->pose.position);
    return;
  }
  ROS_ERROR("[FakeMineCB] Already known fake mine @ (%lf, %lf) with %lf",
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  for (int i(0); i < fake_mines_.size(); i++)
  {
    if (utilities::Points::getEuclidianDistance(
            fake_mines_[i], msg->pose.position) <= fake_mines_[i].z)
    {
      ROS_ERROR("[FakeMineCB] Old fake mine position @ (%lf, %lf) with %lf",
                fake_mines_[i].x, fake_mines_[i].y, fake_mines_[i].z);
      fake_mines_[i] =
          utilities::Points::getMidstPoint(fake_mines_[i], msg->pose.position);
      if (fake_mines_[i].z < msg->pose.position.z)
      {
        fake_mines_[i].z = msg->pose.position.z;
      }
      fake_mines_[i].z += utilities::Points::getEuclidianDistance(
                              fake_mines_[i], msg->pose.position) /
                          2;
      ROS_ERROR("[FakeMineCB] New fake mine position @ (%lf, %lf) with %lf",
                fake_mines_[i].x, fake_mines_[i].y, fake_mines_[i].z);
      return;
    }
  }
}

/**
 * @brief MetalScanner::publishKnownMine
 * @param known
 */
void MetalScanner::publishKnownMine(bool known)
{
  std_msgs::Bool msg;
  msg.data = known;
  known_landmine_pub_.publish(msg);
}

/**
 * @brief MetalScanner::reset
 */
void MetalScanner::reset()
{
  ROSNode::reset();
  scanning_ = false;
  current_state_ = states::S0_SETTING_UP;
  setMovingAway(false);
  setVelocity(0, 0);
  disp_monitor_.reset();
}

/**
 * @brief MetalScanner::isKnownMine
 * @return
 */
bool MetalScanner::isKnownMine() const
{
  return isKnownMine(coils_.getMidstPose().pose.position);
}

/**
 * @brief MetalScanner::isKnownMine
 * @param p
 * @return
 */
bool MetalScanner::isKnownMine(geometry_msgs::Point p) const
{
  return isKnownMine(p.x, p.y);
}

/**
 * @brief MetalScanner::isKnownMine
 * @param x
 * @param y
 * @return
 */
bool MetalScanner::isKnownMine(double x, double y) const
{
  for (int i(0); i < mines_.size(); i++)
  {
    if (utilities::Points::getEuclidianDistance(mines_[i], x, y) <= std_radius_)
    {
      return true;
    }
  }
  return false;
}

/**
 * @brief MetalScanner::isKnownFakeMine
 * @return
 */
bool MetalScanner::isKnownFakeMine() const
{
  return isKnownFakeMine(coils_.getMidstPose().pose.position);
}

/**
 * @brief MetalScanner::isKnownFakeMine
 * @param p
 * @return
 */
bool MetalScanner::isKnownFakeMine(geometry_msgs::Point p) const
{
  return isKnownFakeMine(p.x, p.y);
}

/**
 * @brief MetalScanner::isKnownFakeMine
 * @param x
 * @param y
 * @return
 */
bool MetalScanner::isKnownFakeMine(double x, double y) const
{
  for (int i(0); i < fake_mines_.size(); i++)
  {
    if (utilities::Points::getEuclidianDistance(fake_mines_[i], x, y) <=
        fake_mines_[i].z)
    {
      return true;
    }
  }
  return false;
}
}
