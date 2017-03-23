/**
 *  This source file implements the DisplacementMonitor class.
 *
 *  Version: 1.1.3
 *  Created on: 21/03/2017
 *  Modified on: 21/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *          Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/displacement_monitor.h"

namespace hratc2017
{

/**
 * @brief DisplacementMonitor::DisplacementMonitor
 * @param nh
 */
DisplacementMonitor::DisplacementMonitor(ros::NodeHandle* nh,
                                         double linear_tolerance,
                                         double angular_tolerance)
    : linear_tolerance_(linear_tolerance),
      angular_tolerance_(angular_tolerance), setted_(false)
{
  odom_sub_ =
      nh->subscribe("odom", 1, &DisplacementMonitor::odomCallback, this);
  reset();
}

/**
 * @brief DisplacementMonitor::~DisplacementMonitor
 */
DisplacementMonitor::~DisplacementMonitor() { odom_sub_.shutdown(); }

/**
 * @brief DisplacementMonitor::isSetted
 * @return
 */
bool DisplacementMonitor::isSetted() const { return setted_; }

/**
 * @brief DisplacementMonitor::goalAchieved
 * @return
 */
bool DisplacementMonitor::goalAchieved() const
{
  return fabs(disp_x_ - goal_x_) <= linear_tolerance_ &&
         fabs(disp_y_ - goal_y_) <= linear_tolerance_ &&
         fabs(disp_phi_ - goal_phi_) <= angular_tolerance_;
}

/**
 * @brief DisplacementMonitor::getDispX
 * @return
 */
double DisplacementMonitor::getDispX() const { return disp_x_; }

/**
 * @brief DisplacementMonitor::getDispY
 * @return
 */
double DisplacementMonitor::getDispY() const { return disp_y_; }

/**
 * @brief DisplacementMonitor::getDispPhi
 * @return
 */
double DisplacementMonitor::getDispPhi() const { return disp_phi_; }

/**
 * @brief DisplacementMonitor::setGoal
 * @param x
 * @param y
 * @param phi
 */
void DisplacementMonitor::setGoal(double x, double y, double phi)
{
  goal_x_ = x;
  goal_y_ = y;
  goal_phi_ = phi;
  while (goal_phi_ <= -M_PI)
  {
    goal_phi_ += 2 * M_PI;
  }
  while (goal_phi_ > M_PI)
  {
    goal_phi_ -= 2 * M_PI;
  }
}

/**
 * @brief DisplacementMonitor::setLinearTolerance
 * @param tol
 */
void DisplacementMonitor::setLinearTolerance(double tol)
{
  linear_tolerance_ = tol;
}

/**
 * @brief DisplacementMonitor::setAngularTolerance
 * @param tol
 */
void DisplacementMonitor::setAngularTolerance(double tol)
{
  angular_tolerance_ = tol;
}

/**
 * @brief DisplacementMonitor::reset
 */
void DisplacementMonitor::reset()
{
  start_x_ = curr_x_;
  start_y_ = curr_y_;
  start_phi_ = curr_phi_;
  disp_x_ = 0.0;
  disp_y_ = 0.0;
  disp_phi_ = 0.0;
}

void DisplacementMonitor::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  curr_x_ = msg->pose.pose.position.x;
  curr_y_ = msg->pose.pose.position.y;
  curr_phi_ = tf::getYaw(msg->pose.pose.orientation);
  if (!setted_)
  {
    ROS_INFO("   Odometry initialized!!!");
    setted_ = true;
    prev_phi_ = curr_phi_;
    reset();
  }
  disp_x_ = (curr_x_ - start_x_) * cos(-start_phi_) -
            (curr_y_ - start_y_) * sin(-start_phi_);
  disp_y_ = (curr_y_ - start_y_) * cos(-start_phi_) +
            (curr_x_ - start_x_) * sin(-start_phi_);
  while (curr_phi_ - prev_phi_ < M_PI)
  {
    curr_phi_ += 2 * M_PI;
  }
  while (curr_phi_ - prev_phi_ > M_PI)
  {
    curr_phi_ -= 2 * M_PI;
  }
  disp_phi_ += curr_phi_ - prev_phi_;
  prev_phi_ = curr_phi_;
}
}