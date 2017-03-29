/**
 *  This header file implements the DisplacementMonitor class.
 *
 *  Version: 1.1.3
 *  Created on: 21/03/2017
 *  Modified on: 21/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *          Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_DISPLACEMENT_MONITOR_H_
#define _HRATC2017_DISPLACEMENT_MONITOR_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#define LINEAR_TOLERANCE 0.05
#define ANGULAR_TOLERANCE 0.01

namespace hratc2017
{
class DisplacementMonitor
{
public:
  DisplacementMonitor(ros::NodeHandle* nh,
                      double linear_tolerance = LINEAR_TOLERANCE,
                      double angular_tolerance = ANGULAR_TOLERANCE);
  virtual ~DisplacementMonitor();
  bool isSettedUp() const;
  bool isGoalSetted() const;
  bool goalAchieved() const;
  bool goalXAchieved() const;
  bool goalYAchieved() const;
  bool goalPhiAchieved() const;
  double getDispX() const;
  double getDispY() const;
  double getDispPhi() const;
  double getXError() const;
  double getYError() const;
  double getPhiError() const;
  double getXGoal() const;
  double getYGoal() const;
  double getPhiGoal() const;
  void setGoal(double x, double phi = 0.0);
  void setGoal(double x, double y, double phi);
  void setLinearTolerance(double tol);
  void setAngularTolerance(double tol);
  void reset();

private:
  ros::Subscriber odom_sub_;
  bool setted_up_;
  bool goal_setted_;
  double start_x_, start_y_, start_phi_;
  double curr_x_, curr_y_, curr_phi_, prev_phi_;
  double disp_x_, disp_y_, disp_phi_;
  double goal_x_, goal_y_, goal_phi_;
  double linear_tolerance_, angular_tolerance_;
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};
}

#endif // _HRATC2017_DISPLACEMENT_MONITOR_H_
