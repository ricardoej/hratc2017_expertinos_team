/**
 *  This header file defines the WaypointsController class, which is based
 *on the ROSNode class. It controls the waypoints_controller_node.
 *
 *  Version: 1.0.1
 *  Created on: 06/02/2017
 *  Modified on: 02/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *          Ricardo Emerson Julio (ricardoej@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_ENTRIES_WAYPOINTS_CONTROLLER_H_
#define _HRATC2017_ENTRIES_WAYPOINTS_CONTROLLER_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include "hratc2017/map_coverage.h"
#include "utilities/ros_node.h"

namespace hratc2017
{

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

class WaypointsController : public utilities::ROSNode
{
public:
  WaypointsController(ros::NodeHandle* nh);
  virtual ~WaypointsController();

private:
  MoveBaseClient move_base_client_;
  ros::Subscriber corners_sub_;
  ros::Subscriber scanning_sub_;
  ros::Subscriber set_mine_sub_;
  ros::Publisher waypoints_pub_;
  MapCoverage* map_;
  bool active_goal_;
  bool scanning_;
  virtual void controlLoop();
  void sendGoal(const geometry_msgs::Pose& target_pose);
  void sendGoal(const geometry_msgs::Point& waypoint);
  void sendGoal(double x, double y, double theta = 0.0);
  void cornersCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void scanningCallback(const std_msgs::Bool::ConstPtr& msg);
  void setMineCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void goalActiveCallback();
  void feedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback);
  void resultCallback(const actionlib::SimpleClientGoalState& state,
                      const move_base_msgs::MoveBaseResult::ConstPtr& result);
  void publishWaypoint(const geometry_msgs::Point& waypoint) const;
};
}

#endif /* _HRATC2017_ENTRIES_WAYPOINTS_CONTROLLER_H_ */
