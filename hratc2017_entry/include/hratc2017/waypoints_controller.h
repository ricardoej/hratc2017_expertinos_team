/**
 *  This header file defines the WaypointsController class, which is based
 *on the ROSNode class. It controls the waypoints_controller_node.
 *
 *  Version: 0.0.1
 *  Created on: 06/02/2017
 *  Modified on: 06/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *          Ricardo Emerson Julio (ricardoej@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_ENTRIES_WAYPOINTS_CONTROLLER_H_
#define _HRATC2017_ENTRIES_WAYPOINTS_CONTROLLER_H_

#include <ros/ros.h>
#include "hratc2017/map.h"
#include "utilities/ros_node.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "visualization_msgs/MarkerArray.h"
#include <queue>
#include "geometry_msgs/Point.h"
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#define DEFAULT_MAP_COVERAGE_OFFSET 1.5

namespace hratc2017
{

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class WaypointsController : public utilities::ROSNode
{
public:
  WaypointsController(ros::NodeHandle* nh);
  virtual ~WaypointsController();

private:
  MoveBaseClient move_base_client_;
  ros::Subscriber corners_sub_;
  Map* map_;
  std::queue<geometry_msgs::Point> waypoints_;
  bool hasActiveGoal_;
  virtual void controlLoop();
  void createMap(ros::NodeHandle* nh);
  void createStrategy();
  void mapCornersCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void goalDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr &result);
  void goalActiveCallback();
  void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr &feedback);
};
}

#endif /* _HRATC2017_ENTRIES_WAYPOINTS_CONTROLLER_H_ */
