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
#include <std_srvs/Trigger.h>

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
  ros::Subscriber moving_away_sub_;
  ros::Subscriber avoiding_obstacle_sub_;
  ros::Subscriber set_mine_sub_;
  ros::Publisher waypoints_pub_;
  ros::ServiceServer start_srv_;
  MapCoverage* map_;
  bool active_goal_;
  bool avoiding_obstacle_;
  bool scanning_;
  bool moving_away_;
  bool start_;
  virtual bool isSettedUp();
  virtual void controlLoop();
  bool ok() const;
  void sendGoal(const geometry_msgs::Pose& target_pose);
  void sendGoal(const geometry_msgs::Point& waypoint);
  void sendGoal(double x, double y, double theta = 0.0);
  void cornersCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void avoidingObstacleCallback(const std_msgs::Bool::ConstPtr& msg);
  void scanningCallback(const std_msgs::Bool::ConstPtr& msg);
  void movingAwayCallback(const std_msgs::Bool::ConstPtr& msg);
  bool startCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  void setMineCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void goalActiveCallback();
  void feedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback);
  void resultCallback(const actionlib::SimpleClientGoalState& state,
                      const move_base_msgs::MoveBaseResult::ConstPtr& result);
  void publishWaypoint(const geometry_msgs::Point& waypoint) const;
};
}

#endif /* _HRATC2017_ENTRIES_WAYPOINTS_CONTROLLER_H_ */
