/**
 *  This source file implements the WaypointsController class, which is
 *based on the ROSNode helper class. It controls the waypoints_controller_node.
 *
 *  Version: 1.0.1
 *  Created on: 06/02/2017
 *  Modified on: 02/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *          Ricardo Emerson Julio (ricardoej@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/waypoints_controller.h"

namespace hratc2017
{

/**
 * @brief WaypointsController::WaypointsController
 * @param nh
 */
WaypointsController::WaypointsController(ros::NodeHandle* nh)
    : ROSNode(nh, 2), map_(NULL), move_base_client_("/move_base", true),
      active_goal_(false), avoiding_obstacle_(false), scanning_(false), moving_away_(false)
{
  corners_sub_ =
      nh->subscribe("/corners", 1, &WaypointsController::cornersCallback, this);
  avoiding_obstacle_sub_ = nh->subscribe("avoiding_obstacle", 1,
                                &WaypointsController::avoidingObstacleCallback, this);
  scanning_sub_ = nh->subscribe("scanning", 1,
                                &WaypointsController::scanningCallback, this);
  moving_away_sub_ = nh->subscribe("moving_away", 1,
                                &WaypointsController::movingAwayCallback, this);
  set_mine_sub_ = nh->subscribe("/HRATC_FW/set_mine", 1,
                                &WaypointsController::setMineCallback, this);
  waypoints_pub_ =
      nh->advertise<visualization_msgs::Marker>("waypoint_markers", 0);
}

/**
 * @brief WaypointsController::~WaypointsController
 */
WaypointsController::~WaypointsController()
{
  corners_sub_.shutdown();
  scanning_sub_.shutdown();
  moving_away_sub_.shutdown();
  waypoints_pub_.shutdown();
  if (map_)
  {
    delete map_;
    map_ = NULL;
  }
}

/**
 * @brief WaypointsController::controlLoop
 */
void WaypointsController::controlLoop()
{
  if (!move_base_client_.waitForServer(ros::Duration(0.5)))
  {
    ROS_WARN("Waiting for the move_base action server to come up.");
    return;
  }
  else
  {
    if (!ok() && active_goal_)
    {
      move_base_client_.cancelAllGoals();
      ROS_INFO("Sent a cancel all goals.");
    }
    else if (ok() && !active_goal_ && map_ && !map_->empty())
    {
      sendGoal(map_->getNextWaypoint());
      publishWaypoint(map_->getNextWaypoint());
    }
  }
}

/**
 * @brief WaypointsController::ok
 */
bool WaypointsController::ok() const
{
  return !scanning_ && !moving_away_ && !avoiding_obstacle_;
}

/**
 * @brief WaypointsController::sendGoal
 * @param x
 * @param y
 * @param theta
 */
void WaypointsController::sendGoal(const geometry_msgs::Pose& target_pose)
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = target_pose;
  move_base_client_.sendGoal(
      goal, boost::bind(&WaypointsController::resultCallback, this, _1, _2),
      boost::bind(&WaypointsController::goalActiveCallback, this),
      boost::bind(&WaypointsController::feedbackCallback, this, _1));
}

/**
 * @brief WaypointsController::sendGoal
 * @param target_position
 */
void WaypointsController::sendGoal(const geometry_msgs::Point& waypoint)
{
  sendGoal(waypoint.x, waypoint.y);
}

/**
 * @brief WaypointsController::sendGoal
 * @param x
 * @param y
 * @param theta
 */
void WaypointsController::sendGoal(double x, double y, double theta)
{
  ROS_INFO("Sent new goal @ (%f [m], %f [m], %f [rad])", x, y, theta);
  geometry_msgs::Pose target_pose;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.orientation = tf::createQuaternionMsgFromYaw(theta);
  sendGoal(target_pose);
}

/**
 * @brief WaypointsController::mapCornersCallback receives the map borders
 * @param msg map corner position.
 */
void WaypointsController::cornersCallback(
    const visualization_msgs::MarkerArray::ConstPtr& corners)
{
  if (map_ && !map_->empty())
  {
    ROS_WARN("Ignoring the new message in /corners topic. There still exists "
             "waypoint(s) to pass through.");
    return;
  }
  if (map_)
  {
    delete map_;
    map_ = NULL;
  }
  ros::NodeHandle pnh("~");
  double map_coverage_offset;
  pnh.param("map_coverage_offset", map_coverage_offset,
            DEFAULT_MAP_COVERAGE_OFFSET);
  double map_coverage_margin;
  pnh.param("map_coverage_margin", map_coverage_margin,
            DEFAULT_MAP_COVERAGE_MARGIN);
  double landmine_radius_area;
  pnh.param("landmine_radius_area", landmine_radius_area,
            DEFAULT_LANDMINE_RADIUS_AREA);
  try
  {
    map_ =
        new MapCoverage(corners, map_coverage_offset, map_coverage_margin, landmine_radius_area);
    ROS_INFO("%s", map_->c_str());
  }
  catch (utilities::Exception ex)
  {
    ROS_FATAL("Exception catched: %s", ex.what());
  }
}

/**
 * @brief WaypointsController::avoidingObstacleCallback
 * @param msg
 */
void WaypointsController::avoidingObstacleCallback(const std_msgs::Bool::ConstPtr &msg)
{
  if (avoiding_obstacle_ != msg->data)
  {
    avoiding_obstacle_ = msg->data;
    ROS_DEBUG("Is avoiding obstacle? %s.", avoiding_obstacle_ ? "TRUE" : "FALSE");
  }
}

/**
 * @brief WaypointsController::startScanningCallback receives if the robot is
 * scanning
 * @param msg is scanning?
 */
void WaypointsController::scanningCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (scanning_ != msg->data)
  {
    scanning_ = msg->data;
    ROS_DEBUG("Is scanning? %s.", scanning_ ? "TRUE" : "FALSE");
  }
}

/**
 * @brief WaypointsController::movingAwayCallback
 * @param msg
 */
void WaypointsController::movingAwayCallback(const std_msgs::Bool::ConstPtr &msg)
{
  if (moving_away_ != msg->data)
  {
    moving_away_ = msg->data;
    ROS_DEBUG("Is moving away? %s.", moving_away_ ? "TRUE" : "FALSE");
  }
}

/**
 * @brief WaypointsController::setMineCallback receives a found mine
 * @param msg mine
 */
void WaypointsController::setMineCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	map_->addMine(msg->pose.position);
}

/**
 * @brief WaypointsController::goalActiveCallback called once when the goal
 * becomes active
 */
void WaypointsController::goalActiveCallback()
{
  ROS_INFO("Goal just went active.");
  active_goal_ = true;
}

/**
 * @brief WaypointsController::feedbackCallback called every time feedback
 * is received for the goal
 */
void WaypointsController::feedbackCallback(
    const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
  active_goal_ =
      move_base_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE;
}

/**
 * @brief WaypointsController::resultCallback called once when the goal
 * completes
 */
void WaypointsController::resultCallback(
    const actionlib::SimpleClientGoalState& state,
    const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
  if (!map_)
  {
    ROS_FATAL("Got new result, but there is no strategy yet.");
    return;
  }
  ROS_DEBUG("Finished in state [%s].", state.toString().c_str());
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Succeeded at position (%lf, %lf).", map_->getX(), map_->getY());
    map_->pop();
  }
  else if (state == actionlib::SimpleClientGoalState::ABORTED ||
           state == actionlib::SimpleClientGoalState::REJECTED)
  {
    ROS_INFO("Goal aborted or rejected.");
    map_->pop();
  }
  else if (state == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    ROS_INFO("Goal canceled.");
  }
  active_goal_ = false;
}

void WaypointsController::publishWaypoint(
    const geometry_msgs::Point& waypoint) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "p3at";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = waypoint;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 2;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.36;
  marker.color.b = 0.0;
  waypoints_pub_.publish(marker);
}
}
