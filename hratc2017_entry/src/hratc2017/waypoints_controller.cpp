/**
 *  This source file implements the WaypointsController class, which is
 *based on the ROSNode helper class. It controls the waypoints_controller_node.
 *
 *  Version: 0.0.1
 *  Created on: 06/02/2017
 *  Modified on: 06/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
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
	WaypointsController::WaypointsController(ros::NodeHandle* nh) : ROSNode(nh, 2), map_(NULL), move_base_client_("/move_base", true)
	{
		ROS_INFO("Subscribing to corners");
		corners_sub_ = nh->subscribe("/corners", 100, &WaypointsController::mapCornersCallback, this);

		ROS_INFO("Subscribing to start_scanning");
		start_scanning_sub_ = nh->subscribe("/p3at/start_scanning", 100, &WaypointsController::startScanningCallback, this);

		has_active_goal_ = false;
		is_scanning_ = false;
	}

	/**
	 * @brief WaypointsController::~WaypointsController
	 */
	WaypointsController::~WaypointsController()
	{
		corners_sub_.shutdown();

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
		// wait for the action server to come up
		if (!move_base_client_.waitForServer(ros::Duration(0.5)))
		{
			ROS_INFO("Waiting for the move_base action server to come up");
		}
		else
		{
			if (is_scanning_ && has_active_goal_)
			{
				move_base_client_.cancelAllGoals();
				ROS_INFO("Cancel goals");
			}
			else if (map_ && !has_active_goal_ && waypoints_.size() > 0)
			{
				move_base_msgs::MoveBaseGoal goal;
				goal.target_pose.header.frame_id = "map";
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x = waypoints_.front().x;
				goal.target_pose.pose.position.y = waypoints_.front().y;
				goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

				ROS_INFO("Sending goal [%f, %f, %f]", waypoints_.front().x, waypoints_.front().y, waypoints_.front().z);
				move_base_client_.sendGoal(goal, 
					boost::bind(&WaypointsController::goalDoneCallback, this, _1, _2),
					boost::bind(&WaypointsController::goalActiveCallback, this), 
					boost::bind(&WaypointsController::goalFeedbackCallback, this, _1));			
			}
		}
	}

	/**
	 * @brief WaypointsController::mapCornersCallback receives the map borders
	 * @param msg map corner position.
	 */
	void WaypointsController::mapCornersCallback(
		const visualization_msgs::MarkerArray::ConstPtr & corners)
	    
	{
		map_ = new Map(corners, "relative");

		ROS_INFO("Map created! (%f, %f, %f) (%f, %f, %f) (%f, %f, %f) (%f, %f, %f)",
			map_->getLeftBottomCorner().x,
			map_->getLeftBottomCorner().y,
			map_->getLeftBottomCorner().z,
			map_->getLeftTopCorner().x,
			map_->getLeftTopCorner().y,
			map_->getLeftTopCorner().z,
			map_->getRightTopCorner().x,
			map_->getRightTopCorner().y,
			map_->getRightTopCorner().z,
			map_->getRightBottomCorner().x,
			map_->getRightBottomCorner().y,
			map_->getRightBottomCorner().z);

		createStrategy();
	}

	/**
	 * @brief WaypointsController::startScanningCallback receives if the robot is scanning
	 * @param msg is scanning?
	 */
	void WaypointsController::startScanningCallback(const std_msgs::Bool::ConstPtr& msg)
	{
		ROS_INFO("start_scanning %s", msg->data ? "TRUE" : "FALSE");
		if (is_scanning_ != msg->data)
		{
			is_scanning_ = msg->data;
			ROS_INFO("Is scanning? %s", is_scanning_ ? "TRUE" : "FALSE");
		}
	}

	/**
	 * @brief WaypointsController::createStrategy creates the waypoint/map coverage strategy
	 */
	void WaypointsController::createStrategy()
	{
		geometry_msgs::Point startWaypoint = map_->getLeftBottomCorner();
		geometry_msgs::Point finishWaypoint = map_->getRightBottomCorner();
		geometry_msgs::Point currentWaypoint = startWaypoint;
		waypoints_.push(startWaypoint);
		ROS_INFO("Added Point: (%f, %f, %f)", startWaypoint.x, startWaypoint.y, startWaypoint.z);

		bool hasTargetFinishPoint = false;

		while (true)
		{
			geometry_msgs::Point nextWaypoint;

			if (currentWaypoint.y == map_->getLeftBottomCorner().y)
			{
				nextWaypoint.y = map_->getLeftTopCorner().y;
			}
			else if (currentWaypoint.y == map_->getLeftTopCorner().y)
			{
				nextWaypoint.y = map_->getLeftBottomCorner().y;
			}

			nextWaypoint.x = currentWaypoint.x;
			nextWaypoint.z = currentWaypoint.z;
			waypoints_.push(nextWaypoint);
			ROS_INFO("Added Point: (%f, %f, %f)", nextWaypoint.x, nextWaypoint.y, nextWaypoint.z);
			currentWaypoint = nextWaypoint;

			if (currentWaypoint.x == finishWaypoint.x && currentWaypoint.y == finishWaypoint.y)
			{
				hasTargetFinishPoint = true;
			}

			if (hasTargetFinishPoint)
			{
				break;
			}

			geometry_msgs::Point offsetWaypoint;
			double mapCoverageOffset;
			ros::param::param<double>("/map_coverage_offset", mapCoverageOffset, DEFAULT_MAP_COVERAGE_OFFSET);
			offsetWaypoint.x = currentWaypoint.x + mapCoverageOffset;
			offsetWaypoint.y = currentWaypoint.y;
			offsetWaypoint.z = currentWaypoint.z;
			waypoints_.push(offsetWaypoint);
			ROS_INFO("Added Point: (%f, %f, %f)", offsetWaypoint.x, offsetWaypoint.y, offsetWaypoint.z);
			currentWaypoint = offsetWaypoint;

			if (offsetWaypoint.x >= finishWaypoint.x)
			{
				// At this point, we know that the robot has target the finish point
				hasTargetFinishPoint = true;
			}
		}
	}

	/**
	 * @brief WaypointsController::goalDoneCallback called once when the goal completes
	 */
	void WaypointsController::goalDoneCallback(const actionlib::SimpleClientGoalState& state, 
		const move_base_msgs::MoveBaseResult::ConstPtr &result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Finished in position [%f, %f, %f]", waypoints_.front().x, waypoints_.front().y, waypoints_.front().z);
			waypoints_.pop();
			has_active_goal_ = false;
		}
		else if (state == actionlib::SimpleClientGoalState::ABORTED || state == actionlib::SimpleClientGoalState::REJECTED
			|| state == actionlib::SimpleClientGoalState::PREEMPTED)
		{
			ROS_INFO("Goal aborted or rejected");
			has_active_goal_ = false;
		}
	}

	/**
	 * @brief WaypointsController::goalActiveCallback called once when the goal becomes active
	 */
	void WaypointsController::goalActiveCallback()
	{
	  ROS_INFO("Goal just went active");
	  has_active_goal_ = true;
	}

	/**
	 * @brief WaypointsController::goalFeedbackCallback called every time feedback is received for the goal
	 */
	void WaypointsController::goalFeedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr &feedback)
	{
	  has_active_goal_ = move_base_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE;
	}
}
