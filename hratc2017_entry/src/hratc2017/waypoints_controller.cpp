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
	WaypointsController::WaypointsController(ros::NodeHandle* nh) : ROSNode(nh, 1), map_(NULL), move_base_client_("move_base", true)
	{
		createMap(nh);
		createStrategy();
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
		//wait for the action server to come up
		while(!move_base_client_.waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting for the move_base action server to come up");
		}

		move_base_msgs::MoveBaseGoal goal;

		//we'll send a goal to the robot to move 1 meter forward
		goal.target_pose.header.frame_id = "base_link";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = 1.0;
		goal.target_pose.pose.orientation.w = 1.0;

		ROS_INFO("Sending goal");
		move_base_client_.sendGoal(goal);

		move_base_client_.waitForResult();

		if(move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Hooray, the base moved 1 meter forward");
		else
			ROS_INFO("The base failed to move forward 1 meter for some reason");

	}

	/**
	 * @brief WaypointsController::createMap creates the map with borders
	 */
	void WaypointsController::createMap(ros::NodeHandle* nh)
	{
		ROS_INFO("Subscribing to corners");
		corners_sub_ = nh->subscribe("/corners", 100, &WaypointsController::mapCornersCallback, this);

		// TODO: Retirar essa parte para ler as coordenadas do mapa a partir do tópico /corners
		geometry_msgs::Point leftBottomCorner;
		leftBottomCorner.x = 483236.0625;
		leftBottomCorner.y = 6674518.5;
		leftBottomCorner.z = 10.0;

		geometry_msgs::Point leftTopCorner;
		leftTopCorner.x = 483236.0625;
		leftTopCorner.y = 6674528.5;
		leftTopCorner.z = 10.0;

		geometry_msgs::Point rightTopCorner;
		rightTopCorner.x = 483246.0625;
		rightTopCorner.y = 6674528.5;
		rightTopCorner.z = 10.0;

		geometry_msgs::Point rightBottomCorner;
		rightBottomCorner.x = 483246.0625;
		rightBottomCorner.y = 6674518.5;
		rightBottomCorner.z = 10.0;

		map_ = new Map(leftBottomCorner, leftTopCorner, rightTopCorner, rightBottomCorner);

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
	}

	/**
	 * @brief WaypointsController::mapCornersCallback receives the map borders
	 * @param msg map corner position.
	 */
	void WaypointsController::mapCornersCallback(
		const visualization_msgs::MarkerArray::ConstPtr & corners)
	    
	{
		map_ = new Map(corners);

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
			ros::param::param<double>("map_coverage_offset", mapCoverageOffset, DEFAULT_MAP_COVERAGE_OFFSET);
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
}
