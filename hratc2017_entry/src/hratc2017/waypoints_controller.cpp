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
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

	/**
	 * @brief WaypointsController::WaypointsController
	 * @param nh
	 */
	WaypointsController::WaypointsController(ros::NodeHandle* nh) : ROSNode(nh, 1)
	{
		nh->subscribe("borders", 10, &WaypointsController::mapBordersCallback, this);
	}

	/**
	 * @brief WaypointsController::~WaypointsController
	 */
	WaypointsController::~WaypointsController() {}

	/**
	 * @brief WaypointsController::controlLoop
	 */
	void WaypointsController::controlLoop() 
	{
		//tell the action client that we want to spin a thread by default
		MoveBaseClient ac("move_base", true);

		//wait for the action server to come up
		while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
		}

		move_base_msgs::MoveBaseGoal goal;

		//we'll send a goal to the robot to move 1 meter forward
		goal.target_pose.header.frame_id = "base_link";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = 1.0;
		goal.target_pose.pose.orientation.w = 1.0;

		ROS_INFO("Sending goal");
		ac.sendGoal(goal);

		ac.waitForResult();

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, the base moved 1 meter forward");
		else
		ROS_INFO("The base failed to move forward 1 meter for some reason");

	}

	/**
	 * @brief WaypointsController::mapBordersCallback receives the map borders
	 * @param msg map border position.
	 */
	void WaypointsController::mapBordersCallback(
	    const visualization_msgs::MarkerArray::ConstPtr& msg)
	{
		ROS_INFO("[COILS CB] new reading");
	}
}
