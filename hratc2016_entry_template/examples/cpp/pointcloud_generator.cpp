#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pointcloud_generator_node");
  ros::NodeHandle n;

	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> tilt_ac("/ptu_d46_controller/follow_joint_trajectory", true);
	ROS_INFO("PointCloud Generator -- Waiting for tilt action server to start...");
	tilt_ac.waitForServer();
	ROS_INFO("PointCloud Generator -- Got it!");

	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory.header.stamp = ros::Time::now();
	goal.trajectory.joint_names.resize(2);
	goal.trajectory.points.resize(1);
	goal.trajectory.joint_names[0] = "ptu_d46_pan_joint";
	goal.trajectory.points[0].positions.push_back(0.0);
	goal.trajectory.points[0].velocities.push_back(0.0);
	goal.trajectory.joint_names[1] = "ptu_d46_tilt_joint";
	goal.trajectory.points[0].positions.push_back(tilt);
	goal.trajectory.points[0].velocities.push_back(tilt_speed);
	goal.trajectory.points[0].time_from_start = ros::Duration(0.5);
	goal.goal_tolerance.resize(2);
	goal.goal_tolerance[0].name = "ptu_d46_pan_joint";
	goal.goal_tolerance[0].position = 0.01;
	goal.goal_tolerance[1].name = "ptu_d46_tilt_joint";
	goal.goal_tolerance[1].position = 0.01;
	goal.goal_time_tolerance = ros::Duration(0.5);
	tilt_ac.sendGoal(goal);

	double timeout(10);
	if(!tilt_ac.waitForResult(ros::Duration(timeout)))
	{
			ROS_FATAL("PointCloud Generator -- Unable to move the laser to the start position!");
			ROS_BREAK();
	}
	ROS_INFO("Got it again!!!");
  return 0;
}


