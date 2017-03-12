#include "convert_odom.h"

using namespace std;

ConvertOdom::ConvertOdom(ros::NodeHandle n)
{

  ekf_odom_sub_ = n_.subscribe("robot_pose_ekf/odom",100, &ConvertOdom::ekfOdomCallback, this);
  slam_pose_sub_ = n_.subscribe("slam_out_pose",100, &ConvertOdom::slamPoseCallback, this);

  convertOdom_pub_ = n_.advertise<nav_msgs::Odometry>("odom_w_ekf",100);
  convertSlamPose_pub_ = n_.advertise<nav_msgs::Odometry>("slam_out_odom",100);

}

ConvertOdom::~ConvertOdom(){
  ekf_odom_sub_.shutdown();
  slam_pose_sub_.shutdown();
  convertOdom_pub_.shutdown();
  convertSlamPose_pub_.shutdown();
}

void ConvertOdom::ekfOdomCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& ekf_odom)
{
  odom_converted_.header.frame_id = "odom";
  odom_converted_.child_frame_id = "base_link";
  odom_converted_.header.stamp = ekf_odom->header.stamp;
  odom_converted_.pose.pose.position.x = ekf_odom->pose.pose.position.x;
  odom_converted_.pose.pose.position.y = ekf_odom->pose.pose.position.y;
  odom_converted_.pose.pose.position.z = ekf_odom->pose.pose.position.z;
  odom_converted_.pose.pose.orientation.w = ekf_odom->pose.pose.orientation.w;
  odom_converted_.pose.pose.orientation.x = ekf_odom->pose.pose.orientation.x;
  odom_converted_.pose.pose.orientation.y = ekf_odom->pose.pose.orientation.y;
  odom_converted_.pose.pose.orientation.z = ekf_odom->pose.pose.orientation.z;
}

void ConvertOdom::slamPoseCallback(const geometry_msgs::PoseStampedConstPtr& slam_pose)
{
  slam_pose_converted_.header.frame_id = slam_pose->header.frame_id;
  slam_pose_converted_.header.stamp = slam_pose->header.stamp;

  slam_pose_converted_.pose.pose.position.x = slam_pose->pose.position.x;
  slam_pose_converted_.pose.pose.position.y = slam_pose->pose.position.y;
  slam_pose_converted_.pose.pose.position.z = slam_pose->pose.position.z;

  slam_pose_converted_.pose.pose.orientation.x = slam_pose->pose.orientation.x;
  slam_pose_converted_.pose.pose.orientation.y = slam_pose->pose.orientation.y;
  slam_pose_converted_.pose.pose.orientation.z = slam_pose->pose.orientation.z;
  slam_pose_converted_.pose.pose.orientation.w = slam_pose->pose.orientation.w;

}

void ConvertOdom::spin()
{
  ros::Rate loopRate(30);
  while(n_.ok())
  {

    convertOdom_pub_.publish(odom_converted_);
    convertSlamPose_pub_.publish(slam_pose_converted_);

    ros::spinOnce();
    loopRate.sleep();
  }
}
