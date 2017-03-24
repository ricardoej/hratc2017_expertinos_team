/**
 *  This source file implements the MapController class, which is
 *based on the ROSNode helper class. It controls the map_Controller_node.
 *
 *  Version: 1.0.0
 *  Created on: 21/03/2017
 *  Modified on: 21/03/2017
 *  Author: Ricardo Emerson Julio (ricardoej@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_ENTRIES_MAP_CONTROLLER_H_
#define _HRATC2017_ENTRIES_MAP_CONTROLLER_H_

#include <ros/ros.h>
#include "utilities/ros_node.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/MarkerArray.h>
#include <utilities/mean_filter.h>
#include "tf/tf.h"
#include <tf/transform_datatypes.h>

#define CENTER_X 483238.6184840562
#define CENTER_Y 6674530.097688958


namespace hratc2017
{

namespace states{

enum StateEnum
{
  S0_STOPPED,
  S1_READING1,
  S2_MOVING,
  S3_READING2,
  S4_FINISHED
};
}

typedef states::StateEnum StateEnum;

class PoseEstimator : public utilities::ROSNode
{
public:
  PoseEstimator(ros::NodeHandle* nh);
  virtual ~PoseEstimator();

private:
  bool pose_estimated_sent_;
  bool has_utm_reading1_;
  bool has_utm_reading2_;
  bool has_imu_initial_;
  bool has_odom_initial_;
  bool isMoving_;
  ros::Time timer_;
  //utilities::MeanFilter mean_filter_;
  geometry_msgs::PoseWithCovarianceStamped p1_;
  geometry_msgs::PoseWithCovarianceStamped p2_;
  tf::Quaternion quat_initial_;
  tf::Quaternion quat_data_;
  tf::Quaternion quat_ekf_;
  sensor_msgs::Imu imu_data_;
  sensor_msgs::Imu imu_initial_;
  sensor_msgs::Imu imu_ekf_;
  nav_msgs::Odometry utm_reading1_;
  nav_msgs::Odometry utm_reading2_;
  nav_msgs::Odometry odom_p3at_;
  nav_msgs::Odometry odom_initial_;
  ros::Subscriber gps_odom_sub_;
  ros::Subscriber odom_p3at_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Publisher imu_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher pose_estimated_pub_;
  double centerX_;
  double centerY_;
  double yaw_initial_;
  double yaw_data_;
  double yaw_ekf_;
  StateEnum current_state_;
  virtual void controlLoop();
  void gpsOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void odomP3atCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void cornersCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void setNextStage();
  void setVelocity(double vx, double wz);


};
}

#endif /* _HRATC2017_ENTRIES_MAP_CONTROLLER_H_ */
