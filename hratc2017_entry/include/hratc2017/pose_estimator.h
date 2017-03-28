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

#ifndef _HRATC2017_ENTRIES_POSE_ESTIMATOR_H_
#define _HRATC2017_ENTRIES_POSE_ESTIMATOR_H_

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/MarkerArray.h>
#include "utilities/point_mean_filter.h"
#include "utilities/ros_node.h"

#define CENTER_X 483238.6184840562
#define CENTER_Y 6674530.097688958
#define SAMPLING_DURATION 0.1
#define NUMBER_OF_SAMPLES 10

namespace hratc2017
{
namespace states
{
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
  bool has_pose_estimated_;
  bool has_utm_reading1_;
  bool has_utm_reading2_;
  bool has_imu_initial_;
  bool has_odom_initial_;
  bool moving_;
  bool wating_;
  bool started_hratc2017_;
  ros::Time timer_;
  ros::Time last_timestamp_;

  geometry_msgs::Point p0_;
  geometry_msgs::Point p1_;
  geometry_msgs::Point p2_;
  double angle_;

  tf::Quaternion quat_initial_;
  tf::Quaternion quat_data_;
  tf::Quaternion quat_ekf_;
  sensor_msgs::Imu imu_data_;
  sensor_msgs::Imu imu_initial_;
  sensor_msgs::Imu imu_ekf_;

  nav_msgs::Odometry odom_p3at_;
  nav_msgs::Odometry odom_initial_;
  nav_msgs::Odometry odom_w_offset_;

  ros::Subscriber gps_odom_sub_;
  ros::Subscriber odom_p3at_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Subscriber odom_data_sub_;
  ros::Subscriber odom_initial_sub_;
  ros::Publisher imu_pub_;
  ros::Publisher odom_offset_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher pose_estimated_pub_;
  ros::ServiceClient start_hratc2017_cli_;
  double yaw_initial_;
  double yaw_data_;
  double yaw_ekf_;
  int reading_count_;
  StateEnum current_state_;
  utilities::PointMeanFilter* mean_filter_;

  virtual void controlLoop();
  void gpsOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void odomP3atCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void cornersCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void calcPoseEstimated();
  void sendImuEkf();
  void sendOdomWithOffset();
  void setVelocity(double vx, double wz);
  void startHRATC2017();
};
}

#endif /* _HRATC2017_ENTRIES_POSE_ESTIMATOR_H_ */
