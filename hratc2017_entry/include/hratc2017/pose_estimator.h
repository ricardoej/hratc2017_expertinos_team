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
#include <visualization_msgs/MarkerArray.h>
#include "utilities/mean_filter.h"
#include "utilities/ros_node.h"

#define CENTER_X 483238.6184840562
#define CENTER_Y 6674530.097688958

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
  bool isMoving_;
  bool wating_;
  ros::Time timer_;
  ros::Time last_timestamp_;

  // utilities::MeanFilter mean_filter_;
  geometry_msgs::PoseWithCovarianceStamped p1_;
  geometry_msgs::PoseWithCovarianceStamped p2_;

  tf::Quaternion quat_initial_;
  tf::Quaternion quat_data_;
  tf::Quaternion quat_ekf_;

  sensor_msgs::Imu imu_data_;
  sensor_msgs::Imu imu_initial_;
  sensor_msgs::Imu imu_ekf_;

  nav_msgs::Odometry utm_read1_;
  nav_msgs::Odometry utm_read2_;

//  nav_msgs::Odometry gps_odom_;
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
  double centerX_;
  double centerY_;
  double yaw_initial_;
  double yaw_data_;
  double yaw_ekf_;
  int reading_count_;
  StateEnum current_state_;

  utilities::MeanFilter* mean_filter_x_;
  utilities::MeanFilter* mean_filter_y_;


  virtual void controlLoop();
  void gpsOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void odomP3atCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void cornersCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void calcPoseEstimated();
  void sendImuEkf();
  void sendOdomWithOffset();
  void setVelocity(double vx, double wz);

};
}

#endif /* _HRATC2017_ENTRIES_POSE_ESTIMATOR_H_ */
