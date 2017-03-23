/**
 *  This header file defines the Coils class.
 *
 *  Version: 1.1.4
 *  Created on: 30/01/2017
 *  Modified on: 22/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_SENSORS_COILS_H_
#define _HRATC2017_SENSORS_COILS_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <metal_detector_msgs/Coil.h>
#include "hratc2017/coil.h"

#define MINEFIELD_FRAME_ID "/minefield"
#define BASE_FRAME_ID "/base_link"
#define POSE_UPDATE_INTERVAL 0.15

namespace hratc2017
{

class Coils
{
public:
  Coils(ros::NodeHandle* nh);
  virtual ~Coils();
  float getLeftValue() const;
  float getRightValue() const;
  float getMeanValue() const;
  float getLeftDerivedValue() const;
  float getRightDerivedValue() const;
  float getMeanDerivedValue() const;
  double getLeftSampleTime() const;
  double getRightSampleTime() const;
  double getMeanSampleTime() const;
  void setMaxPoseUpdateInterval(double max_pose_update_interval);
  void setSampleTime(double sample_time);
  void setLowThreshold(double low_threshold);
  void setHighThreshold(double high_threshold);
  void setNumberOfObservations(int number_of_observations);
  void setNumberOfDerivatives(int number_of_observations);
  bool isLeftLow() const;
  bool isLeftHigh() const;
  bool isRightLow() const;
  bool isRightHigh() const;
  bool isOneLow() const;
  bool isOneHigh() const;
  bool isAnyLow() const;
  bool isAnyHigh() const;
  bool isBothLow() const;
  bool isBothHigh() const;
  bool isBothNotLow() const;
  bool isBothNotHigh() const;
  bool isSettedUp();
  void calculateDerivative();
  metal_detector_msgs::Coil to_msg() const;
  std::string str() const;
  const char* c_str() const;
  void operator=(const metal_detector_msgs::Coil::ConstPtr& msg);
  void operator=(const metal_detector_msgs::Coil& msg);
  void setParameters(const ros::NodeHandle& pnh);
  void coilsCallback(const metal_detector_msgs::Coil::ConstPtr& msg);
  geometry_msgs::PoseStamped getLeftPose() const;
  geometry_msgs::PoseStamped getRightPose() const;
  geometry_msgs::PoseStamped getMidstPose() const;

private:
  ros::Subscriber coils_sub_;
  ros::Timer pose_update_;
  tf::TransformListener tf_;
  Coil left_;
  Coil right_;
  float last_sample_;
  bool left_updated_;
  bool right_updated_;
  void updatePose(const ros::TimerEvent& event);
  bool updateCoilTransform(Coil *coil);

};
}

#endif /* _HRATC2017_SENSORS_COILS_H_ */
