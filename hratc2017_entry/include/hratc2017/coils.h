/**
 *  This header file defines the Coils class.
 *
 *  Version: 1.0.1
 *  Created on: 30/01/2017
 *  Modified on: 20/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_SENSORS_COILS_H_
#define _HRATC2017_SENSORS_COILS_H_

#include <sstream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <metal_detector_msgs/Coil.h>
#include "hratc2017/coil.h"

#define MINEFIELD_FRAME_ID "/minefield"

namespace hratc2017
{

class Coils
{
public:
  Coils(tf::TransformListener* tf = NULL);
  virtual ~Coils();
  float getLeftValue() const;
  float getRightValue() const;
  void setLowThreshold(double low_threshold);
  void setHighThreshold(double high_threshold);
  void setNumberOfObservations(int number_of_observations);
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
  metal_detector_msgs::Coil to_msg() const;
  std::string str() const;
  const char* c_str() const;
  void operator=(const metal_detector_msgs::Coil::ConstPtr& msg);
  void operator=(const metal_detector_msgs::Coil& msg);
  void coilsCallback(const metal_detector_msgs::Coil::ConstPtr& msg);
  geometry_msgs::PoseStamped getLeftPose() const;
  geometry_msgs::PoseStamped getRightPose() const;

private:
  Coil left_;
  Coil right_;
  tf::TransformListener* tf_;
  geometry_msgs::PoseStamped EMPTY_POSE;
  geometry_msgs::PoseStamped getPose(std::string frame_id) const;
};
}

#endif /* _HRATC2017_SENSORS_COILS_H_ */
