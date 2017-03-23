/**
 *  This header file defines the Coil class.
 *
 *  Version: 1.1.4
 *  Created on: 16/01/2017
 *  Modified on: 22/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_SENSORS_COIL_H_
#define _HRATC2017_SENSORS_COIL_H_

#include <list>
#include <sstream>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include "utilities/mean_filter.h"

#define DEFAULT_DERIVATIVE_SAMPLE_TIME 0.1
#define DEFAULT_LOW_COIL_SIGNAL_THRESHOLD 0.45
#define DEFAULT_HIGH_COIL_SIGNAL_THRESHOLD 0.65
#define DEFAULT_COIL_SIGNAL_FILTER_NUMBER_OF_OBSERVATIONS 6
#define DEFAULT_NUMBER_OF_DERIVATIVES 5
#define DEFAULT_MAX_POSE_UPDATE_INTERVAL 1.0

namespace hratc2017
{

class Coil
{
public:
  Coil(std::string frame_id, float sample_time = DEFAULT_DERIVATIVE_SAMPLE_TIME,
       float low_threshold = DEFAULT_LOW_COIL_SIGNAL_THRESHOLD,
       float high_threshold = DEFAULT_HIGH_COIL_SIGNAL_THRESHOLD,
       unsigned int number_of_observations =
           DEFAULT_COIL_SIGNAL_FILTER_NUMBER_OF_OBSERVATIONS,
       unsigned int number_of_derivatives = DEFAULT_NUMBER_OF_DERIVATIVES);
  virtual ~Coil();
  geometry_msgs::PoseStamped getPose() const;
  std::string getFrameId() const;
  float getValue() const;
  float getDerivedValue() const;
  float getSampleTime() const;
  void setTransform(tf::Transform tf);
  void setPose(geometry_msgs::PoseStamped pose);
  void setPose(tf::StampedTransform robot_tf);
  void setMaxPoseUpdateInterval(double max_pose_update_interval);
  void setSampleTime(double sample_time);
  void setLowThreshold(double low_threshold);
  void setHighThreshold(double high_threshold);
  void setNumberOfObservations(int number_of_observations);
  void setNumberOfDerivatives(int number_of_derivatives);
  bool isLow() const;
  bool isHigh() const;
  void calculateDerivative();
  std::string str() const;
  const char* c_str() const;
  void operator=(float value);

private:
  tf::Transform coil_tf_;
  geometry_msgs::PoseStamped pose_;
  utilities::MeanFilter* value_filter_;
  utilities::MeanFilter* derived_value_filter_;
  std::string frame_id_;
  float last_value_;
  float sample_time_;
  float low_threshold_;
  float high_threshold_;
  float max_pose_update_interval_;
};
}

#endif /* _HRATC2017_SENSORS_COIL_H_ */
