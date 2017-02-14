/**
 *  This header file defines the Coils class.
 *
 *  Version: 0.0.1
 *  Created on: 30/01/2017
 *  Modified on: 14/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_SENSORS_COILS_H_
#define _HRATC2017_SENSORS_COILS_H_

#include <ros/ros.h>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <metal_detector_msgs/Coil.h>

#define COIL_SIGNAL_THRESHOLD 0.6
#define COIL_SIGNAL_FILTER_NUMBER_OF_OBSERVATIONS 10

namespace hratc2017
{

class  Coils
{
public:
  Coils(double threshold = COIL_SIGNAL_THRESHOLD, int number_of_observations_ = COIL_SIGNAL_FILTER_NUMBER_OF_OBSERVATIONS, float left = 0.0, float right = 0.0);
  Coils(const metal_detector_msgs::Coil::ConstPtr& msg, double threshold = COIL_SIGNAL_THRESHOLD, int number_of_observations_ = COIL_SIGNAL_FILTER_NUMBER_OF_OBSERVATIONS);
  Coils(const metal_detector_msgs::Coil& msg, double threshold = COIL_SIGNAL_THRESHOLD, int number_of_observations_ = COIL_SIGNAL_FILTER_NUMBER_OF_OBSERVATIONS);
  virtual ~Coils();
  float getLeft() const;
  float getRight() const;
  void setThreshold(double threshold);
  void setNumberOfObservations(int number_of_observations);
  bool isHighCoilSignalOnLeft() const;
  bool isHighCoilSignalOnRight() const;
  metal_detector_msgs::Coil to_msg() const;
  std::string str() const;
  const char* c_str() const;
  void operator=(const metal_detector_msgs::Coil::ConstPtr& msg);
  void operator=(const metal_detector_msgs::Coil& msg);
  void coilsCallback(const metal_detector_msgs::Coil::ConstPtr& msg);

private:
  float left_;
  float right_;
  double threshold_;
  int number_of_observations_;
  std::vector<float> left_samples_;
  std::vector<float> right_samples_;
};
}

#endif /* _HRATC2017_SENSORS_COILS_H_ */
