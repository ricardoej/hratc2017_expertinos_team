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
#include <metal_detector_msgs/Coil.h>
#include "hratc2017/coil.h"

namespace hratc2017
{

class  Coils
{
public:
  Coils();
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

private:
  Coil left_;
  Coil right_;
};
}

#endif /* _HRATC2017_SENSORS_COILS_H_ */
