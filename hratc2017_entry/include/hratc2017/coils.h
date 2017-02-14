/**
 *  This header file defines the Coils class.
 *
 *  Version: 0.0.1
 *  Created on: 30/01/2017
 *  Modified on: 01/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_SENSORS_COILS_H_
#define _HRATC2017_SENSORS_COILS_H_

#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <metal_detector_msgs/Coil.h>

#define COIL_SIGNAL_THRESHOLD 0.6
#define NUMBER_OF_OBSERVATIONS 5

namespace hratc2017
{

class  Coils
{
public:
  Coils(double threshold = COIL_SIGNAL_THRESHOLD, float left = 0.0, float right = 0.0);
  Coils(const metal_detector_msgs::Coil::ConstPtr& msg, double threshold = COIL_SIGNAL_THRESHOLD);
  Coils(const metal_detector_msgs::Coil& msg, double threshold = COIL_SIGNAL_THRESHOLD);
  virtual ~Coils();
  float getLeft() const;
  float getRight() const;
  void setThreshold(double threshold);
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
  std::vector<float> left_samples_;
  std::vector<float> right_samples_;
};
}

#endif /* _HRATC2017_SENSORS_COILS_H_ */
