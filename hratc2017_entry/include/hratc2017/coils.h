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

namespace hratc2017
{

class  Coils
{
public:
  Coils(double threshold = Coils::THRESHOLD, float left = 0.0, float right = 0.0);
  Coils(const metal_detector_msgs::Coil::ConstPtr& msg, double threshold = Coils::THRESHOLD);
  Coils(const metal_detector_msgs::Coil& msg, double threshold = Coils::THRESHOLD);
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
  const static double THRESHOLD;

private:
  float left_;
  float right_;
  double threshold_;
};
}

#endif /* _HRATC2017_SENSORS_COILS_H_ */
