/**
 *  This header file defines the Coils class.
 *
 *  Version: 0.0.1
 *  Created on: 30/01/2017
 *  Modified on: 30/01/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_SENSORS_COILS_H_
#define _HRATC2017_SENSORS_COILS_H_

#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <metal_detector_msgs/Coil.h>

namespace sensors
{

class  Coils
{
public:
  Coils(float left = 0.0, float right = 0.0);
  Coils(const metal_detector_msgs::Coil::ConstPtr& msg);
  Coils(const metal_detector_msgs::Coil& msg);
  virtual ~Coils();
  float getLeft() const;
  float getRight() const;
  bool gotLandmineOnLeft() const;
  bool gotLandmineOnRight() const;
  metal_detector_msgs::Coil to_msg() const;
  std::string str() const;
  const char* c_str() const;
  void operator=(const metal_detector_msgs::Coil::ConstPtr& msg);
  void operator=(const metal_detector_msgs::Coil& msg);

private:
  float left_;
  float right_;
  const static float THRESHOLD;
};
}

#endif /* _HRATC2017_SENSORS_COILS_H_ */
