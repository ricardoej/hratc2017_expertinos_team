/**
 *  This header file defines the MetalScanner class, which is based
 *on the ROSNode class. It controls the metal_scanner_node.
 *
 *  Version: 0.0.1
 *  Created on: 09/02/2017
 *  Modified on: 09/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *          Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_ENTRIES_METAL_SCANNER_H_
#define _HRATC2017_ENTRIES_METAL_SCANNER_H_

#include <ros/ros.h>
#include "utilities/ros_node.h"
#include <geometry_msgs/Twist.h>

namespace hratc2017
{

class MetalScanner : public utilities::ROSNode
{
public:
  MetalScanner(ros::NodeHandle* nh);
  virtual ~MetalScanner();

private:
  virtual void controlLoop();
  ros::Publisher cmd_vel_pub_;
  void setVelocity(double vx, double wz);
};
}

#endif /* _HRATC2017_ENTRIES_METAL_SCANNER_H_ */
