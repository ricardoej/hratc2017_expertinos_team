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
#include "hratc2017/coils.h"

#define LINEAR_VELOCITY_X 0.1
#define ANGULAR_VELOCITY_Z 0.3
#define KP 1
#define MIN_COIL_SIGNAL 0.6
#define MAX_COIL_SIGNAL 0.8
#define COIL_SIGNAL_INCREMENT 0.1

namespace hratc2017
{
namespace states
{
enum StateEnum
{
  S0,
  S1,
  S2,
  S3,
  S4,
  S5
};
}

typedef states::StateEnum StateEnum;

class MetalScanner : public utilities::ROSNode
{
public:
  MetalScanner(ros::NodeHandle* nh);
  virtual ~MetalScanner();

private:
  virtual void controlLoop();
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber coils_sub_;
  StateEnum current_state_;
  StateEnum getNextState();
  void setVelocity();
  void setVelocity(double vx, double wz);
  Coils coils_;
  void coilsCallback(const metal_detector_msgs::Coil::ConstPtr& msg);
};
}

#endif /* _HRATC2017_ENTRIES_METAL_SCANNER_H_ */
