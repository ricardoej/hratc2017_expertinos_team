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
#include <std_msgs/Bool.h>

#define LINEAR_VELOCITY_X 0.1
#define ANGULAR_VELOCITY_Z 0.3
#define KP 1.0
#define MIN_COIL_SIGNAL 0.6
#define MAX_COIL_SIGNAL 0.8
#define COIL_SIGNAL_INCREMENT 0.1
#define COIL_SIGNAL_TOLERANCE 0.01
#define SAFE_COIL_SIGNAL 0.35
#define THRESHOLD 0.5

namespace hratc2017
{
namespace states
{
enum StateEnum
{
  S0_SETTING_UP,
  S1_ALINGING,
  S2_SCANNING_FOWARD,
  S3_SCANNING_LEFT,
  S4_SCANNING_RIGHT,
  S5_MOVING_AWAY
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
  ros::Publisher pause_pub_;
  ros::Subscriber coils_sub_;
  ros::Subscriber pause_sub_;
  StateEnum current_state_;
  StateEnum setNextState();
  void setVelocity();
  void setVelocity(double vx, double wz);
  Coils coils_;
  void pauseCallback(const std_msgs::Bool::ConstPtr& msg);
  void coilsCallback(const metal_detector_msgs::Coil::ConstPtr& msg);
  double vx_;
  double wz_;
  double Kp_;
  double error_;
  double min_coil_signal_;
  double max_coil_signal_;
  double coil_signal_increment_;
  double ref_coil_signal_;
  double coil_signal_tolerance_;
  double safe_coil_signal_;
  double threshold_;
  bool paused_;
  void setPause(bool paused = true);
};
}

#endif /* _HRATC2017_ENTRIES_METAL_SCANNER_H_ */
