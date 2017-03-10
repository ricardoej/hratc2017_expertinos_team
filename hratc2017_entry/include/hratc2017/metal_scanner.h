/**
 *  This header file defines the MetalScanner class, which is based
 *on the ROSNode class. It controls the metal_scanner_node.
 *
 *  Version: 1.0.4
 *  Created on: 09/02/2017
 *  Modified on: 10/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *          Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *          Luiz Fernando Nunes (luizfernandolfn@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_ENTRIES_METAL_SCANNER_H_
#define _HRATC2017_ENTRIES_METAL_SCANNER_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include "hratc2017/coils.h"
#include "utilities/ros_node.h"

#define LINEAR_VELOCITY_X 0.1
#define ANGULAR_VELOCITY_Z 0.3
#define LINEAR_KP 1.0
#define ANGULAR_KP 1.0
#define LINEAR_TOLERANCE 0.05
#define ANGULAR_TOLERANCE 0.01
#define MIN_COIL_SIGNAL 0.6
#define MAX_COIL_SIGNAL 0.8
#define THRESHOLD 0.5
#define SAFE_TIME 2.0
#define ROTATION_TIME 2.0
#define MOVING_AWAY_TIME 3.0

namespace hratc2017
{
namespace states
{
enum StateEnum
{
  S0_SETTING_UP,
  S1_ALIGNING,
  S2_SCANNING,
  S3_MOVING_BACK,
  S4_CHANGING_DIRECTION,
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
  ros::Time timer_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher moving_away_pub_;
  ros::Subscriber coils_sub_;
  ros::Subscriber scanning_sub_;
  StateEnum current_state_;
  Coils coils_;
  double vx_;
  double wz_;
  double linear_Kp_;
  double angular_Kp_;
  double linear_error_;
  double angular_error_;
  double linear_reference_;
  double linear_tolerance_;
  double angular_tolerance_;
  double min_coil_signal_;
  double max_coil_signal_;
  double safe_time_;
  double rotation_time_;
  double moving_away_time_;
  bool scanning_;
  bool moving_away_;
  virtual void controlLoop();
  void setNextState();
  void setVelocity();
  void setVelocity(double vx, double wz);
  void setMovingAway(bool moving_away);
  void reset();
  void scanningCallback(const std_msgs::Bool::ConstPtr& msg);
};
}

#endif /* _HRATC2017_ENTRIES_METAL_SCANNER_H_ */
