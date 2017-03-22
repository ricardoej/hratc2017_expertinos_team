/**
 *  This header file defines the MetalScanner class, which is based
 *on the ROSNode class. It controls the metal_scanner_node.
 *
 *  Version: 1.1.1
 *  Created on: 09/02/2017
 *  Modified on: 13/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *          Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *          Luiz Fernando Nunes (luizfernandolfn@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_ENTRIES_METAL_SCANNER_H_
#define _HRATC2017_ENTRIES_METAL_SCANNER_H_

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include "hratc2017/coils.h"
#include "hratc2017/displacement_monitor.h"
#include "utilities/points.h"
#include "utilities/ros_node.h"

#define LINEAR_VELOCITY_X 0.1
#define ANGULAR_VELOCITY_Z 0.3
#define LINEAR_KP 1.0
#define ANGULAR_KP 1.0
#define MIN_COIL_SIGNAL 0.6
#define MAX_COIL_SIGNAL 0.8
#define THRESHOLD 0.5
#define PAUSE_TIME 0.5
#define SAFE_TIME 2.0
#define ROTATION_TIME 2.0
#define MOVING_AWAY_TIME 3.0
#define STANDARD_RADIUS 0.5

namespace hratc2017
{
namespace states
{
enum StateEnum
{
  S0_SETTING_UP,
  S1_ALIGNING,
  S2_SCANNING,
  S3_HOLDING_ON,
  S4_MOVING_BACK,
  S5_CHANGING_DIRECTION,
  S6_MOVING_AWAY
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
  ros::Publisher known_landmine_pub_;
  ros::Publisher moving_away_pub_;
  ros::Subscriber coils_sub_;
  ros::Subscriber scanning_sub_;
  ros::Subscriber mines_sub_;
  ros::Subscriber fake_mines_sub_;
  StateEnum current_state_;
  Coils coils_;
  DisplacementMonitor disp_monitor_;
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
  double pause_time_;
  double safe_time_;
  double rotation_time_;
  double moving_away_time_;
  double std_radius_;
  bool scanning_;
  bool moving_away_;
  std::vector<geometry_msgs::Point> mines_;
  std::vector<geometry_msgs::Point> fake_mines_;
  virtual bool isSettedUp();
  virtual void controlLoop();
  void setNextState();
  void setVelocity();
  void setVelocity(double vx, double wz);
  void setMovingAway(bool moving_away);
  void scanningCallback(const std_msgs::Bool::ConstPtr& msg);
  void minesCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void fakeMinesCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void publishKnownMine(bool known);
  virtual void reset();
  bool isKnownMine() const;
  bool isKnownMine(geometry_msgs::Point p) const;
  bool isKnownMine(double x, double y) const;
  bool isKnownFakeMine() const;
  bool isKnownFakeMine(geometry_msgs::Point p) const;
  bool isKnownFakeMine(double x, double y) const;
};
}

#endif /* _HRATC2017_ENTRIES_METAL_SCANNER_H_ */
