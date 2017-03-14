/**
 *  This header file defines the LandmineAnalyzer class, which is based
 *on the ROSNode class. It controls the landmine_analyzer_node.
 *
 *  Version: 1.1.1
 *  Created on: 30/01/2017
 *  Modified on: 13/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *          Luis Victor Pessiqueli Bonin (luis-bonin@unifei.edu.br)
 *          Luiz Fernando Nunes (luizfernandolfn@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_ENTRIES_LANDMINE_ANALYZER_H_
#define _HRATC2017_ENTRIES_LANDMINE_ANALYZER_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "utilities/ros_node.h"
#include "hratc2017/coils.h"

#define END_DURATION 2.0
#define MAX_COIL_SIGNAL 0.95
#define MIN_RADIUS 0.15
#define MAX_RADIUS 0.55
#define STD_RADIUS 0.45

namespace hratc2017
{

class LandmineAnalyzer : public utilities::ROSNode
{
public:
  LandmineAnalyzer(ros::NodeHandle* nh);
  virtual ~LandmineAnalyzer();

private:
  ros::Time end_time_;
  ros::Timer sampler_;
  ros::Publisher scanning_pub_;
  ros::Publisher set_mine_pub_;
  ros::Publisher set_fake_mine_pub_;
  ros::Publisher filtered_coils_pub_;
  ros::Subscriber coils_sub_;
  ros::Subscriber moving_away_sub_;
  Coils coils_;
  std::vector<geometry_msgs::Point> known_landmines_;
  bool sampling_;
  bool moving_away_;
  bool collected_center_;
  double max_coil_signal_;
  double end_duration_;
  double min_radius_;
  double max_radius_;
  double std_radius_;
  geometry_msgs::Point mine_center_;
  geometry_msgs::Point mine_bound_;
  virtual void controlLoop();
  void sample();
  void publish();
  void publishLandminePose();
  void publishLandminePose(double x, double y);
  void publishFakeLandminePose();
  void publishFakeLandminePose(double x, double y, double radius);
  void publishFilteredCoilSignals() const;
  bool isKnownLandmine() const;
  void setScanning(bool scanning);
  void reset();
  void derivativeCallback(const ros::TimerEvent& event);
  void movingAwayCallback(const std_msgs::Bool::ConstPtr& msg);
  double calculateDistance(geometry_msgs::Point p1, geometry_msgs::Point p2) const;
  double calculateRadius() const;
};
}

#endif /* _HRATC2017_ENTRIES_LANDMINE_ANALYZER_H_ */
