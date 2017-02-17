/**
 *  This header file defines the LandmineAnalyzer class, which is based
 *on the ROSNode class. It controls the landmine_analyzer_node.
 *
 *  Version: 0.0.1
 *  Created on: 30/01/2017
 *  Modified on: 13/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *          Luis Victor Pessiqueli Bonin (luis-bonin@unifei.edu.br)
 *          Luiz Fernando Nunes (luizfernandolfn@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_ENTRIES_LANDMINE_ANALYZER_H_
#define _HRATC2017_ENTRIES_LANDMINE_ANALYZER_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "utilities/ros_node.h"
#include "hratc2017/coils.h"
#include <std_msgs/Bool.h>

#define SAMPLING_END_INTERVAL 2.0
#define MAX_COIL_SIGNAL 0.9
#define ALIGNMENT_TOLERANCE 0.05
#define MIN_SIGNAL_RADIUS 0.15
#define MAX_SIGNAL_RADIUS 0.55

namespace hratc2017
{

class LandmineAnalyzer : public utilities::ROSNode
{
public:
  LandmineAnalyzer(ros::NodeHandle* nh);
  virtual ~LandmineAnalyzer();

private:
  tf::TransformListener tf_;
  ros::Publisher pause_pub_;
  ros::Publisher set_mine_pub_;
  ros::Publisher set_fake_mine_pub_;
  ros::Publisher polygon_pub_;
  ros::Publisher filtered_coils_pub_;
  ros::Subscriber coils_sub_;
  Coils coils_;
  bool max_signal_found_in_both_;
  bool paused_;
  bool possible_mine_found_;
  bool sampling_;
  double sampling_end_interval_;
  double max_coil_signal_;
  double alignment_tolerance_;
  double min_signal_radius_;
  double max_signal_radius_;
  geometry_msgs::Point32 p_max_left_;
  geometry_msgs::Point32 p_max_right_;
  geometry_msgs::Point32 mine_center_;
  geometry_msgs::PoseStamped EMPTY_POSE;
  virtual void controlLoop();
  void landmineDetected(bool left_coil = true) const;
  void landmineDetected(double x, double y) const;
  geometry_msgs::PoseStamped getRobotPose() const;
  geometry_msgs::PoseStamped getCoilPose(bool left_coil = true) const;
  geometry_msgs::PoseStamped getLeftCoilPose() const;
  geometry_msgs::PoseStamped getRightCoilPose() const;
  void publishLandminePose(double x, double y) const;
  void publishFakeLandminePose(double x, double y, double radius) const;
  void publishFilteredCoilSignals() const;
  geometry_msgs::PolygonStamped landmine_;
  void setScanning(bool scanning);
  void reset();
};
}

#endif /* _HRATC2017_ENTRIES_LANDMINE_ANALYZER_H_ */
