/**
 *  This header file defines the LandmineAnalyzer class, which is based
 *on the ROSNode class. It controls the landmine_analyzer_node.
 *
 *  Version: 0.0.1
 *  Created on: 30/01/2017
 *  Modified on: 01/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *          Luis Victor Pessiqueli Bonin (luis-bonin@unifei.edu.br)
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

#define COIL_SIGNAL_THRESHOLD 0.6
#define SAMPLING_END_INTERVAL 2.0

namespace hratc2017
{

class LandmineAnalyzer : public utilities::ROSNode
{
public:
  LandmineAnalyzer(ros::NodeHandle* nh);
  virtual ~LandmineAnalyzer();

private:
  tf::TransformListener tf_;
  ros::Subscriber coils_sub_;
  ros::Publisher set_mine_pub_;
  Coils coils_;
  geometry_msgs::PoseStamped EMPTY_POSE;
  virtual void controlLoop();
  void landmineDetected(bool left_coil = true) const;
  void landmineDetected(double x, double y) const;
  void coilsCallback(const metal_detector_msgs::Coil::ConstPtr& msg);
  geometry_msgs::PoseStamped getRobotPose() const;
  geometry_msgs::PoseStamped getCoilPose(bool left_coil = true) const;
  geometry_msgs::PoseStamped getLeftCoilPose() const;
  geometry_msgs::PoseStamped getRightCoilPose() const;
  geometry_msgs::PolygonStamped landmine_;
  void reset();
  bool sampling_;
  double sampling_end_interval_;
};
}

#endif /* _HRATC2017_ENTRIES_LANDMINE_ANALYZER_H_ */
