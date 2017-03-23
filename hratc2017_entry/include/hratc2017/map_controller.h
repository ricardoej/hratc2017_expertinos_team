/**
 *  This source file implements the MapController class, which is
 *based on the ROSNode helper class. It controls the map_Controller_node.
 *
 *  Version: 1.0.0
 *  Created on: 21/03/2017
 *  Modified on: 21/03/2017
 *  Author: Ricardo Emerson Julio (ricardoej@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_ENTRIES_MAP_CONTROLLER_H_
#define _HRATC2017_ENTRIES_MAP_CONTROLLER_H_

#include <ros/ros.h>
#include "utilities/ros_node.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>

namespace hratc2017
{
class MapController : public utilities::ROSNode
{
public:
  MapController(ros::NodeHandle* nh);
  virtual ~MapController();

private:
  bool has_utm_initial_pose_;
  bool initial_pose_sent_;
  nav_msgs::Odometry utm_initial_pose_;
  ros::Subscriber gps_odom_sub_;
  ros::Publisher initial_pose_pub_;
  virtual void controlLoop();
  void gpsOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void cornersCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
};
}

#endif /* _HRATC2017_ENTRIES_MAP_CONTROLLER_H_ */
