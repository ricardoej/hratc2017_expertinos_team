/**
 *  This header file defines the WaypointsController class, which is based
 *on the ROSNode class. It controls the waypoints_controller_node.
 *
 *  Version: 0.0.1
 *  Created on: 06/02/2017
 *  Modified on: 06/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *          Ricardo Emerson Julio (ricardoej@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_ENTRIES_WAYPOINTS_CONTROLLER_H_
#define _HRATC2017_ENTRIES_WAYPOINTS_CONTROLLER_H_

#include <ros/ros.h>
#include "utilities/ros_node.h"

namespace hratc2017
{

class WaypointsController : public utilities::ROSNode
{
public:
  WaypointsController(ros::NodeHandle* nh);
  virtual ~WaypointsController();

private:
  virtual void controlLoop();
};
}

#endif /* _HRATC2017_ENTRIES_WAYPOINTS_CONTROLLER_H_ */
