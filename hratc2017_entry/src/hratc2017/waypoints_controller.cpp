/**
 *  This source file implements the WaypointsController class, which is
 *based on the ROSNode helper class. It controls the waypoints_controller_node.
 *
 *  Version: 0.0.1
 *  Created on: 06/02/2017
 *  Modified on: 06/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *          Ricardo Emerson Julio (ricardoej@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/waypoints_controller.h"

namespace hratc2017
{

/**
 * @brief WaypointsController::WaypointsController
 * @param nh
 */
WaypointsController::WaypointsController(ros::NodeHandle* nh) : ROSNode(nh, 30)
{
}

/**
 * @brief WaypointsController::~WaypointsController
 */
WaypointsController::~WaypointsController() {}

/**
 * @brief WaypointsController::controlLoop
 */
void WaypointsController::controlLoop() {}
}
