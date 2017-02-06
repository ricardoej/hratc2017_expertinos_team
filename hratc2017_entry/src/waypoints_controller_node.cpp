/**
 *  This source file implements the main function that calls the Waypoints
 *Controller node.
 *
 *  Version: 0.0.1
 *  Created on: 06/02/2017
 *  Modified on: 06/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/waypoints_controller.h"

/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoints_controller_node");
  ros::NodeHandle nh;
  hratc2017::WaypointsController node(&nh);
  node.spin();
  return 0;
}
