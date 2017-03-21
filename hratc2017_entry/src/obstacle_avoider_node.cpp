/**
 *  This source file implements the main function that calls the Obstacle Avoider node.
 *
 *  Version: 1.1.2
 *  Created on: 20/03/2017
 *  Modified on: 21/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/obstacle_avoider.h"

/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_avoider_node");
  ros::NodeHandle nh;
  hratc2017::ObstacleAvoider* node = new hratc2017::ObstacleAvoider(&nh);
  node->spin();
  delete node;
  return 0;
}
