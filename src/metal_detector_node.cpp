/**
 *  This source file implements the main function that calls the Metal Detector
 *node controller.
 *
 *  Version: 0.0.1
 *  Created on: 30/01/2017
 *  Modified on: 30/01/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "entries/metal_detector_node.h"

/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "metal_detector_node");
  ros::NodeHandle nh;
  entries::MetalDetectorNode node(&nh);
  node.spin();
  return 0;
}
