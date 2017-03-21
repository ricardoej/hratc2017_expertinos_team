/**
 *  This source file implements the main function that calls the Metal Scanner
 *node.
 *
 *  Version: 0.0.1
 *  Created on: 10/02/2017
 *  Modified on: 21/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/metal_scanner.h"

/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "metal_scanner_node");
  ros::NodeHandle nh;
  hratc2017::MetalScanner* node = new hratc2017::MetalScanner(&nh);
  node->spin();
  delete node;
  return 0;
}
