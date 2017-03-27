/**
 *  This source file implements the main function that calls the Map
 *Controller node.
 *
 *  Version: 0.0.1
 *  Created on: 21/03/2017
 *  Modified on: 21/03/2017
 *  Author: Ricardo Emerson Julio (ricardoej@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/pose_estimator.h"

/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_estimator_node");
  ros::NodeHandle nh;
  hratc2017::PoseEstimator* node = new hratc2017::PoseEstimator(&nh);
  node->spin();
  delete node;
  return 0;
}
