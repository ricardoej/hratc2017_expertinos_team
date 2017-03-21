/**
 *  This source file implements the main function that calls the Landmine
 *Analyzer node controller.
 *
 *  Version: 0.0.1
 *  Created on: 30/01/2017
 *  Modified on: 21/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/landmine_analyzer.h"

/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "landmine_analyzer_node");
  ros::NodeHandle nh;
  hratc2017::LandmineAnalyzer* node = new hratc2017::LandmineAnalyzer(&nh);
  node->spin();
  delete node;
  return 0;
}
