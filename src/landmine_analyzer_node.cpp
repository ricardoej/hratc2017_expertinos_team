/**
 *  This source file implements the main function that calls the Landmine
 *Analyzer node controller.
 *
 *  Version: 0.0.1
 *  Created on: 30/01/2017
 *  Modified on: 30/01/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "entries/landmine_analyzer.h"

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
  entries::LandmineAnalyzer node(&nh);
  node.spin();
  return 0;
}
