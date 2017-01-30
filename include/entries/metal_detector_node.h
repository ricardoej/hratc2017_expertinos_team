/**
 *  This header file defines the MetalDetectorNode class, which is based
 *on the ROSNode class. It controls the metal_detector_node.
 *
 *  Version: 0.0.1
 *  Created on: 30/01/2017
 *  Modified on: 30/01/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_METAL_DETECTOR_NODE_H_
#define _HRATC2017_METAL_DETECTOR_NODE_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <utilities/ros_node.h>

namespace entries
{

class MetalDetectorNode : public utilities::ROSNode
{
public:
  MetalDetectorNode(ros::NodeHandle* nh);
  virtual ~MetalDetectorNode();

private:
  virtual void controlLoop();
};
}

#endif /* _HRATC2017_METAL_DETECTOR_NODE_H_ */
