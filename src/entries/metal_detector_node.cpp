/**
 *  This source file implements the MetalDetectorNode class, which is
 *based on the ROSNode helper class. It controls the metal_detector_node.
 *
 *  Version: 0.0.1
 *  Created on: 30/01/2017
 *  Modified on: 30/01/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "entries/metal_detector_node.h"

namespace entries
{

/**
 * @brief MetalDetectorNode::MetalDetectorNode
 * @param nh
 */
MetalDetectorNode::MetalDetectorNode(ros::NodeHandle* nh)
    : ROSNode(nh, 30)
{
}

/**
 * @brief MetalDetectorNode::~MetalDetectorNode
 */
MetalDetectorNode::~MetalDetectorNode()
{
}

/**
 * @brief SystemUserInterfaceNode::controlLoop
 */
void MetalDetectorNode::controlLoop() {}

}
