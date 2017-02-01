/**
 *  This source file implements the ROSNode class.
 *
 *  Version: 1.4.0
 *  Created on: 05/10/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/ros_node.h"

namespace utilities
{

/**
 * @brief ROSNode::Node builds an ROSNode object given a ROS NodeHandle
 * and also its desired spin rate.
 * @param nh must NOT be NULL.
 * @param loop_rate must be positive.
 */
ROSNode::ROSNode(ros::NodeHandle* nh, float loop_rate) : loop_rate_(loop_rate)
{
  if (!nh)
  {
    ROS_FATAL("ROS node handle must not be NULL!!!");
    ros::shutdown();
    return;
  }
  if (loop_rate <= 0)
  {
    ROS_FATAL("The node spin rate must be positive!!!");
    ros::shutdown();
    return;
  }
  nh_ = nh;
  name_ = ros::this_node::getName();
}

/**
 * @brief ROSNode::~ROSNode announces that this ROS node will shutdown. This
 * will not destruct the used ROS NodeHandle object.
 */
ROSNode::~ROSNode() {}

/**
 * @brief ROSNode::spin loops while there is not another instance
 * of this node with this node name, or while the Ctrl+C buttons
 * is not pressed at the terminal. In addition, it periodicly updates
 * this node, as well as, controls the updates rate.
 */
void ROSNode::spin()
{
  ros::Rate loop_rate(loop_rate_);
  ROS_INFO("%s is ON!!!", name_.c_str());
  while (nh_->ok())
  {
    controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

/**
 * @brief ROSNode::shutdown
 */
void ROSNode::shutdown() const
{
  ROS_WARN("%s is OFF now!!!", name_.c_str());
  nh_->shutdown();
}

/**
 * @brief ROSNode::shutdown
 * @param message
 */
void ROSNode::shutdown(std::string message) const
{
  shutdown();
  throw utilities::Exception(message);
}

/**
 * @brief ROSNode::getNodeHandle encapsulates this ROS node handle.
 * @return a pointer to an internal member that handles this node.
 */
ros::NodeHandle* ROSNode::getNodeHandle() const { return nh_; }

/**
 * @brief ROSNode::getName encapsulates this ROS node name.
 * @return this ROS node whole name.
 */
std::string ROSNode::getName() const { return name_; }

/**
 * @brief ROSNode::ok
 * @return if this ROS node controller is still running properly.
 */
bool ROSNode::ok() const
{
  return nh_->ok();
}
}
