/**
 *  This header file defines the ROSNode class. It is highly recommended
 *  whenever an oriented-object programming ROS Node class is created
 *  to enhance this one.
 *
 *  Version: 1.4.0
 *  Created on: 05/10/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_ROS_NODE_H_
#define _UTILITIES_ROS_NODE_H_

#include <ros/ros.h>
#include "utilities/exception.h"

namespace utilities
{
class ROSNode
{
public:
  virtual ~ROSNode(); // destructor
  virtual void spin(); // standard spin method (according to the given loop rate)

protected:
  ROSNode(ros::NodeHandle *nh, float loop_rate); // protected constructor
  ros::NodeHandle* getNodeHandle() const;
  std::string getName() const;
  bool ok() const;
  void shutdown() const;
  void shutdown(std::string message) const;
  
private:
  float loop_rate_; // positive spin rate
  std::string name_; // ROS node name
  ros::NodeHandle *nh_; // private ros node handle (has-a relationship)
  virtual void controlLoop() = 0;
};
}

#endif // _UTILITIES_ROS_NODE_H_
