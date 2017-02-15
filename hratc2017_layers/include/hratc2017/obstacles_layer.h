/**
 *  This header file defines the ObstaclesLayer class, which is based
 *on the costmap_2d::Layer class.
 *
 *  Version: 0.0.6
 *  Created on: 15/02/2017
 *  Modified on: 15/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *          Audeliano Wolian Li (audeliano@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_LAYERS_OBSTACLES_LAYER_H_
#define _HRATC2017_LAYERS_OBSTACLES_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>

namespace hratc2017
{
class ObstaclesLayer : public costmap_2d::Layer
{
public:
  ObstaclesLayer();
  virtual ~ObstaclesLayer();
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x,
                            double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i,
                           int min_j, int max_i, int max_j);

private:
  void reconfigureCallback(costmap_2d::GenericPluginConfig& config,
                           uint32_t level);
  void obstaclesCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  ros::Subscriber obstacles_sub_;
  bool new_obstacle_;
  std::vector<geometry_msgs::Point> obstacles_;
  double min_x_, min_y_, max_x_, max_y_;
  int num_parts_;
  double radius_;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>* dsrv_;
};
}
#endif /* _HRATC2017_LAYERS_OBSTACLES_LAYER_H_ */
