/**
 *  This header file defines the LandminesLayer class, which is based
 *on the constmap_2d::Layer class.
 *
 *  Version: 0.0.2
 *  Created on: 01/02/2017
 *  Modified on: 01/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_LAYERS_LANDMINES_LAYER_H_
#define _HRATC2017_LAYERS_LANDMINES_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

namespace hratc2017
{
class LandminesLayer : public costmap_2d::Layer
{
public:
  LandminesLayer();
  virtual ~LandminesLayer();
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x,
                            double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i,
                           int min_j, int max_i, int max_j);

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level);

  double mark_x_, mark_y_;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>* dsrv_;
};
}
#endif /* _HRATC2017_LAYERS_LANDMINES_LAYER_H_ */
