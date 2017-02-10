/**
 *  This header file defines the CoveredAreaLayer class, which is based
 *on the constmap_2d::Layer class.
 *
 *  Version: 0.0.2
 *  Created on: 01/02/2017
 *  Modified on: 09/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_LAYERS_COVERED_AREA_LAYER_H_
#define _HRATC2017_LAYERS_COVERED_AREA_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#define DEFAULT_VALUE 64
#define DETECTION_RADIUS 0.15 //m

namespace hratc2017
{
class CoveredAreaLayer : public costmap_2d::CostmapLayer
{
public:
  CoveredAreaLayer();
  virtual ~CoveredAreaLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x,
                            double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i,
                           int min_j, int max_i, int max_j);
  bool isDiscretized() { return true; }

  virtual void matchSize();
  virtual void reset();

private:
  double radius_;
  bool rolling_window_;
  std::string global_frame_;
  void reconfigureCallback(costmap_2d::GenericPluginConfig& config,
                           uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>* dsrv_;
};
}
#endif /* _HRATC2017_LAYERS_COVERED_AREA_LAYER_H_ */
