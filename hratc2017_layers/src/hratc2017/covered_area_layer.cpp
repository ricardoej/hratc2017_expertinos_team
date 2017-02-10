/**
 *  This source file implements the CoveredareaLayer class, which is based
 *on the constmap_2d::Layer class.
 *
 *  Version: 0.0.2
 *  Created on: 01/02/2017
 *  Modified on: 01/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/covered_area_layer.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(hratc2017::CoveredAreaLayer, costmap_2d::Layer)

namespace hratc2017
{

/**
 * @brief CoveredAreaLayer::CoveredAreaLayer
 */
CoveredAreaLayer::CoveredAreaLayer() : dsrv_(NULL) {}

/**
 * @brief CoveredAreaLayer::~CoveredAreaLayer
 */
CoveredAreaLayer::~CoveredAreaLayer()
{
  if (dsrv_)
  {
    delete dsrv_;
    dsrv_ = NULL;
  }
}

/**
 * @brief CoveredAreaLayer::onInitialize
 */
void CoveredAreaLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  rolling_window_ = layered_costmap_->isRolling();
  global_frame_ = layered_costmap_->getGlobalFrameID();
  int default_value;
  nh.param("default_value", default_value, DEFAULT_VALUE);
  default_value_ = default_value < 0 || default_value > 255
                       ? DEFAULT_VALUE
                       : (unsigned char)default_value;
  ROS_INFO("    Covered Area default cost value: %d", default_value);
  nh.param("radius", radius_, DETECTION_RADIUS);
  ROS_INFO("    Detection radius : %lf", radius_);
  current_ = true;
  matchSize();
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType
      cb = boost::bind(&CoveredAreaLayer::reconfigureCallback, this, _1, _2);
  dsrv_->setCallback(cb);
}

/**
 * @brief CoveredAreaLayer::matchSize
 */
void CoveredAreaLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(),
            master->getResolution(), master->getOriginX(),
            master->getOriginY());
}

/**
 * @brief CoveredAreaLayer::reset
 */
void CoveredAreaLayer::reset()
{
  deactivate();
  resetMaps();
  current_ = true;
  activate();
}

/**
 * @brief CoveredAreaLayer::reconfigureCallback
 * @param config
 * @param level
 */
void CoveredAreaLayer::reconfigureCallback(
    costmap_2d::GenericPluginConfig& config, uint32_t level)
{
  enabled_ = config.enabled;
}

/**
 * @brief CoveredAreaLayer::updateBounds
 * @param robot_x
 * @param robot_y
 * @param robot_yaw
 * @param min_x
 * @param min_y
 * @param max_x
 * @param max_y
 */
void CoveredAreaLayer::updateBounds(double robot_x, double robot_y,
                                    double robot_yaw, double* min_x,
                                    double* min_y, double* max_x, double* max_y)
{
  if (rolling_window_)
  {
    updateOrigin(robot_x - getSizeInMetersX() / 2,
                 robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_)
  {
    return;
  }
  useExtraBounds(min_x, min_y, max_x, max_y);
  /*double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
  unsigned int mx;
  unsigned int my;
  if (worldToMap(mark_x, mark_y, mx, my))
  {
    setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
  }
  *min_x = std::min(*min_x, mark_x);
  *min_y = std::min(*min_y, mark_y);
  *max_x = std::max(*max_x, mark_x);
  *max_y = std::max(*max_y, mark_y);*/
}

/**
 * @brief CoveredAreaLayer::updateCosts
 * @param master_grid
 * @param min_i
 * @param min_j
 * @param max_i
 * @param max_j
 */
void CoveredAreaLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                   int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
  {
    return;
  }
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == default_value_)
      {
        master_grid.setCost(i, j, costmap_[index]);
      }
    }
  }
}
}
