/**
 *  This source file implements the LandminesLayer class, which is based
 *on the constmap_2d::Layer class.
 *
 *  Version: 0.0.2
 *  Created on: 01/02/2017
 *  Modified on: 03/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/landmines_layer.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(hratc2017::LandminesLayer, costmap_2d::Layer)

namespace hratc2017
{

/**
 * @brief LandminesLayer::LandminesLayer
 */
LandminesLayer::LandminesLayer() : dsrv_(NULL), radius_(0.5) {}

/**
 * @brief LandminesLayer::~LandminesLayer
 */
LandminesLayer::~LandminesLayer()
{
  set_mine_sub_.shutdown();
  if (dsrv_)
  {
    delete dsrv_;
    dsrv_ = NULL;
  }
}

/**
 * @brief LandminesLayer::onInitialize
 */
void LandminesLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);

  nh.param("radius", radius_, 0.5);
  ROS_INFO("    Landmine radius: %lf", radius_);

  std::string source;
  nh.param("topic", source, std::string("/HRATC_FW/set_mine"));
  ROS_INFO("    Subscribed to topic: %s", source.c_str());

  current_ = true;

  set_mine_sub_ = nh.subscribe(source, 10, &LandminesLayer::setMineCallback, this);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType
      cb = boost::bind(&LandminesLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

/**
 * @brief LandminesLayer::reconfigureCB
 * @param config
 * @param level
 */
void LandminesLayer::reconfigureCB(costmap_2d::GenericPluginConfig& config,
                                   uint32_t level)
{
  enabled_ = config.enabled;
}

/**
 * @brief LandminesLayer::updateBounds
 * @param robot_x
 * @param robot_y
 * @param robot_yaw
 * @param min_x
 * @param min_y
 * @param max_x
 * @param max_y
 */
void LandminesLayer::updateBounds(double robot_x, double robot_y,
                                  double robot_yaw, double* min_x,
                                  double* min_y, double* max_x, double* max_y)
{
  /*if (!enabled_)
  {
    return;
  }*/
}

/**
 * @brief LandminesLayer::updateCosts
 * @param master_grid
 * @param min_i
 * @param min_j
 * @param max_i
 * @param max_j
 */
void LandminesLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i,
                                 int min_j, int max_i, int max_j)
{
  if (!enabled_)
  {
    return;
  }
  unsigned int mx;
  unsigned int my;
  for (int i(0); i < marks_.size(); i++)
  {
    if (master_grid.worldToMap(marks_[i].x, marks_[i].y, mx, my))
    {
      master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

/**
 * @brief hratc2017::LandminesLayer::setMineCallback
 * @param msg
 */
void LandminesLayer::setMineCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  ROS_INFO("Adding a new landmine at (%f, %f).", msg->pose.position.x, msg->pose.position.y);
  marks_.push_back(msg->pose.position);
}
}
