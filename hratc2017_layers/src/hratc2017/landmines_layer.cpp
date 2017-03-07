/**
 *  This source file implements the LandminesLayer class, which is based
 *on the costmap_2d::Layer class.
 *
 *  Version: 0.0.3
 *  Created on: 01/02/2017
 *  Modified on: 15/02/2017
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
LandminesLayer::LandminesLayer()
    : dsrv_(NULL), radius_(0.5), num_parts_(12),
      min_x_(std::numeric_limits<float>::max()),
      min_y_(std::numeric_limits<float>::max()),
      max_x_(-std::numeric_limits<float>::max()),
      max_y_(-std::numeric_limits<float>::max())
{
}

/**
 * @brief LandminesLayer::~LandminesLayer
 */
LandminesLayer::~LandminesLayer()
{
  landmines_sub_.shutdown();
  fake_landmines_sub_.shutdown();
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
  nh.param("radius", radius_, 0.15);
  ROS_INFO("    Landmine radius: %lf", radius_);
  nh.param("num_parts", num_parts_, 12);
  ROS_INFO("    Number of parts of landmine circumference: %d", num_parts_);
  std::string source;
  current_ = true;
  nh.param("landmines_topic", source, std::string("/HRATC_FW/set_mine"));
  ROS_INFO("    Subscribed to topic: %s", source.c_str());
  landmines_sub_ =
      nh.subscribe(source, 10, &LandminesLayer::landminesCallback, this);
  nh.param("fake_landmines_topic", source, std::string("/HRATC_FW/set_fake_mine"));
  ROS_INFO("    Subscribed to topic: %s", source.c_str());
  fake_landmines_sub_ =
      nh.subscribe(source, 10, &LandminesLayer::fakeLandminesCallback, this);
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType
      cb = boost::bind(&LandminesLayer::reconfigureCallback, this, _1, _2);
  dsrv_->setCallback(cb);
}

/**
 * @brief LandminesLayer::reconfigureCallback
 * @param config
 * @param level
 */
void LandminesLayer::reconfigureCallback(
    costmap_2d::GenericPluginConfig& config, uint32_t level)
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
  if (!enabled_ || (landmines_.empty() && fake_landmines_.empty()))
  {
    return;
  }
  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);
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
  if (!enabled_ || (landmines_.empty() && fake_landmines_.empty()))
  {
    return;
  }
  if (radius_ <= 0 || num_parts_ <= 0)
  {
    unsigned int xc, yc;
    for (int i(0); i < landmines_.size(); i++)
    {
      if (master_grid.worldToMap(landmines_[i].x, landmines_[i].y, xc, yc))
      {
        master_grid.setCost(xc, yc, costmap_2d::LETHAL_OBSTACLE);
      }
    }
    for (int i(0); i < fake_landmines_.size(); i++)
    {
      if (master_grid.worldToMap(fake_landmines_[i].x, fake_landmines_[i].y, xc, yc))
      {
        master_grid.setCost(xc, yc, costmap_2d::LETHAL_OBSTACLE);
      }
    }
    return;
  }
  double angle;
  std::vector<geometry_msgs::Point> circumference;
  for (int i(0); i < landmines_.size(); i++)
  {
    angle = 0.0;
    circumference.clear();
    while (angle < 2 * M_PI)
    {
      geometry_msgs::Point p;
      p.x = radius_ * cos(angle) + landmines_[i].x;
      p.y = radius_ * sin(angle) + landmines_[i].y;
      circumference.push_back(p);
      angle += M_PI / radius_ / num_parts_;
    }
    master_grid.setConvexPolygonCost(circumference,
                                     costmap_2d::LETHAL_OBSTACLE);
  }
  for (int i(0); i < fake_landmines_.size(); i++)
  {
    if (fake_landmines_[i].z <= 0)
    {
      unsigned int xc, yc;
      if (master_grid.worldToMap(fake_landmines_[i].x, fake_landmines_[i].y, xc, yc))
      {
        master_grid.setCost(xc, yc, costmap_2d::LETHAL_OBSTACLE);
      }
      continue;
    }
    angle = 0.0;
    circumference.clear();
    while (angle < 2 * M_PI)
    {
      geometry_msgs::Point p;
      p.x = radius_ * cos(angle) + fake_landmines_[i].x;
      p.y = radius_ * sin(angle) + fake_landmines_[i].y;
      circumference.push_back(p);
      angle += M_PI / fake_landmines_[i].z / num_parts_;
    }
    master_grid.setConvexPolygonCost(circumference,
                                     costmap_2d::LETHAL_OBSTACLE);
  }
}

/**
 * @brief hratc2017::LandminesLayer::setMineCallback
 * @param msg
 */
void LandminesLayer::landminesCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_INFO("Adding a new landmine at (%f, %f).", msg->pose.position.x,
            msg->pose.position.y);
  min_x_ = std::min(min_x_, msg->pose.position.x - radius_);
  min_y_ = std::min(min_y_, msg->pose.position.y - radius_);
  max_x_ = std::max(max_x_, msg->pose.position.x + radius_);
  max_y_ = std::max(max_y_, msg->pose.position.y + radius_);
  landmines_.push_back(msg->pose.position);
}

/**
 * @brief hratc2017::LandminesLayer::setMineCallback
 * @param msg
 */
void LandminesLayer::fakeLandminesCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_INFO("Adding a new fake landmine at (%f, %f).", msg->pose.position.x,
            msg->pose.position.y);
  min_x_ = std::min(min_x_, msg->pose.position.x - msg->pose.position.z);
  min_y_ = std::min(min_y_, msg->pose.position.y - msg->pose.position.z);
  max_x_ = std::max(max_x_, msg->pose.position.x + msg->pose.position.z);
  max_y_ = std::max(max_y_, msg->pose.position.y + msg->pose.position.z);
  //fake_landmines_.push_back(msg->pose.position);
}
}
