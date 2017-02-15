/**
 *  This source file implements the ObstaclesLayer class, which is based
 *on the costmap_2d::Layer class.
 *
 *  Version: 0.0.6
 *  Created on: 15/02/2017
 *  Modified on: 15/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *          Audeliano Wolian Li (audeliano@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/obstacles_layer.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(hratc2017::ObstaclesLayer, costmap_2d::Layer)

namespace hratc2017
{

/**
 * @brief ObstaclesLayer::ObstaclesLayer
 */
ObstaclesLayer::ObstaclesLayer()
    : dsrv_(NULL), radius_(0.5), num_parts_(12),
      min_x_(std::numeric_limits<float>::max()),
      min_y_(std::numeric_limits<float>::max()),
      max_x_(-std::numeric_limits<float>::max()),
      max_y_(-std::numeric_limits<float>::max())
{
}

/**
 * @brief ObstaclesLayer::~ObstaclesLayer
 */
ObstaclesLayer::~ObstaclesLayer()
{
  obstacles_sub_.shutdown();
  if (dsrv_)
  {
    delete dsrv_;
    dsrv_ = NULL;
  }
}

/**
 * @brief ObstaclesLayer::onInitialize
 */
void ObstaclesLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  nh.param("radius", radius_, 0.15);
  ROS_INFO("    Obstacle radius: %lf", radius_);
  nh.param("num_parts", num_parts_, 12);
  ROS_INFO("    Number of parts of obstacle circumference: %d", num_parts_);
  std::string source;
  nh.param("obstacles_topic", source, std::string("obstacles"));
  ROS_INFO("    Subscribed to topic: %s", source.c_str());
  current_ = true;
  obstacles_sub_ =
      nh.subscribe(source, 10, &ObstaclesLayer::obstaclesCallback, this);
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType
      cb = boost::bind(&ObstaclesLayer::reconfigureCallback, this, _1, _2);
  dsrv_->setCallback(cb);
}

/**
 * @brief ObstaclesLayer::reconfigureCallback
 * @param config
 * @param level
 */
void ObstaclesLayer::reconfigureCallback(
    costmap_2d::GenericPluginConfig& config, uint32_t level)
{
  enabled_ = config.enabled;
}

/**
 * @brief ObstaclesLayer::updateBounds
 * @param robot_x
 * @param robot_y
 * @param robot_yaw
 * @param min_x
 * @param min_y
 * @param max_x
 * @param max_y
 */
void ObstaclesLayer::updateBounds(double robot_x, double robot_y,
                                  double robot_yaw, double* min_x,
                                  double* min_y, double* max_x, double* max_y)
{
  if (!enabled_ || obstacles_.empty())
  {
    return;
  }
  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);
}

/**
 * @brief ObstaclesLayer::updateCosts
 * @param master_grid
 * @param min_i
 * @param min_j
 * @param max_i
 * @param max_j
 */
void ObstaclesLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i,
                                 int min_j, int max_i, int max_j)
{
  if (!enabled_ || obstacles_.empty())
  {
    return;
  }
  if (radius_ <= 0 || num_parts_ <= 0)
  {
    unsigned int xc, yc;
    for (int i(0); i < obstacles_.size(); i++)
    {
      if (master_grid.worldToMap(obstacles_[i].x, obstacles_[i].y, xc, yc))
      {
        master_grid.setCost(xc, yc, costmap_2d::LETHAL_OBSTACLE);
      }
    }
    return;
  }
  double angle;
  std::vector<geometry_msgs::Point> circumference;
  for (int i(0); i < obstacles_.size(); i++)
  {
    angle = 0.0;
    circumference.clear();
    while (angle < 2 * M_PI)
    {
      geometry_msgs::Point p;
      p.x = radius_ * cos(angle) + obstacles_[i].x;
      p.y = radius_ * sin(angle) + obstacles_[i].y;
      circumference.push_back(p);
      angle += M_PI / radius_ / num_parts_;
    }
    master_grid.setConvexPolygonCost(circumference,
                                     costmap_2d::LETHAL_OBSTACLE);
  }
}

/**
 * @brief ObstaclesLayer::obstaclesCallback
 * @param msg
 */
void ObstaclesLayer::obstaclesCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_DEBUG("Adding a new obstacle at (%f, %f).", msg->pose.position.x,
            msg->pose.position.y);
  min_x_ = std::min(min_x_, msg->pose.position.x - radius_);
  min_y_ = std::min(min_y_, msg->pose.position.y - radius_);
  max_x_ = std::max(max_x_, msg->pose.position.x + radius_);
  max_y_ = std::max(max_y_, msg->pose.position.y + radius_);
  geometry_msgs::Point obstacle(msg->pose.position);
  /*obstacle.x = msg->pose.position.x;
  obstacle.y = msg->pose.position.y;*/
  obstacles_.push_back(obstacle);
}
}
