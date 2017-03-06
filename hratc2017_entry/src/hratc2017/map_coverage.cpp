/**
 *  This source file implments the Map class.
 *
 *  Version: 1.0.1
 *  Created on: 06/02/2017
 *  Modified on: 02/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *          Ricardo Emerson Julio (ricardoej@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/map_coverage.h"

namespace hratc2017
{

/**
 * @brief MapCoverage::MapCoverage
 * @param msg
 * @param type
 */
MapCoverage::MapCoverage(visualization_msgs::MarkerArray::ConstPtr msg, std::string type,
         double map_coverage_offset, double map_coverage_margin)
    : map_coverage_offset_(map_coverage_offset),
      map_coverage_margin_(map_coverage_margin)
{
  if (msg->markers.size() != 4)
  {
    throw utilities::Exception("Map must have four corners!!!");
  }
  left_bottom_corner_ = msg->markers[0].pose.position;
  right_bottom_corner_ = msg->markers[1].pose.position;
  right_top_corner_ = msg->markers[2].pose.position;
  left_top_corner_ = msg->markers[3].pose.position;
  if (type == "relative")
  {
    double x_size(std::abs(left_top_corner_.x - right_top_corner_.x));
    double y_size(std::abs(left_bottom_corner_.x - right_bottom_corner_.x));
    left_bottom_corner_.x = -1 * x_size / 2 + map_coverage_margin_;
    left_bottom_corner_.y = -1 * y_size / 2 + map_coverage_margin_;
    left_bottom_corner_.z = 0;
    left_top_corner_.x = -1 * x_size / 2 + map_coverage_margin_;
    left_top_corner_.y = y_size / 2 - map_coverage_margin_;
    left_top_corner_.z = 0;
    right_top_corner_.x = x_size / 2 - map_coverage_margin_;
    right_top_corner_.y = y_size / 2 - map_coverage_margin_;
    right_top_corner_.z = 0;
    right_bottom_corner_.x = x_size / 2 - map_coverage_margin_;
    right_bottom_corner_.y = -1 * y_size / 2 + map_coverage_margin_;
    right_bottom_corner_.z = 0;
  }
  generateWaypoints();
}

/**
 * @brief MapCoverage::MapCoverage
 * @param left_bottom_corner
 * @param left_top_corner
 * @param right_top_corner
 * @param right_bottom_corner
 */
MapCoverage::MapCoverage(geometry_msgs::Point left_bottom_corner,
         geometry_msgs::Point left_top_corner,
         geometry_msgs::Point right_top_corner,
         geometry_msgs::Point right_bottom_corner, double map_coverage_offset,
         double map_coverage_margin)
    : map_coverage_offset_(map_coverage_offset),
      map_coverage_margin_(map_coverage_margin),
      left_bottom_corner_(left_bottom_corner),
      left_top_corner_(left_top_corner), right_top_corner_(right_top_corner),
      right_bottom_corner_(right_bottom_corner)
{
  generateWaypoints();
}

/**
 * @brief MapCoverage::~MapCoverage
 */
MapCoverage::~MapCoverage() {}

/**
 * @brief MapCoverage::getNextWaypoint
 * @return
 */
geometry_msgs::Point MapCoverage::getNextWaypoint() const { return waypoints_.front(); }

/**
 * @brief MapCoverage::getX
 * @return
 */
double MapCoverage::getX() const
{
  return waypoints_.front().x;
}

/**
 * @brief MapCoverage::getY
 * @return
 */
double MapCoverage::getY() const
{
  return waypoints_.front().y;
}

/**
 * @brief MapCoverage::empty
 * @return
 */
bool MapCoverage::empty() const { return waypoints_.empty(); }

/**
 * @brief MapCoverage::pop
 */
void MapCoverage::pop() { waypoints_.pop(); }

/**
 * @brief MapCoverage::str
 * @return
 */
std::string MapCoverage::str() const
{
  std::stringstream ss;
  ss << "left bottom: (" << left_bottom_corner_.x << ", "
     << left_bottom_corner_.y;
  ss << "), left top: (" << left_top_corner_.x << ", " << left_top_corner_.y;
  ss << "), right bottom: (" << right_bottom_corner_.x << ", "
     << right_bottom_corner_.y;
  ss << "), right top: (" << right_top_corner_.x << ", " << right_top_corner_.y;
  ss << "), waypoints: ";
  return ss.str();
}

/**
 * @brief MapCoverage::c_str
 * @return
 */
const char* MapCoverage::c_str() const { return str().c_str(); }

/**
 * @brief MapCoverage::generateWaypoints generates the waypoint/map coverage strategy
 */
void MapCoverage::generateWaypoints()
{
  clear();
  geometry_msgs::Point last_waypoint(right_bottom_corner_);
  geometry_msgs::Point current_waypoint(left_bottom_corner_);
  while (current_waypoint.x <= last_waypoint.x)
  {
    waypoints_.push(current_waypoint);
    current_waypoint.y = current_waypoint.y == left_bottom_corner_.y
                             ? left_top_corner_.y
                             : left_bottom_corner_.y;
    waypoints_.push(current_waypoint);
    current_waypoint.x += map_coverage_offset_;
  }
}

/**
 * @brief MapCoverage::clear
 */
void MapCoverage::clear()
{
  while (!waypoints_.empty())
  {
    waypoints_.pop();
  }
}
}
