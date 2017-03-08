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
         double map_coverage_offset, double map_coverage_margin, double landmine_radius_area)
    : map_coverage_offset_(map_coverage_offset),
      map_coverage_margin_(map_coverage_margin),
      landmine_radius_area_(landmine_radius_area)
{
  if (msg->markers.size() != 4)
  {
    throw utilities::Exception("Map must have four corners!!!");
  }
  left_bottom_corner_ = msg->markers[0].pose.position;
  right_bottom_corner_ = msg->markers[1].pose.position;
  right_top_corner_ = msg->markers[2].pose.position;
  left_top_corner_ = msg->markers[3].pose.position;
  if (type == RELATIVE_MAP_COORDINATE_TYPE)
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
         double map_coverage_margin, double landmine_radius_area)
    : map_coverage_offset_(map_coverage_offset),
      map_coverage_margin_(map_coverage_margin),
      left_bottom_corner_(left_bottom_corner),
      left_top_corner_(left_top_corner), right_top_corner_(right_top_corner),
      right_bottom_corner_(right_bottom_corner),
      landmine_radius_area_(landmine_radius_area)
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
geometry_msgs::Point MapCoverage::getNextWaypoint() const 
{
  geometry_msgs::Point next_point = waypoints_.front();

  if (isInsideMineArea(next_point))
  {
      if (next_point.y > 0)
      {
        next_point.y -= landmine_radius_area_;
      }
      else
      {
        next_point.y += landmine_radius_area_; 
      }
  }
  
  return next_point;
}

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
void MapCoverage::pop() { waypoints_.pop_front(); }

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
const char* MapCoverage::c_str() const 
{
  std::string buf("Map coverage strategy:");
  for (std::list<geometry_msgs::Point>::const_iterator ci = waypoints_.begin(); ci != waypoints_.end(); ++ci)
  {
    std::ostringstream stringStream;
    stringStream << " (" << ci->x << ", " << ci->y << ", " << ci->z << ")";
    buf.append(stringStream.str());
  }
  return buf.c_str(); 
}

/**
 * @brief MapCoverage::generateWaypoints generates the waypoint/map coverage strategy
 */
void MapCoverage::generateWaypoints()
{
  generateWaypoints(left_bottom_corner_, right_bottom_corner_);
}

/**
 * @brief MapCoverage::generateWaypoints generates the waypoint/map coverage strategy
 */
void MapCoverage::generateWaypoints(geometry_msgs::Point start_position, geometry_msgs::Point end_position)
{
  clear();
  geometry_msgs::Point last_waypoint(end_position);
  geometry_msgs::Point current_waypoint(start_position);
  while (current_waypoint.x <= last_waypoint.x)
  {
    waypoints_.push_back(current_waypoint);
    current_waypoint.y = current_waypoint.y == left_bottom_corner_.y
                             ? left_top_corner_.y
                             : left_bottom_corner_.y;
    waypoints_.push_back(current_waypoint);
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
    waypoints_.pop_front();
  }
}

/**
 * @brief MapCoverage::addMine
 */
void MapCoverage::addMine(geometry_msgs::Point position)
{
  mines_.push_back(position);
}

/**
 * @brief MapCoverage::isInsideMineArea verifies if a waypoint is inside a mine area
 */
bool MapCoverage::isInsideMineArea(geometry_msgs::Point waypoint) const
{
  for (std::list<geometry_msgs::Point>::const_iterator mine_point = mines_.begin(); mine_point != mines_.end(); ++mine_point)
  {
    double distance = sqrt(pow((waypoint.x - mine_point->x), 2) + pow((waypoint.y - mine_point->y), 2));
    if (distance <= landmine_radius_area_)
    {
      return true;
    }
  }

  return false;
}
}
