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
#include <ros/ros.h>

namespace hratc2017
{

/**
 * @brief MapCoverage::MapCoverage
 * @param msg
 * @param type
 */
MapCoverage::MapCoverage(visualization_msgs::MarkerArray::ConstPtr msg,
  double map_coverage_offset, double map_coverage_margin, double landmine_radius_area)
    : map_coverage_offset_(map_coverage_offset),
      map_coverage_margin_(map_coverage_margin),
      landmine_radius_area_(landmine_radius_area)
{
  if (msg->markers.size() < 3)
  {
    throw utilities::Exception("Map must have at least three corners!!!");
  }

  for (int i = 0; i < msg->markers.size(); ++i)
  {
    corners_.push_front(msg->markers[i].pose.position);
  }

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
  geometry_msgs::Point landmine;

  if (isInsideMineArea(next_point, landmine))
  {
      if (next_point.y > 0)
      {
        next_point.y = landmine.y - landmine_radius_area_;
      }
      else
      {
        next_point.y = landmine.y + landmine_radius_area_; 
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
  ss << "Map\n";
  for (std::list<geometry_msgs::Point>::const_iterator ci = corners_.begin(); ci != corners_.end(); ++ci)
  {
    ss << "(" << ci->x << ", " << ci->y << "), ";
  }
  ss << "\nWaypoints:\n";
  ss << waypoints_str();
  return ss.str();
}

/**
 * @brief MapCoverage::waypoints_str
 * @return
 */
std::string MapCoverage::waypoints_str() const 
{
  std::ostringstream ss;
  for (std::list<geometry_msgs::Point>::const_iterator ci = waypoints_.begin(); ci != waypoints_.end(); ++ci)
  {
    ss << "(" << ci->x << ", " << ci->y << "), ";
  }
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

  double current_offset = map_coverage_margin_;

  while (current_offset > map_coverage_offset_)
  {
    for (std::list<geometry_msgs::Point>::const_iterator ci = corners_.begin(); ci != corners_.end(); ++ci)
    {
      geometry_msgs::Point point;
      point.x = ci->x * current_offset;
      point.y = ci->y * current_offset;
      waypoints_.push_back(point);
    }

    geometry_msgs::Point point;
    point.x = corners_.front().x * current_offset;
    point.y = corners_.front().y * current_offset;
    waypoints_.push_back(point);

    current_offset -= map_coverage_offset_;
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
bool MapCoverage::isInsideMineArea(geometry_msgs::Point waypoint, geometry_msgs::Point &landmine) const
{
  for (std::list<geometry_msgs::Point>::const_iterator mine_point = mines_.begin(); mine_point != mines_.end(); ++mine_point)
  {
    double distance = sqrt(pow((waypoint.x - mine_point->x), 2) + pow((waypoint.y - mine_point->y), 2));
    if (distance <= landmine_radius_area_)
    {
      landmine = *mine_point;
      return true;
    }
  }

  return false;
}
}
