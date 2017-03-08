/**
 *  This header file defines the MapCoverage class.
 *
 *  Version: 1.0.1
 *  Created on: 06/02/2017
 *  Modified on: 02/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *          Ricardo Emerson Julio (ricardoej@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_ENTRIES_MAP_H_
#define _HRATC2017_ENTRIES_MAP_H_

#include <list>
#include <sstream>
#include <visualization_msgs/MarkerArray.h>
#include "utilities/exception.h"

#define DEFAULT_MAP_COVERAGE_OFFSET 1.0
#define DEFAULT_MAP_COVERAGE_MARGIN 2.0
#define DEFAULT_LANDMINE_RADIUS_AREA 1.0
#define ABSOLUTE_MAP_COORDINATE_TYPE "absolute"
#define RELATIVE_MAP_COORDINATE_TYPE "relative"

namespace hratc2017
{
class MapCoverage
{
public:
  MapCoverage(visualization_msgs::MarkerArray::ConstPtr msg,
      std::string type = RELATIVE_MAP_COORDINATE_TYPE,
      double map_coverage_offset = DEFAULT_MAP_COVERAGE_OFFSET,
      double map_coverage_margin = DEFAULT_MAP_COVERAGE_MARGIN,
      double landmine_radius_area = DEFAULT_LANDMINE_RADIUS_AREA);
  MapCoverage(geometry_msgs::Point left_bottom_corner,
      geometry_msgs::Point left_top_corner,
      geometry_msgs::Point right_top_corner,
      geometry_msgs::Point right_bottom_corner,
      double map_coverage_offset = DEFAULT_MAP_COVERAGE_OFFSET,
      double map_coverage_margin = DEFAULT_MAP_COVERAGE_MARGIN,
      double landmine_radius_area = DEFAULT_LANDMINE_RADIUS_AREA);
  virtual ~MapCoverage();
  geometry_msgs::Point getNextWaypoint() const;
  double getX() const;
  double getY() const;
  bool empty() const;
  void pop();
  void addMine(geometry_msgs::Point position);
  virtual std::string str() const;
  const char* c_str() const;

private:
  double map_coverage_offset_;
  double map_coverage_margin_;
  double landmine_radius_area_;
  geometry_msgs::Point left_top_corner_;
  geometry_msgs::Point right_top_corner_;
  geometry_msgs::Point left_bottom_corner_;
  geometry_msgs::Point right_bottom_corner_;
  std::list<geometry_msgs::Point> waypoints_;
  std::list<geometry_msgs::Point> mines_;
  std::string waypoints_str() const;
  void generateWaypoints();
  void generateWaypoints(geometry_msgs::Point start_position, geometry_msgs::Point end_position, std::string type = "horizontal");
  void clear();
  bool isInsideMineArea(geometry_msgs::Point waypoint, geometry_msgs::Point &landmine) const;
};
}

#endif /* _HRATC2017_ENTRIES_MAP_H_ */
