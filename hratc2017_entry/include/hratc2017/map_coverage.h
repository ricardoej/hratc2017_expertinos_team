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

#define DEFAULT_MAP_COVERAGE_OFFSET 0.1
#define DEFAULT_MAP_COVERAGE_MARGIN 0.9
#define DEFAULT_LANDMINE_RADIUS_AREA 1.0

namespace hratc2017
{
class MapCoverage
{
public:
  MapCoverage(visualization_msgs::MarkerArray::ConstPtr msg,
              double map_coverage_offset = DEFAULT_MAP_COVERAGE_OFFSET,
              double map_coverage_margin = DEFAULT_MAP_COVERAGE_MARGIN,
              double landmine_radius_area = DEFAULT_LANDMINE_RADIUS_AREA,
              std::list<geometry_msgs::Point> waypoints =
                  std::list<geometry_msgs::Point>());
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
  std::list<geometry_msgs::Point> corners_;
  std::list<geometry_msgs::Point> waypoints_;
  std::list<geometry_msgs::Point> mines_;
  std::string waypoints_str() const;
  void generateWaypoints();
  void clear();
  bool isInsideMineArea(geometry_msgs::Point waypoint,
                        geometry_msgs::Point& landmine) const;
  bool isInsideMapArea(geometry_msgs::Point waypoint) const;
};
}

#endif /* _HRATC2017_ENTRIES_MAP_H_ */
