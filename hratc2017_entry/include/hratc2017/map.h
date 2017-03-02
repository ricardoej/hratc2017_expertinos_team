/**
 *  This header file defines the Map class.
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

#include <queue>
#include <sstream>
#include <visualization_msgs/MarkerArray.h>
#include "utilities/exception.h"

#define DEFAULT_MAP_COVERAGE_OFFSET 1.5
#define DEFAULT_MAP_COVERAGE_MARGIN 2.0

namespace hratc2017
{
class Map
{
public:
  Map(visualization_msgs::MarkerArray::ConstPtr msg,
      std::string type = "absolute",
      double map_coverage_offset = DEFAULT_MAP_COVERAGE_OFFSET,
      double map_coverage_margin = DEFAULT_MAP_COVERAGE_MARGIN);
  Map(geometry_msgs::Point left_bottom_corner,
      geometry_msgs::Point left_top_corner,
      geometry_msgs::Point right_top_corner,
      geometry_msgs::Point right_bottom_corner,
      double map_coverage_offset = DEFAULT_MAP_COVERAGE_OFFSET,
      double map_coverage_margin = DEFAULT_MAP_COVERAGE_MARGIN);
  virtual ~Map();
  geometry_msgs::Point getNextWaypoint() const;
  double getX() const;
  double getY() const;
  bool empty() const;
  void pop();
  virtual std::string str() const;
  const char* c_str() const;

private:
  double map_coverage_offset_;
  double map_coverage_margin_;
  geometry_msgs::Point left_top_corner_;
  geometry_msgs::Point right_top_corner_;
  geometry_msgs::Point left_bottom_corner_;
  geometry_msgs::Point right_bottom_corner_;
  std::queue<geometry_msgs::Point> waypoints_;
  void generateWaypoints();
  void clear();
};
}

#endif /* _HRATC2017_ENTRIES_MAP_H_ */
