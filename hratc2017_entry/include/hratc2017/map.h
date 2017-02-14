/**
 *  This header file defines the Map class.
 *
 *  Version: 0.0.1
 *  Created on: 06/02/2017
 *  Modified on: 06/02/2017
 *  Author: Ricardo Emerson Julio (ricardoej@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_ENTRIES_MAP_H_
#define _HRATC2017_ENTRIES_MAP_H_

#include <ros/ros.h>
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"

namespace hratc2017
{
  class  Map
  {
    public:
      Map();
      Map(visualization_msgs::MarkerArray::ConstPtr msg);
      Map(geometry_msgs::Point leftBottomCorner, geometry_msgs::Point leftTopCorner, geometry_msgs::Point rightTopCorner, geometry_msgs::Point rightBottomCorner);
      virtual ~Map();
      geometry_msgs::Point getLeftTopCorner() const;
      geometry_msgs::Point getRightTopCorner() const;
      geometry_msgs::Point getLeftBottomCorner() const;
      geometry_msgs::Point getRightBottomCorner() const;

    private:
      geometry_msgs::Point leftTopCorner_;
      geometry_msgs::Point rightTopCorner_;
      geometry_msgs::Point leftBottomCorner_;
      geometry_msgs::Point rightBottomCorner_;
  };
}

#endif /* _HRATC2017_ENTRIES_MAP_H_ */
