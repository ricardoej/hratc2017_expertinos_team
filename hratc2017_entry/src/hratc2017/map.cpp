/**
 *  This header file defines the Map class. This class encapsulates helpers
 *  methods that evaluates metal detector readings.
 *
 *  Version: 0.0.1
 *  Created on: 06/02/2017
 *  Modified on: 06/02/2017
 *  Author: Ricardo Emerson Julio (ricardoej@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/map.h"
#define OFFSET 1.5

namespace hratc2017
{
  Map::Map() {}

  Map::Map(visualization_msgs::MarkerArray::ConstPtr msg, std::string type)
  {
    leftBottomCorner_ = msg->markers[0].pose.position;
    rightBottomCorner_ = msg->markers[1].pose.position;
    rightTopCorner_ = msg->markers[2].pose.position;
    leftTopCorner_ = msg->markers[3].pose.position;

    if (type == "relative")
    {
      double xSize = std::abs(leftTopCorner_.x - rightTopCorner_.x);
      double ySize = std::abs(leftBottomCorner_.x - rightBottomCorner_.x);

      leftBottomCorner_.x = -1 * xSize / 2 + OFFSET;
      leftBottomCorner_.y = -1 * ySize / 2 + OFFSET;
      leftBottomCorner_.z = 0;

      leftTopCorner_.x = -1 * xSize / 2 + OFFSET;
      leftTopCorner_.y = ySize / 2 - OFFSET;
      leftTopCorner_.z = 0;

      rightTopCorner_.x = xSize / 2 - OFFSET;
      rightTopCorner_.y = ySize / 2 - OFFSET;
      rightTopCorner_.z = 0;

      rightBottomCorner_.x = xSize / 2 - OFFSET;
      rightBottomCorner_.y = -1 * ySize / 2 + OFFSET;
      rightBottomCorner_.z = 0;
    }
  }

  Map::Map(geometry_msgs::Point leftBottomCorner, geometry_msgs::Point leftTopCorner, geometry_msgs::Point rightTopCorner, geometry_msgs::Point rightBottomCorner)
  {
    leftBottomCorner_ = leftBottomCorner;
    leftTopCorner_ = leftTopCorner;
    rightTopCorner_ = rightTopCorner;
    rightBottomCorner_ = rightBottomCorner;
  }

  Map::~Map() {}

  geometry_msgs::Point  Map::getLeftTopCorner() const
  {
    return leftTopCorner_;
  }

  geometry_msgs::Point  Map::getRightTopCorner() const
  {
    return rightTopCorner_;
  }

  geometry_msgs::Point  Map::getLeftBottomCorner() const
  {
    return leftBottomCorner_;
  }

  geometry_msgs::Point  Map::getRightBottomCorner() const
  {
    return rightBottomCorner_;
  }
}