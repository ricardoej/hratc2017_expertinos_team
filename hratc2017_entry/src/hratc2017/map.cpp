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

namespace hratc2017
{
  Map::Map(visualization_msgs::MarkerArray::ConstPtr& msg)
  {

    for (int i = 0; i < 3; i++)
    {
      for (int j = i; j < 4; j++)
      {
        if (msg->markers[i].pose.position.x < msg->markers[j].pose.position.x
          && msg->markers[i].pose.position.y < msg->markers[j].pose.position.y)
        {
          leftBottomCorner_ = msg->markers[i].pose.position;
        }
        else if (msg->markers[i].pose.position.x < msg->markers[j].pose.position.x
          && msg->markers[i].pose.position.y > msg->markers[j].pose.position.y)
        {
          leftTopCorner_ = msg->markers[i].pose.position;
        }
        else if (msg->markers[i].pose.position.x > msg->markers[j].pose.position.x
          && msg->markers[i].pose.position.y < msg->markers[j].pose.position.y)
        {
          rightBottomCorner_ = msg->markers[i].pose.position;
        }
        else
        {
          rightTopCorner_ = msg->markers[i].pose.position; 
        }
      }
    }
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