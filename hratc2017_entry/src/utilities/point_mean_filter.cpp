/**
 *  This header file implements the PointMeanFilter helper class, which
 *is based on MeanFilter helper class.
 *
 *  Version: 1.1.5
 *  Created on: 27/03/2017
 *  Modified on: 27/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/point_mean_filter.h"

namespace utilities
{

/**
 * @brief PointMeanFilter::PointMeanFilter
 * @param number_of_samples
 */
PointMeanFilter::PointMeanFilter(unsigned int number_of_samples)
    : MeanFilter<geometry_msgs::Point>::MeanFilter(number_of_samples)
{
}

/**
 * @brief PointMeanFilter::~PointMeanFilter
 */
PointMeanFilter::~PointMeanFilter() {}

/**
 * @brief PointMeanFilter::getMeanValue
 * @return
 */
geometry_msgs::Point PointMeanFilter::getMeanValue() const
{
  geometry_msgs::Point mean;
  mean.x = 0.0;
  mean.y = 0.0;
  mean.z = 0.0;
  std::list<geometry_msgs::Point>::const_iterator it(
      Filter<geometry_msgs::Point>::begin());
  while (it != Filter<geometry_msgs::Point>::end())
  {
    mean.x += it->x;
    mean.y += it->y;
    mean.z += it->z;
    it++;
  }
  mean.x /= Filter<geometry_msgs::Point>::size();
  mean.y /= Filter<geometry_msgs::Point>::size();
  mean.z /= Filter<geometry_msgs::Point>::size();
  return mean;
}
}
