/**
 *  This header file defines the PointMeanFilter helper class, which
 *is based on MeanFilter helper class.
 *
 *  Version: 1.1.5
 *  Created on: 27/03/2017
 *  Modified on: 27/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_POINT_MEAN_FILTER_H_
#define _UTILITIES_POINT_MEAN_FILTER_H_

#include <geometry_msgs/Point.h>
#include "utilities/mean_filter.h"

namespace utilities
{
class PointMeanFilter : public MeanFilter<geometry_msgs::Point>
{
public:
  PointMeanFilter(unsigned int number_of_samples);
  virtual ~PointMeanFilter();
  virtual geometry_msgs::Point getMeanValue() const;
};
}

#endif // _UTILITIES_POINT_MEAN_FILTER_H_
