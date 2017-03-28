/**
 *  This header file defines the DoubleMeanFilter helper class, which
 *is based on MeanFilter helper class.
 *
 *  Version: 1.1.5
 *  Created on: 27/03/2017
 *  Modified on: 27/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_DOUBLE_MEAN_FILTER_H_
#define _UTILITIES_DOUBLE_MEAN_FILTER_H_

#include "utilities/mean_filter.h"

namespace utilities
{
class DoubleMeanFilter : public MeanFilter<double>
{
public:
  DoubleMeanFilter(unsigned int number_of_samples);
  virtual ~DoubleMeanFilter();
  virtual double getMeanValue() const;
};
}

#endif // _UTILITIES_DOUBLE_MEAN_FILTER_H_
