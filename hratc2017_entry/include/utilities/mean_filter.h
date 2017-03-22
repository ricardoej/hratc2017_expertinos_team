/**
 *  This header file implements the MeanFilter helper class, which is based on
 *Filter helper class.
 *
 *  Version: 1.1.4
 *  Created on: 22/03/2017
 *  Modified on: 22/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_MEAN_FILTER_H_
#define _UTILITIES_MEAN_FILTER_H_

#include "utilities/filter.h"

namespace utilities
{
class MeanFilter : public Filter
{
public:
  MeanFilter(unsigned int number_of_samples);
  virtual ~MeanFilter();
  double getFilteredValue() const;
  virtual void add(double sample);

private:
  double filtered_value_;
};
}

#endif // _UTILITIES_MEAN_FILTER_H_
