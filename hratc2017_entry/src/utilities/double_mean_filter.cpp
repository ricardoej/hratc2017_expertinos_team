/**
 *  This header file implements the DoubleMeanFilter helper class, which
 *is based on MeanFilter helper class.
 *
 *  Version: 1.1.5
 *  Created on: 27/03/2017
 *  Modified on: 27/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/double_mean_filter.h"

namespace utilities
{

/**
 * @brief DoubleMeanFilter::DoubleMeanFilter
 * @param number_of_samples
 */
DoubleMeanFilter::DoubleMeanFilter(unsigned int number_of_samples)
    : MeanFilter<double>::MeanFilter(number_of_samples)
{
}

/**
 * @brief DoubleMeanFilter::~DoubleMeanFilter
 */
DoubleMeanFilter::~DoubleMeanFilter() {}

/**
 * @brief DoubleMeanFilter::getMeanValue
 */
double DoubleMeanFilter::getMeanValue() const
{
  double sum(0.0);
  std::list<double>::const_iterator it(Filter<double>::begin());
  while (it != Filter<double>::end())
  {
    sum += *it;
    it++;
  }
  return sum / Filter<double>::size();
}
}
