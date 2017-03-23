/**
 *  This source file implements the MeanFilter helper class, which is based on
 *Filter helper class.
 *
 *  Version: 1.1.4
 *  Created on: 22/03/2017
 *  Modified on: 22/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/mean_filter.h"

namespace utilities
{

/**
 * @brief MeanFilter::MeanFilter
 * @param number_of_samples
 */
MeanFilter::MeanFilter(unsigned int number_of_samples)
    : Filter(number_of_samples), filtered_value_(0.0)
{
}

/**
 * @brief MeanFilter::~MeanFilter
 */
MeanFilter::~MeanFilter() {}

/**
 * @brief MeanFilter::getFilteredValue
 * @return
 */
double MeanFilter::getFilteredValue() const { return filtered_value_; }

/**
 * @brief MeanFilter::add
 * @param sample
 */
void MeanFilter::add(double sample)
{
  Filter::add(sample);
  double sum(0.0);
  std::list<double>::const_iterator it(Filter::begin());
  while (it != Filter::end())
  {
    sum += *it;
    it++;
  }
  filtered_value_ = sum / Filter::size();
}
}
