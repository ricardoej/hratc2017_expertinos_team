/**
 *  This header file defines and implements the MeanFilter helper class, which
 *is based on Filter helper class.
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
template <typename T> class MeanFilter : public Filter<T>
{
public:
  MeanFilter(unsigned int number_of_samples);
  virtual ~MeanFilter();
  virtual void add(T sample);

private:
  virtual T getMeanValue() const = 0;
};

/**
 * @brief MeanFilter::MeanFilter
 * @param number_of_samples
 */
template <typename T>
MeanFilter<T>::MeanFilter(unsigned int number_of_samples)
    : Filter<T>::Filter(number_of_samples)
{
}

/**
 * @brief MeanFilter::~MeanFilter
 */
template <typename T> MeanFilter<T>::~MeanFilter() {}

/**
 * @brief MeanFilter::add
 * @param sample
 */
template <typename T> void MeanFilter<T>::add(T sample)
{
  Filter<T>::add(sample);
  Filter<T>::setFilteredValue(getMeanValue());
}
}

#endif // _UTILITIES_MEAN_FILTER_H_
