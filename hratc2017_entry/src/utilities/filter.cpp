/**
 *  This source file implements the Filter helper class.
 *
 *  Version: 1.1.4
 *  Created on: 22/03/2017
 *  Modified on: 22/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/filter.h"

namespace utilities
{

/**
 * @brief Filter::Filter
 * @param number_of_samples
 */
Filter::Filter(unsigned int number_of_samples)
    : number_of_samples_(number_of_samples)
{
}

/**
 * @brief Filter::~Filter
 */
Filter::~Filter() {}

/**
 * @brief Filter::empty
 * @return
 */
bool Filter::empty() const { return samples_.empty(); }

/**
 * @brief Filter::size
 * @return
 */
unsigned int Filter::size() const { return samples_.size(); }

/**
 * @brief Filter::begin
 * @return
 */
std::list<double>::const_iterator Filter::begin() const
{
  return samples_.begin();
}

/**
 * @brief Filter::end
 * @return
 */
std::list<double>::const_iterator Filter::end() const
{
  return samples_.end();
}

/**
 * @brief Filter::add
 * @param sample
 */
void Filter::add(double sample)
{
  if (samples_.size() >= number_of_samples_)
  {
    samples_.pop_back();
  }
  samples_.push_front(sample);
}

/**
 * @brief Filter::setNumberOfSamples
 * @param number_of_samples
 */
void Filter::setNumberOfSamples(unsigned int number_of_samples)
{
  while (samples_.size() > number_of_samples)
  {
    samples_.pop_back();
  }
  number_of_samples_ = number_of_samples;
}

/**
 * @brief Filter::clear
 */
void Filter::clear() { return samples_.clear(); }

/**
 * @brief Filter::str
 * @return
 */
std::string Filter::str() const
{
  std::stringstream ss;
  std::list<double>::const_iterator it(samples_.begin());
  while (it != samples_.end())
  {
    ss << *it << " ";
    it++;
  }
  return ss.str();
}

/**
 * @brief Filter::c_str
 * @return
 */
const char* Filter::c_str() const { return str().c_str(); }
}
