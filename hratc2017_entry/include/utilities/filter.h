/**
 *  This header file defines and implements the Filter helper class.
 *
 *  Version: 1.1.5
 *  Created on: 22/03/2017
 *  Modified on: 26/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_FILTER_H_
#define _UTILITIES_FILTER_H_

#include <list>
#include <sstream>

namespace utilities
{
template <typename T> class Filter
{
public:
  Filter(unsigned int number_of_samples);
  virtual ~Filter();
  bool empty() const;
  unsigned int size() const;
  virtual T getFilteredValue() const;
  typename std::list<T>::const_iterator begin() const;
  typename std::list<T>::const_iterator end() const;
  virtual void add(T sample);
  void setNumberOfSamples(unsigned int number_of_samples);
  void clear();
  std::string str() const;
  const char* c_str() const;

protected:
  virtual void setFilteredValue(T filtered_value);

private:
  T filtered_value_;
  std::list<T> samples_;
  unsigned int number_of_samples_;
};

/**
 * @brief Filter::Filter
 * @param number_of_samples
 */
template <typename T>
Filter<T>::Filter(unsigned int number_of_samples)
    : number_of_samples_(number_of_samples)
{
}

/**
 * @brief Filter::~Filter
 */
template <typename T> Filter<T>::~Filter() {}

/**
 * @brief Filter::empty
 * @return
 */
template <typename T> bool Filter<T>::empty() const { return samples_.empty(); }

/**
 * @brief Filter::size
 * @return
 */
template <typename T> unsigned int Filter<T>::size() const
{
  return samples_.size();
}

/**
 * @brief Filter::getFilteredValue
 * @return
 */
template <typename T> T Filter<T>::getFilteredValue() const
{
  return filtered_value_;
}

/**
 * @brief Filter::begin
 * @return
 */
template <typename T>
typename std::list<T>::const_iterator Filter<T>::begin() const
{
  return samples_.begin();
}

/**
 * @brief Filter::end
 * @return
 */
template <typename T>
typename std::list<T>::const_iterator Filter<T>::end() const
{
  return samples_.end();
}

/**
 * @brief Filter::add
 * @param sample
 */
template <typename T> void Filter<T>::add(T sample)
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
template <typename T>
void Filter<T>::setNumberOfSamples(unsigned int number_of_samples)
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
template <typename T> void Filter<T>::clear() { return samples_.clear(); }

/**
 * @brief Filter::str
 * @return
 */
template <typename T> std::string Filter<T>::str() const
{
  std::stringstream ss;
  typename std::list<T>::const_iterator it(samples_.begin());
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
template <typename T> const char* Filter<T>::c_str() const
{
  return str().c_str();
}

/**
 * @brief Filter::setFilteredValue
 * @param filtered_value
 */
template <typename T> void Filter<T>::setFilteredValue(T filtered_value)
{
  filtered_value_ = filtered_value;
}
}

#endif // _UTILITIES_FILTER_H_
