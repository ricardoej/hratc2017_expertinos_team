/**
 *  This header file implements the Filter helper class.
 *
 *  Version: 1.1.4
 *  Created on: 22/03/2017
 *  Modified on: 22/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_FILTER_H_
#define _UTILITIES_FILTER_H_

#include <list>
#include <sstream>

namespace utilities
{
class Filter
{
public:
  Filter(unsigned int number_of_samples);
  virtual ~Filter();
  bool empty() const;
  unsigned int size() const;
  std::list<double>::const_iterator begin() const;
  std::list<double>::const_iterator end() const;
  virtual void add(double sample);
  void setNumberOfSamples(unsigned int number_of_samples);
  void clear();
  std::string str() const;
  const char* c_str() const;

private:
  std::list<double> samples_;
  unsigned int number_of_samples_;
};
}

#endif // _UTILITIES_FILTER_H_
