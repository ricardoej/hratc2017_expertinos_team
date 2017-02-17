/**
 *  This source file implements the Coil class. This class encapsulates helpers
 *methods that evaluates metal detector readings.
 *
 *  Version: 0.1.0
 *  Created on: 16/02/2017
 *  Modified on: 16/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/coil.h"
#include <ros/ros.h>

namespace hratc2017
{

/**
 * @brief Coil::Coil
 * @param low_threshold
 * @param high_threshold
 * @param number_of_observations
 */
Coil::Coil(std::string frame_id, float low_threshold, float high_threshold,
           int number_of_observations)
    : frame_id_(frame_id), low_threshold_(low_threshold),
      high_threshold_(high_threshold),
      number_of_observations_(number_of_observations)
{
}

/**
 * @brief Coil::~Coil
 */
Coil::~Coil() {}

/**
 * @brief Coil::getFrameId
 * @return
 */
std::string Coil::getFrameId() const
{
  return frame_id_;
}

/**
 * @brief Coil::getValue
 * @return
 */
float Coil::getValue() const { return value_; }

/**
 * @brief Coil::setLowThreshold
 * @param low_threshold
 */
void Coil::setLowThreshold(double low_threshold)
{
  low_threshold_ = low_threshold > high_threshold_ || low_threshold < 0.0 ||
                           low_threshold > 1.0
                       ? LOW_COIL_SIGNAL_THRESHOLD
                       : low_threshold;
}

/**
 * @brief Coil::setHighThreshold
 * @param high_threshold
 */
void Coil::setHighThreshold(double high_threshold)
{
  high_threshold_ = high_threshold < low_threshold_ || high_threshold < 0.0 ||
                            high_threshold > 1.0
                        ? HIGH_COIL_SIGNAL_THRESHOLD
                        : high_threshold;
}

/**
 * @brief Coil::setNumberOfObservations
 * @param number_of_observations
 */
void Coil::setNumberOfObservations(int number_of_observations)
{
  number_of_observations_ = number_of_observations > 0
                                ? number_of_observations
                                : COIL_SIGNAL_FILTER_NUMBER_OF_OBSERVATIONS;
}

/**
 * @brief Coil::isLowSignal
 * @return
 */
bool Coil::isLow() const { return value_ <= low_threshold_; }

/**
 * @brief Coil::isHighSignal
 * @return
 */
bool Coil::isHigh() const { return value_ >= high_threshold_; }

/**
 * @brief Coil::str
 * @return
 */
std::string Coil::str() const
{
  std::stringstream ss;
  ss << frame_id_ << ": " << value_;
  return ss.str();
}

/**
 * @brief Coil::c_str
 * @return
 */
const char* Coil::c_str() const { return str().c_str(); }

/**
 * @brief Coil::operator =
 * @param value
 */
void Coil::operator=(float value)
{
  if (samples_.size() >= number_of_observations_)
  {
    samples_.pop_back();
  }
  float sum(value);
  std::list<float>::iterator it(samples_.begin());
  while (it != samples_.end())
  {
    sum += *it;
    it++;
  }
  samples_.push_front(value);
  value_ = sum / samples_.size();
}
}
