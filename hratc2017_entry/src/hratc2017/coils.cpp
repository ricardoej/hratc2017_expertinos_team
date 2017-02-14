/**
 *  This source file implements the Coils class. This class encapsulates helpers
 *methods that evaluates metal detector readings.
 *
 *  Version: 0.0.1
 *  Created on: 30/01/2017
 *  Modified on: 14/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/coils.h"

namespace hratc2017
{

/**
 * @brief Coils::Coils
 * @param left
 * @param right
 */
Coils::Coils(double threshold, int number_of_observations, float left,
             float right)
    : left_(left), right_(right), threshold_(threshold),
      number_of_observations_(number_of_observations)
{
}

/**
 * @brief Coils::Coils
 * @param msg
 */
Coils::Coils(const metal_detector_msgs::Coil::ConstPtr& msg, double threshold,
             int number_of_observations)
    : left_(msg->left_coil), right_(msg->right_coil), threshold_(threshold),
      number_of_observations_(number_of_observations)
{
}

/**
 * @brief Coils::Coils
 * @param msg
 */
Coils::Coils(const metal_detector_msgs::Coil& msg, double threshold,
             int number_of_observations)
    : left_(msg.left_coil), right_(msg.right_coil), threshold_(threshold),
      number_of_observations_(number_of_observations)
{
}

/**
 * @brief Coils::~Coils
 */
Coils::~Coils() {}

/**
 * @brief Coils::getLeft
 * @return
 */
float Coils::getLeft() const { return left_; }

/**
 * @brief Coils::getRight
 * @return
 */
float Coils::getRight() const { return right_; }

/**
 * @brief Coils::setThreshold
 * @param threshold
 */
void Coils::setThreshold(double threshold)
{
  threshold_ =
      threshold < 0 || threshold > 1.0 ? COIL_SIGNAL_THRESHOLD : threshold;
}

/**
 * @brief Coils::setNumberOfObservations
 * @param number_of_observations
 */
void Coils::setNumberOfObservations(int number_of_observations)
{
  number_of_observations_ = number_of_observations > 0
                                ? number_of_observations
                                : COIL_SIGNAL_FILTER_NUMBER_OF_OBSERVATIONS;
}

/**
 * @brief Coils::gotLandmineOnLeft
 * @return
 */
bool Coils::isHighCoilSignalOnLeft() const { return left_ >= threshold_; }

/**
 * @brief Coils::gotLandmineOnRight
 * @return
 */
bool Coils::isHighCoilSignalOnRight() const { return right_ >= threshold_; }

/**
 * @brief Coils::to_msg
 * @return
 */
metal_detector_msgs::Coil Coils::to_msg() const
{
  metal_detector_msgs::Coil msg;
  msg.header.stamp = ros::Time::now();
  msg.left_coil = left_;
  msg.right_coil = right_;
  return msg;
}

/**
 * @brief Coils::str
 * @return
 */
std::string Coils::str() const
{
  std::stringstream ss;
  ss << "coil: (" << left_ << "," << right_ << ")";
  return ss.str();
}

/**
 * @brief Coils::c_str
 * @return
 */
const char* Coils::c_str() const { return str().c_str(); }

/**
 * @brief Coils::operator =
 * @param msg
 */
void Coils::operator=(const metal_detector_msgs::Coil::ConstPtr& msg)
{
  left_ = msg->left_coil;
  right_ = msg->right_coil;
}

/**
 * @brief Coils::operator =
 * @param msg
 */
void Coils::operator=(const metal_detector_msgs::Coil& msg)
{
  left_ = msg.left_coil;
  right_ = msg.right_coil;
}

/**
 * @brief LandmineAnalyzer::coilsCallback receives the metal detector coils
 * signal data whenever a new one is available.
 * @param msg new coils signal data.
 */
void Coils::coilsCallback(const metal_detector_msgs::Coil::ConstPtr& msg)
{
  int counter(0);
  float filtered_signal(0.0);
  left_samples_.push_back(msg->left_coil);
  for (int i(left_samples_.size() - 1); i >= 0; i--)
  {
    filtered_signal += left_samples_[i];
    if (counter++ == number_of_observations_)
    {
      break;
    }
  }
  left_ = filtered_signal / counter;
  filtered_signal = 0;
  counter = 0;
  right_samples_.push_back(msg->right_coil);
  for (int i(right_samples_.size() - 1); i >= 0; i--)
  {
    filtered_signal += right_samples_[i];
    if (counter++ == number_of_observations_)
    {
      break;
    }
  }
  right_ = filtered_signal / counter;
}
}
