/**
 *  This source file implements the Coils class. This class encapsulates helpers
 *methods that evaluates metal detector readings.
 *
 *  Version: 0.0.1
 *  Created on: 30/01/2017
 *  Modified on: 01/02/2017
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
Coils::Coils(double threshold, float left, float right) : left_(left), right_(right), threshold_(threshold) {}

/**
 * @brief Coils::Coils
 * @param msg
 */
Coils::Coils(const metal_detector_msgs::Coil::ConstPtr& msg, double threshold)
    : left_(msg->left_coil), right_(msg->right_coil), threshold_(threshold)
{
}

/**
 * @brief Coils::Coils
 * @param msg
 */
Coils::Coils(const metal_detector_msgs::Coil& msg, double threshold)
    : left_(msg.left_coil), right_(msg.right_coil), threshold_(threshold)
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
  threshold_ = threshold < 0 || threshold > 1.0 ? COIL_SIGNAL_THRESHOLD : threshold;
}

/**
 * @brief Coils::gotLandmineOnLeft
 * @return
 */
bool Coils::isHighCoilSignalOnLeft() const
{
  return left_ >= threshold_;
}

/**
 * @brief Coils::gotLandmineOnRight
 * @return
 */
bool Coils::isHighCoilSignalOnRight() const
{
  return right_ >= threshold_;
}

/**
 * @brief Coils::to_msg
 * @return
 */
metal_detector_msgs::Coil Coils::to_msg() const
{
  metal_detector_msgs::Coil msg;
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
void Coils::coilsCallback(
    const metal_detector_msgs::Coil::ConstPtr& msg)
{
  left_samples_.push_back(msg->left_coil);
  for (int i(left_samples_.size()); i >= 0; i++)
  {
    left_ += left_samples_[i];
  }
  left_ /= NUMBER_OF_OBSERVATIONS;
  right_samples_.push_back(msg->right_coil);
  for (int i(right_samples_.size()); i >= 0; i++)
  {
    right_ += right_samples_[i];
  }
  right_ /= NUMBER_OF_OBSERVATIONS;
}
}
