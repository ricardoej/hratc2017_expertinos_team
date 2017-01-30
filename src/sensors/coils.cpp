/**
 *  This source file implements the Coils class. This class encapsulates helpers
 *methods that evaluates metal detector readings.
 *
 *  Version: 0.0.1
 *  Created on: 30/01/2017
 *  Modified on: 30/01/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "sensors/coils.h"

namespace sensors
{

const float Coils::THRESHOLD = 1.0f;

/**
 * @brief Coils::Coils
 * @param left
 * @param right
 */
Coils::Coils(float left, float right) : left_(left), right_(right) {}

/**
 * @brief Coils::Coils
 * @param msg
 */
Coils::Coils(const metal_detector_msgs::Coil::ConstPtr& msg)
    : left_(msg->left_coil), right_(msg->right_coil)
{
}

/**
 * @brief Coils::Coils
 * @param msg
 */
Coils::Coils(const metal_detector_msgs::Coil& msg)
    : left_(msg.left_coil), right_(msg.right_coil)
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
 * @brief Coils::gotLandmineOnLeft
 * @return
 */
bool Coils::gotLandmineOnLeft() const
{
  ///not implemented yet!!!
  return left_ >= THRESHOLD;
}

/**
 * @brief Coils::gotLandmineOnRight
 * @return
 */
bool Coils::gotLandmineOnRight() const
{
  ///not implemented yet!!!
  return right_ >= THRESHOLD;
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
}
