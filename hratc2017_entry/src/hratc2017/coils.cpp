/**
 *  This source file implements the Coils class. This class encapsulates helpers
 *methods that evaluates metal detector readings.
 *
 *  Version: 1.0.1
 *  Created on: 30/01/2017
 *  Modified on: 20/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/coils.h"

namespace hratc2017
{

/**
 * @brief Coils::Coils
 */
Coils::Coils() : left_("left_coil"), right_("right_coil"), tf_(NULL)
{
  EMPTY_POSE.header.frame_id = "UNDEF";
  EMPTY_POSE.pose.position.x = 0;
  EMPTY_POSE.pose.position.y = 0;
  EMPTY_POSE.pose.position.z = 0;
  EMPTY_POSE.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
}

/**
 * @brief Coils::Coils
 * @param tf
 */
Coils::Coils(tf::TransformListener* tf)
    : left_("left_coil"), right_("right_coil"), tf_(tf)
{
  EMPTY_POSE.header.frame_id = "UNDEF";
  EMPTY_POSE.pose.position.x = 0;
  EMPTY_POSE.pose.position.y = 0;
  EMPTY_POSE.pose.position.z = 0;
  EMPTY_POSE.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
}

/**
 * @brief Coils::~Coils
 */
Coils::~Coils()
{
  if (tf_)
  {
    delete tf_;
    tf_ = NULL;
  }
}

/**
 * @brief Coils::getLeftValue
 * @return
 */
float Coils::getLeftValue() const { return left_.getValue(); }

/**
 * @brief Coils::getRightValue
 * @return
 */
float Coils::getRightValue() const { return right_.getValue(); }

/**
 * @brief Coils::setLowThreshold
 * @param low_threshold
 */
void Coils::setLowThreshold(double low_threshold)
{
  left_.setLowThreshold(low_threshold);
  right_.setLowThreshold(low_threshold);
}

/**
 * @brief Coils::setHighThreshold
 * @param high_threshold
 */
void Coils::setHighThreshold(double high_threshold)
{
  left_.setHighThreshold(high_threshold);
  right_.setHighThreshold(high_threshold);
}

/**
 * @brief Coils::setNumberOfObservations
 * @param number_of_observations
 */
void Coils::setNumberOfObservations(int number_of_observations)
{
  left_.setNumberOfObservations(number_of_observations);
  right_.setNumberOfObservations(number_of_observations);
}

/**
 * @brief Coils::isLeftLow
 * @return
 */
bool Coils::isLeftLow() const { return left_.isLow(); }

/**
 * @brief Coils::isLeftHigh
 * @return
 */
bool Coils::isLeftHigh() const { return left_.isHigh(); }

/**
 * @brief Coils::isRightLow
 * @return
 */
bool Coils::isRightLow() const { return right_.isLow(); }

/**
 * @brief Coils::isRightHigh
 * @return
 */
bool Coils::isRightHigh() const { return right_.isHigh(); }

/**
 * @brief Coils::isOneLow
 * @return
 */
bool Coils::isOneLow() const { return left_.isLow() != right_.isLow(); }

/**
 * @brief Coils::isOneHigh
 * @return
 */
bool Coils::isOneHigh() const { return left_.isHigh() != right_.isHigh(); }

/**
 * @brief Coils::isAnyLow
 * @return
 */
bool Coils::isAnyLow() const { return left_.isLow() || right_.isLow(); }

/**
 * @brief Coils::isAnyHigh
 * @return
 */
bool Coils::isAnyHigh() const { return left_.isHigh() || right_.isHigh(); }

/**
 * @brief Coils::isBothLow
 * @return
 */
bool Coils::isBothLow() const { return left_.isLow() && right_.isLow(); }

/**
 * @brief Coils::isBothHigh
 * @return
 */
bool Coils::isBothHigh() const { return left_.isHigh() && right_.isHigh(); }

/**
 * @brief Coils::isBothNotLow
 * @return
 */
bool Coils::isBothNotLow() const { return !left_.isLow() && !right_.isLow(); }

/**
 * @brief Coils::isBothNotHigh
 * @return
 */
bool Coils::isBothNotHigh() const
{
  return !left_.isHigh() && !right_.isHigh();
}

/**
 * @brief Coils::to_msg
 * @return
 */
metal_detector_msgs::Coil Coils::to_msg() const
{
  metal_detector_msgs::Coil msg;
  msg.header.stamp = ros::Time::now();
  msg.left_coil = left_.getValue();
  msg.right_coil = right_.getValue();
  return msg;
}

/**
 * @brief Coils::str
 * @return
 */
std::string Coils::str() const
{
  return "coils: (" + left_.str() + ", " + right_.str() + ")";
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
  left_ = msg->left_coil;
  right_ = msg->right_coil;
}

/**
 * @brief Coils::getLeftPose
 * @return
 */
geometry_msgs::PoseStamped Coils::getLeftPose() const
{
  return getPose(left_.getFrameId());
}

/**
 * @brief Coils::getRightPose
 * @return
 */
geometry_msgs::PoseStamped Coils::getRightPose() const
{
  return getPose(right_.getFrameId());
}

/**
 * @brief Coils::getPose
 * @param coil
 * @return
 */
geometry_msgs::PoseStamped Coils::getPose(std::string frame_id) const
{
  if (!tf_)
  {
    return EMPTY_POSE;
  }
  geometry_msgs::PoseStamped pose;
  tf::StampedTransform transform;
  ros::Time now(ros::Time::now());
  try
  {
    tf_->waitForTransform(MINEFIELD_FRAME_ID, frame_id, now,
                          ros::Duration(2.0));
    tf_->lookupTransform(MINEFIELD_FRAME_ID, frame_id, now, transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    // tem que tratar melhor essa excessao!!!
    return EMPTY_POSE;
  }
  pose.header.frame_id = frame_id;
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = transform.getOrigin().x();
  pose.pose.position.y = transform.getOrigin().y();
  pose.pose.position.z = transform.getOrigin().z();
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  return pose;
}
}
