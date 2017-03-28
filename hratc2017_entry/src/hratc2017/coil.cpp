/**
 *  This source file implements the Coil class. This class encapsulates helpers
 *methods that evaluates metal detector readings.
 *
 *  Version: 1.1.4
 *  Created on: 16/02/2017
 *  Modified on: 22/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/coil.h"

namespace hratc2017
{

/**
 * @brief Coil::Coil
 * @param frame_id
 * @param sample_time
 * @param low_threshold
 * @param high_threshold
 * @param number_of_observations
 */
Coil::Coil(std::string frame_id, float sample_time, float low_threshold,
           float high_threshold, unsigned int number_of_observations,
           unsigned int number_of_derivatives)
    : frame_id_(frame_id),
      value_filter_(new utilities::DoubleMeanFilter(number_of_observations)),
      derived_value_filter_(new utilities::DoubleMeanFilter(number_of_derivatives)),
      sample_time_(sample_time), low_threshold_(low_threshold),
      high_threshold_(high_threshold)
{
  pose_.header.frame_id = "UNDEF";
  pose_.header.stamp = ros::Time::now();
  pose_.pose.position.x = 0;
  pose_.pose.position.y = 0;
  pose_.pose.position.z = 0;
  pose_.pose.orientation.x = 0;
  pose_.pose.orientation.y = 0;
  pose_.pose.orientation.z = 0;
  pose_.pose.orientation.w = 1;
}

/**
 * @brief Coil::~Coil
 */
Coil::~Coil()
{
  if (value_filter_)
  {
    delete value_filter_;
    value_filter_ = NULL;
  }
  if (derived_value_filter_)
  {
    delete derived_value_filter_;
    derived_value_filter_ = NULL;
  }
}

/**
 * @brief Coil::getPose
 * @return
 */
geometry_msgs::PoseStamped Coil::getPose() const { return pose_; }

/**
 * @brief Coil::getFrameId
 * @return
 */
std::string Coil::getFrameId() const { return frame_id_; }

/**
 * @brief Coil::getValue
 * @return
 */
float Coil::getValue() const { return value_filter_->getFilteredValue(); }

/**
 * @brief Coil::getDerivedValue
 * @return
 */
float Coil::getDerivedValue() const { return derived_value_filter_->getFilteredValue(); }

/**
 * @brief Coil::getSampleTime
 * @return
 */
float Coil::getSampleTime() const { return sample_time_; }

/**
 * @brief Coil::setTransform
 * @param tf
 */
void Coil::setTransform(tf::Transform tf) { coil_tf_ = tf; }

/**
 * @brief Coil::setPose
 * @param pose
 */
void Coil::setPose(geometry_msgs::PoseStamped pose)
{
  if (pose_.header.stamp < pose.header.stamp)
  {
    pose_ = pose;
  }
}

/**
 * @brief Coil::setPose
 * @param robot_tf
 */
void Coil::setPose(tf::StampedTransform robot_tf)
{
  if (pose_.header.stamp >= robot_tf.stamp_)
  {
    return;
  }
  tf::Transform result_tf;
  result_tf.mult(robot_tf, coil_tf_);
  pose_.header.frame_id = frame_id_;
  pose_.header.stamp = ros::Time::now();
  pose_.pose.position.x = result_tf.getOrigin().x();
  pose_.pose.position.y = result_tf.getOrigin().y();
  pose_.pose.position.z = result_tf.getOrigin().z();
  pose_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
}

/**
 * @brief Coil::setMaxPoseUpdateInterval
 * @param max_pose_update_interval
 */
void Coil::setMaxPoseUpdateInterval(double max_pose_update_interval)
{
  max_pose_update_interval_ = max_pose_update_interval <= 0.0
                                  ? DEFAULT_MAX_POSE_UPDATE_INTERVAL
                                  : max_pose_update_interval;
}

/**
 * @brief Coil::setSampleTime
 * @param sample_time
 */
void Coil::setSampleTime(double sample_time)
{
  sample_time_ =
      sample_time <= 0.0 ? DEFAULT_DERIVATIVE_SAMPLE_TIME : sample_time;
}

/**
 * @brief Coil::setLowThreshold
 * @param low_threshold
 */
void Coil::setLowThreshold(double low_threshold)
{
  low_threshold_ = low_threshold > high_threshold_ || low_threshold < 0.0 ||
                           low_threshold > 1.0
                       ? DEFAULT_LOW_COIL_SIGNAL_THRESHOLD
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
                        ? DEFAULT_HIGH_COIL_SIGNAL_THRESHOLD
                        : high_threshold;
}

/**
 * @brief Coil::setNumberOfObservations
 * @param number_of_observations
 */
void Coil::setNumberOfObservations(int number_of_observations)
{
  number_of_observations =
      number_of_observations > 0
          ? number_of_observations
          : DEFAULT_COIL_SIGNAL_FILTER_NUMBER_OF_OBSERVATIONS;
  value_filter_->setNumberOfSamples(number_of_observations);
}

/**
 * @brief Coil::setNumberOfDerivatives
 * @param number_of_derivatives
 */
void Coil::setNumberOfDerivatives(int number_of_derivatives)
{
  number_of_derivatives = number_of_derivatives > 0
                               ? number_of_derivatives
                               : DEFAULT_NUMBER_OF_DERIVATIVES;
  derived_value_filter_->setNumberOfSamples(number_of_derivatives);
}

/**
 * @brief Coil::isLowSignal
 * @return
 */
bool Coil::isLow() const { return value_filter_->getFilteredValue() <= low_threshold_; }

/**
 * @brief Coil::isHighSignal
 * @return
 */
bool Coil::isHigh() const { return value_filter_->getFilteredValue() >= high_threshold_; }

/**
 * @brief Coil::calculateDerivative
 */
void Coil::calculateDerivative()
{
  derived_value_filter_->add((value_filter_->getFilteredValue() - last_value_) / sample_time_);
}

/**
 * @brief Coil::str
 * @return
 */
std::string Coil::str() const
{
  std::stringstream ss;
  ss << frame_id_ << ": " << value_filter_->getFilteredValue();
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
  last_value_ = value_filter_->getFilteredValue();
  value_filter_->add(value);
}
}
