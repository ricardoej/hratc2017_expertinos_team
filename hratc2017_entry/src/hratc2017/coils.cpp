/**
 *  This source file implements the Coils class. This class encapsulates helpers
 *methods that evaluates metal detector readings.
 *
 *  Version: 1.1.4
 *  Created on: 30/01/2017
 *  Modified on: 22/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/coils.h"

namespace hratc2017
{

/**
 * @brief Coils::Coils
 * @param tf
 */
Coils::Coils(ros::NodeHandle* nh)
    : left_("left_coil"), right_("right_coil"), left_updated_(false),
      right_updated_(false)
{
  last_sample_ = (getRightValue() + getLeftValue()) / 2;
  pose_update_ = nh->createTimer(ros::Duration(POSE_UPDATE_INTERVAL),
                                 &Coils::updatePose, this, false, false);
  coils_sub_ = nh->subscribe("/coils", 10, &Coils::coilsCallback, this);
}

/**
 * @brief Coils::~Coils
 */
Coils::~Coils() {}

/**
 * @brief Coils::getLeftValue
 * @return the left coil's current filtered signal
 */
float Coils::getLeftValue() const { return left_.getValue(); }

/**
 * @brief Coils::getRightValue
 * @return the right coil's current filtered signal
 */
float Coils::getRightValue() const { return right_.getValue(); }

/**
 * @brief Coils::getMeanValue
 * @return
 */
float Coils::getMeanValue() const
{
  return (left_.getValue() + right_.getValue()) / 2;
}

/**
 * @brief Coils::getLeftDerivedValue
 * @return
 */
float Coils::getLeftDerivedValue() const { return left_.getDerivedValue(); }

/**
 * @brief Coils::getRightDerivedValue
 * @return
 */
float Coils::getRightDerivedValue() const { return right_.getDerivedValue(); }

/**
 * @brief Coils::getMeanDerivedValue
 * @return
 */
float Coils::getMeanDerivedValue() const
{
  return (left_.getDerivedValue() + right_.getDerivedValue()) / 2;
}

/**
 * @brief Coils::getLeftSampleTime
 * @return
 */
double Coils::getLeftSampleTime() const { return left_.getSampleTime(); }

/**
 * @brief Coils::getRightSampleTime
 * @return
 */
double Coils::getRightSampleTime() const { return right_.getSampleTime(); }

/**
 * @brief Coils::getMeanSampleTime
 * @return
 */
double Coils::getMeanSampleTime() const
{
  return (left_.getSampleTime() + right_.getSampleTime()) / 2;
}

/**
 * @brief Coils::setMaxPoseUpdateInterval
 * @param max_pose_update_interval
 */
void Coils::setMaxPoseUpdateInterval(double max_pose_update_interval)
{
  left_.setMaxPoseUpdateInterval(max_pose_update_interval);
  right_.setMaxPoseUpdateInterval(max_pose_update_interval);
}

/**
 * @brief Coils::setSampleTime
 * @param sample_time
 */
void Coils::setSampleTime(double sample_time)
{
  left_.setSampleTime(sample_time);
  right_.setSampleTime(sample_time);
}

/**
 * @brief Coils::setLowThreshold sets the new desired low threshold.
 * @param low_threshold
 */
void Coils::setLowThreshold(double low_threshold)
{
  left_.setLowThreshold(low_threshold);
  right_.setLowThreshold(low_threshold);
}

/**
 * @brief Coils::setHighThreshold sets the new desired high threshold.
 * @param high_threshold
 */
void Coils::setHighThreshold(double high_threshold)
{
  left_.setHighThreshold(high_threshold);
  right_.setHighThreshold(high_threshold);
}

/**
 * @brief Coils::setNumberOfObservations sets the number of considered samples
 * in the filter.
 * @param number_of_observations
 */
void Coils::setNumberOfObservations(int number_of_observations)
{
  left_.setNumberOfObservations(number_of_observations);
  right_.setNumberOfObservations(number_of_observations);
}

/**
 * @brief Coils::setNumberOfDerivatives sets the number of considered samples
 * in the derivative vector.
 * @param number_of_derivatives
 */
void Coils::setNumberOfDerivatives(int number_of_derivatives)
{
  left_.setNumberOfDerivatives(number_of_derivatives);
  right_.setNumberOfDerivatives(number_of_derivatives);
}

/**
 * @brief Coils::isLeftLow verifies if the left coil's current signal is beneath
 * the low threshold.
 * @return
 */
bool Coils::isLeftLow() const { return left_.isLow(); }

/**
 * @brief Coils::isLeftHigh verifies if the left coil's current signal is above
 * the high threshold.
 * @return
 */
bool Coils::isLeftHigh() const { return left_.isHigh(); }

/**
 * @brief Coils::isRightLow verifies if the right coil's current signal is
 * beneath the low threshold.
 * @return
 */
bool Coils::isRightLow() const { return right_.isLow(); }

/**
 * @brief Coils::isRightHigh verifies if the right coil's current signal is
 * above the high threshold.
 * @return
 */
bool Coils::isRightHigh() const { return right_.isHigh(); }

/**
 * @brief Coils::isOneLow verifies if one and only one coil's current signal is
 * beneath the low threshold.
 * @return
 */
bool Coils::isOneLow() const { return left_.isLow() != right_.isLow(); }

/**
 * @brief Coils::isOneHigh verifies if one and only one coil's current signal is
 * above the high threshold.
 * @return
 */
bool Coils::isOneHigh() const { return left_.isHigh() != right_.isHigh(); }

/**
 * @brief Coils::isAnyLow verifies if current signal of left or right coil is
 * beneath the low threshold.
 * @return
 */
bool Coils::isAnyLow() const { return left_.isLow() || right_.isLow(); }

/**
 * @brief Coils::isAnyHigh verifies if current signal of left or right coil is
 * above the high threshold.
 * @return
 */
bool Coils::isAnyHigh() const { return left_.isHigh() || right_.isHigh(); }

/**
 * @brief Coils::isBothLow verifies if current signal of both coils are beneath
 * the low threshold.
 * @return
 */
bool Coils::isBothLow() const { return left_.isLow() && right_.isLow(); }

/**
 * @brief Coils::isBothHigh verifies if current signal of both coils are above
 * the high threshold.
 * @return
 */
bool Coils::isBothHigh() const { return left_.isHigh() && right_.isHigh(); }

/**
 * @brief Coils::isBothNotLow verifies if current signal of both coils are not
 * beneath the low threshold.
 * @return
 */
bool Coils::isBothNotLow() const { return !left_.isLow() && !right_.isLow(); }

/**
 * @brief Coils::isBothNotHigh verifies if current signal of both coils are not
 * above the high threshold.
 * @return
 */
bool Coils::isBothNotHigh() const
{
  return !left_.isHigh() && !right_.isHigh();
}

/**
 * @brief Coils::isSettedUp
 * @return
 */
bool Coils::isSettedUp()
{
  if (!left_updated_ && updateCoilTransform(&left_))
  {
    ROS_INFO("   Updated left transform.");
    left_updated_ = true;
  }
  if (!right_updated_ && updateCoilTransform(&right_))
  {
    ROS_INFO("   Updated right transform.");
    right_updated_ = true;
  }
  if (left_updated_ && right_updated_)
  {
    ROS_INFO("   Starting pose update timer.");
    pose_update_.start();
    return true;
  }
  return false;
}

/**
 * @brief Coils::calculateDerivative
 */
void Coils::calculateDerivative()
{
  left_.calculateDerivative();
  right_.calculateDerivative();
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
 * @brief Coils::setParameters
 */
void Coils::setParameters(const ros::NodeHandle& pnh)
{
  double aux;
  pnh.param("max_pose_update_interval", aux, DEFAULT_MAX_POSE_UPDATE_INTERVAL);
  ROS_INFO("   Maximun pose update interval: %f", aux);
  setMaxPoseUpdateInterval(aux);
  pnh.param("derivative_sample_time", aux, DEFAULT_DERIVATIVE_SAMPLE_TIME);
  ROS_INFO("   Derivative sample time: %f", aux);
  setSampleTime(aux);
  pnh.param("low_coil_signal_threshold", aux,
            DEFAULT_LOW_COIL_SIGNAL_THRESHOLD);
  ROS_INFO("   Low coil signal threshold: %f", aux);
  setLowThreshold(aux);
  pnh.param("high_coil_signal_threshold", aux,
            DEFAULT_HIGH_COIL_SIGNAL_THRESHOLD);
  ROS_INFO("   High coil signal threshold: %f", aux);
  setHighThreshold(aux);
  int number;
  pnh.param("coil_signal_filter_number_of_observations", number,
            DEFAULT_COIL_SIGNAL_FILTER_NUMBER_OF_OBSERVATIONS);
  ROS_INFO("   Coil signal filter number of observations: %d", number);
  setNumberOfObservations(number);
  pnh.param("number_of_derivatives", number, DEFAULT_NUMBER_OF_DERIVATIVES);
  ROS_INFO("   Number of derivatives: %d", number);
  setNumberOfDerivatives(number);
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
  return left_.getPose();
}

/**
 * @brief Coils::getRightPose
 * @return
 */
geometry_msgs::PoseStamped Coils::getRightPose() const
{
  return right_.getPose();
}

/**
 * @brief Coils::getMeanPose
 * @return
 */
geometry_msgs::PoseStamped Coils::getMidstPose() const
{
  geometry_msgs::PoseStamped midst_pose(left_.getPose());
  midst_pose.pose.position.x += right_.getPose().pose.position.x;
  midst_pose.pose.position.x /= 2;
  midst_pose.pose.position.y += right_.getPose().pose.position.y;
  midst_pose.pose.position.y /= 2;
  return midst_pose;
}

/**
 * @brief Coils::updatePose
 * @param event
 */
void Coils::updatePose(const ros::TimerEvent& event)
{
  try
  {
    tf::StampedTransform robot_tf;
    tf_.lookupTransform(MINEFIELD_FRAME_ID, BASE_FRAME_ID, ros::Time(0),
                        robot_tf);
    left_.setPose(robot_tf);
    right_.setPose(robot_tf);
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("Robot pose was not updated: %s", ex.what());
  }
}

/**
 * @brief Coils::updateCoilPose
 * @param coil
 */
bool Coils::updateCoilTransform(Coil* coil)
{
  try
  {
    tf::StampedTransform coil_tf;
    tf_.lookupTransform(BASE_FRAME_ID, coil->getFrameId(), ros::Time(0),
                        coil_tf);
    coil->setTransform(coil_tf);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }
  return true;
}
}
