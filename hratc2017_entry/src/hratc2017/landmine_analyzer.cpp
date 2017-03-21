/**
 *  This source file implements the LandmineAnalyzer class, which is
 *based on the ROSNode helper class. It controls the landmine_analyzer_node.
 *
 *  Version: 1.1.1
 *  Created on: 30/01/2017
 *  Modified on: 13/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *          Luis Victor Pessiqueli Bonin (luis-bonin@unifei.edu.br)
 *          Luiz Fernando Nunes (luizfernandolfn@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "hratc2017/landmine_analyzer.h"

namespace hratc2017
{

/**
 * @brief LandmineAnalyzer::LandmineAnalyzer builds a new LandmineAnalyzer
 * controller. This object is responsible for keeps the landmines map
 * up-to-date.
 * @param nh
 */
LandmineAnalyzer::LandmineAnalyzer(ros::NodeHandle* nh)
    : ROSNode(nh, 30), coils_(new tf::TransformListener()), moving_away_(false)
{
  ros::NodeHandle pnh("~");
  coils_.setParameters(pnh);
  pnh.param("max_coil_signal", max_coil_signal_, MAX_COIL_SIGNAL);
  ROS_INFO("   Max coil signal: %lf", max_coil_signal_);
  pnh.param("end_duration", end_duration_, END_DURATION);
  ROS_INFO("   Sampling end duration: %f", end_duration_);
  pnh.param("min_radius", min_radius_, MIN_RADIUS);
  ROS_INFO("   Min valid radius: %f", min_radius_);
  pnh.param("max_radius", max_radius_, MAX_RADIUS);
  ROS_INFO("   Max valid radius: %f", max_radius_);
  pnh.param("std_radius", std_radius_, STD_RADIUS);
  ROS_INFO("   Standard radius of separation: %f", std_radius_);
  sampler_ = nh->createTimer(ros::Duration(coils_.getLeftSampleTime()),
                             &LandmineAnalyzer::derivativeCallback, this);
  set_mine_pub_ =
      nh->advertise<geometry_msgs::PoseStamped>("/HRATC_FW/set_mine", 10, true);
  set_fake_mine_pub_ = nh->advertise<geometry_msgs::PoseStamped>(
      "/HRATC_FW/set_fake_mine", 10, true);
  scanning_pub_ = nh->advertise<std_msgs::Bool>("scanning", 1, true);
  filtered_coils_pub_ =
      nh->advertise<metal_detector_msgs::Coil>("/coils/filtered", 10);
  coils_sub_ = nh->subscribe("/coils", 10, &Coils::coilsCallback, &coils_);
  moving_away_sub_ = nh->subscribe("moving_away", 1,
                                   &LandmineAnalyzer::movingAwayCallback, this);
  reset();
}

/**
 * @brief LandmineAnalyzer::~LandmineAnalyzer
 */
LandmineAnalyzer::~LandmineAnalyzer()
{
  sampler_.stop();
  set_mine_pub_.shutdown();
  set_fake_mine_pub_.shutdown();
  scanning_pub_.shutdown();
  filtered_coils_pub_.shutdown();
  coils_sub_.shutdown();
  moving_away_sub_.shutdown();
}

/**
 * @brief LandmineAnalyzer::controlLoop
 */
void LandmineAnalyzer::controlLoop()
{
  publishFilteredCoilSignals();
  if (coils_.isBothLow())
  {
    if (sampling_ && (ros::Time::now() - end_time_).toSec() > end_duration_)
    {
      publish();
      reset();
    }
    return;
  }
  if (!moving_away_)
  {
    sample();
  }
}

/**
 * @brief LandmineAnalyzer::sample
 */
void LandmineAnalyzer::sample()
{
  setScanning(true);
  end_time_ = ros::Time::now();
  geometry_msgs::PoseStamped midst_pose(coils_.getMidstPose());
  if (!collected_center_)
  {
    mine_center_ = midst_pose.pose.position;
    if (!sampling_)
    {
      sampling_ = true;
      mine_bound_ = midst_pose.pose.position;
    }
  }
  else if (coils_.isBothNotLow())
  {
    mine_bound_ = midst_pose.pose.position;
  }
  if (coils_.isAnyHigh())
  {
    collected_center_ = true;
  }
}

/**
 * @brief LandmineAnalyzer::publish
 */
void LandmineAnalyzer::publish()
{
  double radius(calculateRadius());
  if (radius >= min_radius_ && radius <= max_radius_)
  {
    publishLandminePose();
  }
  else
  {
    publishFakeLandminePose();
  }
}

/**
 * @brief LandmineAnalyzer::publishLandminePose
 */
void LandmineAnalyzer::publishLandminePose()
{
  publishLandminePose(mine_center_.x, mine_center_.y);
}

/**
 * @brief LandmineAnalyzer::publishLandminePose
 * @param x
 * @param y
 */
void LandmineAnalyzer::publishLandminePose(double x, double y)
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "minefield";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = 0.0;
  msg.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  set_mine_pub_.publish(msg);
  ROS_INFO("   Real mine found @ (%lf, %lf)!!!", x, y);
}

/**
 * @brief LandmineAnalyzer::publishFakeLandminePose
 */
void LandmineAnalyzer::publishFakeLandminePose()
{
  publishFakeLandminePose(mine_center_.x, mine_center_.y, calculateRadius());
}

/**
 * @brief LandmineAnalyzer::publishFakeLandminePose
 * @param x
 * @param y
 * @param radius
 */
void LandmineAnalyzer::publishFakeLandminePose(double x, double y,
                                               double radius)
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "minefield";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = radius;
  msg.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  set_fake_mine_pub_.publish(msg);
  ROS_INFO("   Fake mine found @ (%lf, %lf) with radius %lf [m]!!!", x, y,
           radius);
}

/**
 * @brief LandmineAnalyzer::publishFilteredCoilSignals
 */
void LandmineAnalyzer::publishFilteredCoilSignals() const
{
  filtered_coils_pub_.publish(coils_.to_msg());
}

/**
 * @brief LandmineAnalyzer::setScanning
 * @param scanning
 */
void LandmineAnalyzer::setScanning(bool scanning)
{
  std_msgs::Bool msg;
  msg.data = scanning;
  scanning_pub_.publish(msg);
}

/**
 * @brief LandmineAnalyzer::reset
 */
void LandmineAnalyzer::reset()
{
  ROSNode::reset();
  sampling_ = false;
  setScanning(false);
  collected_center_ = false;
  mine_center_ = geometry_msgs::Point();
  mine_bound_ = geometry_msgs::Point();
}

/**
 * @brief LandmineAnalyzer::derivativeCallback
 * @param event
 */
void LandmineAnalyzer::derivativeCallback(const ros::TimerEvent& event)
{
  coils_.calculateDerivative();
  double derivative(coils_.getMeanDerivedValue());
  // ROS_INFO("derived value: %lf", derivative);
  if (sampling_ && !moving_away_ && derivative < 0.0)
  {
    ROS_WARN("Negative derived value!!");
    /*
    publishFakeLandminePose();
    reset();*/
  }
}

/**
 * @brief LandmineAnalyzer::movingAwayCallback
 * @param msg
 */
void LandmineAnalyzer::movingAwayCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (moving_away_ != msg->data)
  {
    moving_away_ = msg->data;
    ROS_INFO("   moving away: %s", moving_away_ ? "true" : "false");
  }
}

/**
 * @brief LandmineAnalyzer::calculateDistance
 * @param p1
 * @param p2
 * @return
 */
double LandmineAnalyzer::calculateDistance(geometry_msgs::Point p1,
                                           geometry_msgs::Point p2) const
{
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

/**
 * @brief LandmineAnalyzer::calculateRadius
 * @return
 */
double LandmineAnalyzer::calculateRadius() const
{
  return calculateDistance(mine_center_, mine_bound_);
}
}
