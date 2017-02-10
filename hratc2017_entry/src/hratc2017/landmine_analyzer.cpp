/**
 *  This source file implements the LandmineAnalyzer class, which is
 *based on the ROSNode helper class. It controls the landmine_analyzer_node.
 *
 *  Version: 0.0.1
 *  Created on: 30/01/2017
 *  Modified on: 01/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *          Luis Victor Pessiqueli Bonin (luis-bonin@unifei.edu.br)
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
LandmineAnalyzer::LandmineAnalyzer(ros::NodeHandle* nh) : ROSNode(nh, 30), tf_()
{
  ros::NodeHandle pnh("~");
  pnh.param("threshold", threshold_, COIL_SIGNAL_THRESHOLD);
  ROS_INFO("   Coil signal threshold: %lf", threshold_);
  pnh.param("sampling_end_interval", sampling_end_interval_,
            SAMPLING_END_INTERVAL);
  ROS_INFO("   Sampling end interval: %f [s]", sampling_end_interval_);
  coils_sub_ =
      nh->subscribe("/coils", 10, &LandmineAnalyzer::coilsCallback, this);
  set_mine_pub_ =
      nh->advertise<geometry_msgs::PoseStamped>("/HRATC_FW/set_mine", 1);
  EMPTY_POSE.header.frame_id = "UNDEF";
  EMPTY_POSE.pose.position.x = 0;
  EMPTY_POSE.pose.position.y = 0;
  EMPTY_POSE.pose.position.z = 0;
  quaternionTFToMsg(tf::createQuaternionFromYaw(0.0 * M_PI / 180.0),
                    EMPTY_POSE.pose.orientation);
  reset();
}

/**
 * @brief LandmineAnalyzer::~LandmineAnalyzer
 */
LandmineAnalyzer::~LandmineAnalyzer()
{
  coils_sub_.shutdown();
  set_mine_pub_.shutdown();
}

/**
 * @brief LandmineAnalyzer::controlLoop
 */
void LandmineAnalyzer::controlLoop()
{
  if (!coils_.isHighCoilSignalOnLeft() && !coils_.isHighCoilSignalOnRight())
  {
    if (!sampling_)
    {
      return;
    }
    double elapsed_time((ros::Time::now() - landmine_.header.stamp).toSec());
    if (elapsed_time > sampling_end_interval_)
    {
      ROS_INFO("CÁLCULOS");
      reset();
    }
  }
  sampling_ = true;
  landmine_.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped coil_pose;
  geometry_msgs::Point32 p;
  if (coils_.isHighCoilSignalOnLeft())
  {
    coil_pose = getLeftCoilPose();
    p.x = coil_pose.pose.position.x;
    p.y = coil_pose.pose.position.y;
    landmine_.polygon.points.push_back(p);
  }
  if (coils_.isHighCoilSignalOnRight())
  {
    coil_pose = getRightCoilPose();
    p.x = coil_pose.pose.position.x;
    p.y = coil_pose.pose.position.y;
    landmine_.polygon.points.push_back(p);
  }
}

/**
 * @brief LandmineAnalyzer::landmineDetected publishes the landmine pose based
 * on the robot's current pose.
 */
void LandmineAnalyzer::landmineDetected(bool left_coil) const
{
  geometry_msgs::PoseStamped coil_pose(getCoilPose(left_coil));
  landmineDetected(coil_pose.pose.position.x, coil_pose.pose.position.y);
}

/**
 * @brief LandmineAnalyzer::landmineDetected publishes the given landmine pose.
 * @param x
 * @param y
 */
void LandmineAnalyzer::landmineDetected(double x, double y) const
{
  geometry_msgs::PoseStamped mine_pose;
  mine_pose.header.stamp = ros::Time::now();
  mine_pose.header.frame_id = "minefield";
  mine_pose.pose.position.x = x;
  mine_pose.pose.position.y = y;
  mine_pose.pose.position.z = 0.0;
  mine_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  ROS_INFO("Setting a mine on x:%lf y:%lf", x, y);
  set_mine_pub_.publish(mine_pose);
}

/**
 * @brief LandmineAnalyzer::coilsCallback receives the metal detector coils
 * signal data whenever a new one is available.
 * @param msg new coils signal data.
 */
void LandmineAnalyzer::coilsCallback(
    const metal_detector_msgs::Coil::ConstPtr& msg)
{
  coils_ = msg;
  //ROS_INFO("[COILS CB] new reading: %s", coils_.c_str());
}

/**
 * @brief LandmineAnalyzer::getRobotPose
 * @return
 */
geometry_msgs::PoseStamped LandmineAnalyzer::getRobotPose() const
{
  geometry_msgs::PoseStamped robot_pose;
  tf::StampedTransform robot_transform;
  ros::Time now(ros::Time::now());
  try
  {
    tf_.waitForTransform("/minefield", "/base_link", now, ros::Duration(2.0));
    tf_.lookupTransform("/minefield", "/base_link", now, robot_transform);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    // tem que tratar melhor essa excessao!!!
    return EMPTY_POSE;
  }
  robot_pose.header.frame_id = "base_link";
  robot_pose.header.stamp = ros::Time::now();
  robot_pose.pose.position.x = robot_transform.getOrigin().x();
  robot_pose.pose.position.y = robot_transform.getOrigin().y();
  robot_pose.pose.position.z = robot_transform.getOrigin().z();
  quaternionTFToMsg(robot_transform.getRotation(), robot_pose.pose.orientation);
  return robot_pose;
}

/**
 * @brief LandmineAnalyzer::getCoilPose
 * @param robot_pose
 * @return
 */
geometry_msgs::PoseStamped LandmineAnalyzer::getCoilPose(bool left_coil) const
{
  geometry_msgs::PoseStamped coil_pose;
  tf::StampedTransform coil_transform;
  ros::Time now(ros::Time::now());
  try
  {
    tf_.waitForTransform("/base_link", "/left_coil", now, ros::Duration(2.0));
    tf_.lookupTransform("/base_link", "/left_coil", now, coil_transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    // tem que tratar melhor essa excessao!!!
    return EMPTY_POSE;
  }
  geometry_msgs::PoseStamped robot_pose(LandmineAnalyzer::getRobotPose());
  tf::Transform robot_transform;
  robot_transform.setOrigin(tf::Vector3(robot_pose.pose.position.x,
                                        robot_pose.pose.position.y,
                                        robot_pose.pose.position.z));
  tf::Quaternion q;
  quaternionMsgToTF(robot_pose.pose.orientation, q);
  robot_transform.setRotation(q);
  tf::Transform result;
  result.mult(robot_transform, coil_transform);
  coil_pose.header.frame_id = left_coil ? "left_coil" : "right_coil";
  coil_pose.header.stamp = ros::Time::now();
  coil_pose.pose.position.x = result.getOrigin().x();
  coil_pose.pose.position.y = result.getOrigin().y();
  coil_pose.pose.position.z = result.getOrigin().z();
  quaternionTFToMsg(result.getRotation(), coil_pose.pose.orientation);
  return coil_pose;
}

/**
 * @brief LandmineAnalyzer::getLeftCoilPose
 * @param robot_pose
 * @return
 */
geometry_msgs::PoseStamped LandmineAnalyzer::getLeftCoilPose() const
{
  return getCoilPose(true);
}

/**
 * @brief LandmineAnalyzer::getRightCoilPose
 * @param robot_pose
 * @return
 */
geometry_msgs::PoseStamped LandmineAnalyzer::getRightCoilPose() const
{
  return getCoilPose(false);
}

void LandmineAnalyzer::reset()
{
  landmine_ = geometry_msgs::PolygonStamped();
  sampling_ = false;
}
}
