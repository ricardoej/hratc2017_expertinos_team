/**
 *  This source file implements the LandmineAnalyzer class, which is
 *based on the ROSNode helper class. It controls the landmine_analyzer_node.
 *
 *  Version: 1.0.4
 *  Created on: 30/01/2017
 *  Modified on: 10/03/2017
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
  pnh.param("max_tolerance", max_tolerance_, MAX_TOLERANCE);
  ROS_INFO("   Max tolerance: %lf", max_tolerance_);
  pnh.param("sampling_end_interval", sampling_end_interval_,
            SAMPLING_END_INTERVAL);
  ROS_INFO("   Sampling end interval: %f", sampling_end_interval_);
  pnh.param("min_radius", min_radius_, MIN_RADIUS);
  ROS_INFO("   Min valid radius: %f", min_radius_);
  pnh.param("max_radius", max_radius_, MAX_RADIUS);
  ROS_INFO("   Max valid radius: %f", max_radius_);
  pnh.param("std_radius", std_radius_, STD_RADIUS);
  ROS_INFO("   Standard radius: %f", std_radius_);
  sampler_ = nh->createTimer(ros::Duration(coils_.getLeftSampleTime()),
                             &LandmineAnalyzer::derivativeCallback, this);
  set_mine_pub_ =
      nh->advertise<geometry_msgs::PoseStamped>("/HRATC_FW/set_mine", 10, true);
  set_fake_mine_pub_ = nh->advertise<geometry_msgs::PoseStamped>(
      "/HRATC_FW/set_fake_mine", 10, true);
  polygon_pub_ =
      nh->advertise<geometry_msgs::PolygonStamped>("landmine/polygon", 1);
  scanning_pub_ = nh->advertise<std_msgs::Bool>("scanning", 1, true);
  filtered_coils_pub_ =
      nh->advertise<metal_detector_msgs::Coil>("/metal_detector/filtered", 10);
  coils_sub_ = nh->subscribe("/metal_detector", 10, &Coils::coilsCallback, &coils_);
  moving_away_sub_ = nh->subscribe("moving_away", 1,
                                   &LandmineAnalyzer::movingAwayCallback, this);

  /*************/
  temporario_ = ros::Time::now();
  /***********/


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
  polygon_pub_.shutdown();
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


  /********/
  if ((ros::Time::now() - temporario_).toSec() <= 5 * 60)
  {
    return;
  }
  /*******/

  publishFilteredCoilSignals();
  if (coils_.isBothNotHigh())
  {
    if (sampling_ &&
        (ros::Time::now() - landmine_.header.stamp).toSec() >
            sampling_end_interval_)
    {
      if (analyze())
      {
        publishFakeLandminePose(mine_center_.x, mine_center_.y, mine_radius_);
      }
      else
      {
        publishLandminePose(mine_center_.x, mine_center_.y);
      }
      reset();
    }
    return;
  }
  if (isKnownLandmine())
  {
    if (sampling_)
    {
      reset();
    }
    return;
  }
  if (!moving_away_)
  {
    sampling_ = true;
    setScanning(true);
    landmine_.header.stamp = ros::Time::now();
    sample();
    polygon_pub_.publish(landmine_);
  }
}

/**
 * @brief LandmineAnalyzer::analyze
 * @return
 */
bool LandmineAnalyzer::analyze()
{
  // Se entrou nesta parte do código, os sensores
  // foram saturados ao mesmo tempo, portanto
  // analiza-se o raio do sinal para ver se é uma mina.
  if (possible_mine_found_)
  {
    mine_center_.x = (p_max_left_.x + p_max_right_.x) / 2;
    mine_center_.y = (p_max_left_.y + p_max_right_.y) / 2;
  }
  // Caso entre nesta parte do código, os sensores não foram saturados,
  // ou seja, mesmo que o raio do sinal seja correto, é uma mina falsa
  // pela baixa intensidade captada.
  else
  {
    float divisor(0.0);
    // Como não fora setado um centro para o falso positivo,
    // obtém-se seu valor pela média ponderada pela intensidade do
    // sinal (colocada na coordenada z do landmine_.polygon.points),
    // dos pontos encontrados na área de sinal elevado.
    for (int i(0); i < landmine_.polygon.points.size(); i++)
    {
      mine_center_.x +=
          landmine_.polygon.points[i].x * landmine_.polygon.points[i].z;
      mine_center_.y +=
          landmine_.polygon.points[i].y * landmine_.polygon.points[i].z;
      divisor += landmine_.polygon.points[i].z;
    }
    mine_center_.x /= divisor;
    mine_center_.y /= divisor;
  }
  mine_radius_ = sqrt(pow(landmine_.polygon.points[0].x - mine_center_.x, 2) +
                      pow(landmine_.polygon.points[0].y - mine_center_.y, 2));
  ROS_ERROR("found radius: %lf", mine_radius_);
  return possible_mine_found_ && mine_radius_ >= min_radius_ &&
         mine_radius_ <= max_radius_;
}

/**
 * @brief LandmineAnalyzer::sample
 */
void LandmineAnalyzer::sample()
{
  geometry_msgs::PoseStamped left_coil_pose(coils_.getLeftPose());
  geometry_msgs::PoseStamped right_coil_pose(coils_.getRightPose());
  geometry_msgs::Point32 p;
  if (coils_.isLeftHigh())
  {
    p.x = left_coil_pose.pose.position.x;
    p.y = left_coil_pose.pose.position.y;
    // O peso (valor do sinal) no ponto foi salvo na
    // cordenada z para se obter uma média ponderada.
    p.z = coils_.getLeftValue();
    landmine_.polygon.points.push_back(p);
  }
  if (coils_.isRightHigh())
  {
    p.x = right_coil_pose.pose.position.x;
    p.y = right_coil_pose.pose.position.y;
    // O peso (valor do sinal) no ponto foi salvo na
    // cordenada z para se obter uma média ponderada.
    p.z = coils_.getRightValue();
    landmine_.polygon.points.push_back(p);
  }
  bool left_ok(fabs(max_coil_signal_ - coils_.getLeftValue()) <=
               max_tolerance_);
  bool right_ok(fabs(max_coil_signal_ - coils_.getRightValue()) <=
                max_tolerance_);
  if (left_ok || right_ok)
  {
    possible_mine_found_ = true;
    if (left_ok)
    {
      p_max_left_.x = left_coil_pose.pose.position.x;
      p_max_left_.y = left_coil_pose.pose.position.y;
    }
    else if (!max_signal_found_in_both_)
    {
      if (!possible_mine_found_)
      {
        ROS_INFO("Possible mine found on right coil!!!");
      }
      p_max_left_.x = right_coil_pose.pose.position.x;
      p_max_left_.y = right_coil_pose.pose.position.y;
    }
    if (right_ok)
    {
      p_max_right_.x = right_coil_pose.pose.position.x;
      p_max_right_.y = right_coil_pose.pose.position.y;
    }
    else if (!max_signal_found_in_both_)
    {
      if (!possible_mine_found_)
      {
        ROS_INFO("Possible mine found in left coil!!!");
      }
      p_max_right_.x = left_coil_pose.pose.position.x;
      p_max_right_.y = left_coil_pose.pose.position.y;
    }
    if (!max_signal_found_in_both_ && left_ok && right_ok)
    {
      ROS_INFO("Possible mine found on both coils!!!");
      max_signal_found_in_both_ = true;
    }
  }
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
  known_landmines_.push_back(msg.pose.position);
  set_mine_pub_.publish(msg);
  ROS_INFO("   Real mine found @ (%lf, %lf)!!!", x, y);
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
  known_landmines_.push_back(msg.pose.position);
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
 * @brief LandmineAnalyzer::isKnownLandmine
 * @return
 */
bool LandmineAnalyzer::isKnownLandmine() const
{
  if (known_landmines_.empty())
  {
    return false;
  }
  geometry_msgs::PoseStamped coil_pose;
  if (coils_.isLeftHigh())
  {
    coil_pose = coils_.getLeftPose();
  }
  else
  {
    coil_pose = coils_.getRightPose();
  }
  double delta_x, delta_y;
  for (int i(0); i < known_landmines_.size(); i++)
  {
    delta_x = coil_pose.pose.position.x - known_landmines_[i].x;
    delta_y = coil_pose.pose.position.y - known_landmines_[i].y;
    if (pow(delta_x, 2) + pow(delta_y, 2) <= pow(std_radius_, 2))
    {
      ROS_INFO("Already known landmine.");
      return true;
    }
  }
  return false;
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
  ROS_INFO("   Reseting Landmine Analyzer!!!");
  landmine_ = geometry_msgs::PolygonStamped();
  landmine_.header.frame_id = "minefield";
  sampling_ = false;
  possible_mine_found_ = false;
  max_signal_found_in_both_ = false;
  setScanning(false);
}

/**
 * @brief LandmineAnalyzer::derivativeCallback
 * @param event
 */
void LandmineAnalyzer::derivativeCallback(const ros::TimerEvent& event)
{
  coils_.calculateDerivative();
  double derivative(coils_.getMeanDerivedValue());
  //ROS_INFO("derived value: %lf", derivative);
  if (sampling_ && !moving_away_ && derivative < 0.0)
  {
    ROS_WARN("Negative derived value!!");
    /*analyze();
    publishFakeLandminePose(mine_center_.x, mine_center_.y, mine_radius_);
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
}
