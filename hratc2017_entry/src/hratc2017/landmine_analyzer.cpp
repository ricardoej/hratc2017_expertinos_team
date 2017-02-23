/**
 *  This source file implements the LandmineAnalyzer class, which is
 *based on the ROSNode helper class. It controls the landmine_analyzer_node.
 *
 *  Version: 1.0.1
 *  Created on: 30/01/2017
 *  Modified on: 20/02/2017
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
    : ROSNode(nh, 30), coils_(new tf::TransformListener())
{
  ros::NodeHandle pnh("~");
  double threshold;
  pnh.param("low_coil_signal_threshold", threshold, LOW_COIL_SIGNAL_THRESHOLD);
  coils_.setLowThreshold(threshold);
  ROS_INFO("   Low coil signal threshold: %f", threshold);
  pnh.param("high_coil_signal_threshold", threshold,
            HIGH_COIL_SIGNAL_THRESHOLD);
  coils_.setHighThreshold(threshold);
  ROS_INFO("   High coil signal threshold: %f", threshold);
  int number_of_observations;
  pnh.param("coil_signal_filter_number_of_observations", number_of_observations,
            COIL_SIGNAL_FILTER_NUMBER_OF_OBSERVATIONS);
  coils_.setNumberOfObservations(number_of_observations);
  ROS_INFO("   Coil signal filter number of observations: %d",
           number_of_observations);
  pnh.param("max_coil_signal", max_coil_signal_, MAX_COIL_SIGNAL);
  ROS_INFO("   Max coil signal: %lf", max_coil_signal_);
  pnh.param("alignment_tolerance", alignment_tolerance_, ALIGNMENT_TOLERANCE);
  ROS_INFO("   Max coil signal: %lf", alignment_tolerance_);
  pnh.param("sampling_end_interval", sampling_end_interval_,
            SAMPLING_END_INTERVAL);
  ROS_INFO("   Sampling end interval: %f [s]", sampling_end_interval_);
  pnh.param("minimal_signal_radius", min_signal_radius_, MIN_SIGNAL_RADIUS);
  ROS_INFO("   Minimal signal radius: %f", min_signal_radius_);
  pnh.param("maximal_signal_radius", max_signal_radius_, MAX_SIGNAL_RADIUS);
  ROS_INFO("   Max signal radius: %f", max_signal_radius_);
  pnh.param("landmine_radius", landmine_radius_ , LANDMINE_RADIUS);
  ROS_INFO("   Landmine radius: %f", landmine_radius_);
  set_fake_mine_pub_ = nh->advertise<geometry_msgs::PoseStamped>(
      "/HRATC_FW/set_fake_mine", 10, true);
  set_mine_pub_ =
      nh->advertise<geometry_msgs::PoseStamped>("/HRATC_FW/set_mine", 10, true);
  polygon_pub_ =
      nh->advertise<geometry_msgs::PolygonStamped>("landmine/polygon", 1);
  pause_pub_ = nh->advertise<std_msgs::Bool>("start_scanning", 1, true);
  filtered_coils_pub_ =
      nh->advertise<metal_detector_msgs::Coil>("/coils/filtered", 10);
  coils_sub_ = nh->subscribe("/coils", 10, &Coils::coilsCallback, &coils_);
  reset();
}

/**
 * @brief LandmineAnalyzer::~LandmineAnalyzer
 */
LandmineAnalyzer::~LandmineAnalyzer()
{
  set_mine_pub_.shutdown();
  set_fake_mine_pub_.shutdown();
  polygon_pub_.shutdown();
  pause_pub_.shutdown();
  coils_sub_.shutdown();
  filtered_coils_pub_.shutdown();
}

/**
 * @brief LandmineAnalyzer::controlLoop
 */
void LandmineAnalyzer::controlLoop()
{
  publishFilteredCoilSignals();
  if (coils_.isBothNotHigh())
  {
    if (!sampling_)
    {
      return;
    }
    double elapsed_time((ros::Time::now() - landmine_.header.stamp).toSec());
    if (elapsed_time > sampling_end_interval_)
    {
      if (possible_mine_found_) // Se entrou nesta parte do código, os sensores
                                // foram saturados ao mesmo tempo, portanto
                                // analiza-se o raio do sinal para ver se é uma
                                // mina.
      {
        mine_center_.x = (p_max_left_.x + p_max_right_.x) / 2;
        mine_center_.y = (p_max_left_.y + p_max_right_.y) / 2;
        float radius(
            sqrt(pow(landmine_.polygon.points[0].x - mine_center_.x, 2) +
                 pow(landmine_.polygon.points[0].y - mine_center_.y, 2)));
        ROS_INFO("   Signal radius = %f [meters]", radius);
        float area(M_PI * pow(radius, 2));
        ROS_INFO("   Signal area = %f [square meters]", area);
        if (radius >= min_signal_radius_ &&
            radius <= max_signal_radius_) // Se o raio tem os valores padrão
                                          // para uma mina, a mina fora
                                          // identificada.
        {
          publishLandminePose(mine_center_.x, mine_center_.y);
          ROS_INFO("   Real mine found!!!");
          ROS_INFO("   Mine location guess: [%f, %f]", mine_center_.x,
                   mine_center_.y);
        }
        else // Senão, temos um falso postivo, com sinal correto, mas raio
             // pequeno demais.
        {
          publishFakeLandminePose(mine_center_.x, mine_center_.y, radius);
          ROS_INFO("   False mine found!!!");
          ROS_INFO("   False mine location guess: [%f, %f]", mine_center_.x,
                   mine_center_.y);
        }
      }

      else // Caso entre nesta parte do código, os sensores não foram saturados,
           // ou seja, mesmo que o raio do sinal seja correto, é uma mina falsa
           // pela baixa intensidade captada.
      {
        float divisor(0);
        for (int i = 0; i < landmine_.polygon.points.size();
             i++) // Como não fora setado um centro para o falso positivo,
        // obtém-se seu valor pela média ponderada pela intensidade do
        // sinal (colocada na coordenada z do landmine_.polygon.points
        // - linhas 157 e 166), dos pontos encontrados na área de
        // sinal elevado.
        {
          mine_center_.x +=
              landmine_.polygon.points[i].x * landmine_.polygon.points[i].z;
          mine_center_.y +=
              landmine_.polygon.points[i].y * landmine_.polygon.points[i].z;
          divisor += landmine_.polygon.points[i].z;
        }
        mine_center_.x /= divisor;
        mine_center_.y /= divisor;
        float radius(
            sqrt(pow(landmine_.polygon.points[0].x - mine_center_.x, 2) +
                 pow(landmine_.polygon.points[0].y - mine_center_.y, 2)));
        ROS_INFO("   Signal radius = %f [meters]", radius);
        float area(M_PI * pow(radius, 2));
        ROS_INFO("   Signal area = %f [square meters]", area);
        publishFakeLandminePose(mine_center_.x, mine_center_.y,
                                radius); // Publica-se o falso positivo.
        ROS_INFO("   False mine found!!!");
        ROS_INFO("   False mine location guess: [%f, %f]", mine_center_.x,
                 mine_center_.y);
      }
      reset();
    }
    return;
  }

  //é necessário ficar publicando para twist_mux travar o navigation
  // constantemente
  sampling_ = true;
  setScanning(true);
  landmine_.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped coil_pose;
  geometry_msgs::Point32 p;
  if (coils_.isLeftHigh())
  {
    coil_pose = coils_.getLeftPose();
    p.x = coil_pose.pose.position.x;
    p.y = coil_pose.pose.position.y;
    p.z =
        coils_.getLeftValue(); // O peso (valor do sinal) no ponto foi salvo na
    // cordenada z para se obter uma média ponderada.
    landmine_.polygon.points.push_back(p);
  }
  if (coils_.isRightHigh())
  {
    coil_pose = coils_.getRightPose();
    p.x = coil_pose.pose.position.x;
    p.y = coil_pose.pose.position.y;
    p.z =
        coils_.getRightValue(); // O peso (valor do sinal) no ponto foi salvo na
    // cordenada z para se obter uma média ponderada.
    landmine_.polygon.points.push_back(p);
  }

  if (fabs(max_coil_signal_ - coils_.getLeftValue()) <= alignment_tolerance_ &&
      fabs(max_coil_signal_ - coils_.getRightValue()) <= alignment_tolerance_)
  {
    ROS_INFO("Possible mine found on both coils!!!");
    coil_pose = coils_.getLeftPose();
    p_max_left_.x = coil_pose.pose.position.x;
    p_max_left_.y = coil_pose.pose.position.y;
    coil_pose = coils_.getRightPose();
    p_max_right_.x = coil_pose.pose.position.x;
    p_max_right_.y = coil_pose.pose.position.y;
    possible_mine_found_ = true;
    max_signal_found_in_both_ = true;
  }
  else
  {
    if (!max_signal_found_in_both_)
    {
      if (coils_.getLeftValue() >= max_coil_signal_ ||
          coils_.getRightValue() >= max_coil_signal_)
      {
        ROS_INFO("Possible mine found in left coil!!!");
        coil_pose = coils_.getLeftPose();
        p_max_left_.x = coil_pose.pose.position.x;
        p_max_left_.y = coil_pose.pose.position.y;
        coil_pose = coils_.getRightPose();
        p_max_right_.x = coil_pose.pose.position.x;
        p_max_right_.y = coil_pose.pose.position.y;
        possible_mine_found_ = true;
      }
    }
  }
  polygon_pub_.publish(landmine_);
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
  real_landmines_.push_back(msg.pose.position);
  set_mine_pub_.publish(msg);
}

/**
 * @brief LandmineAnalyzer::publishFakeLandminePose
 * @param x
 * @param y
 * @param radius
 */
void LandmineAnalyzer::publishFakeLandminePose(double x, double y,
                                               double radius) const
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "minefield";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = radius;
  msg.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  set_fake_mine_pub_.publish(msg);
}

/**
 * @brief LandmineAnalyzer::publishFilteredCoilSignals
 */
void LandmineAnalyzer::publishFilteredCoilSignals() const
{
  filtered_coils_pub_.publish(coils_.to_msg());
}

bool LandmineAnalyzer::isKnownLandmine() const
{
  if(real_landmines_.empty())
    return false;
  geometry_msgs::PoseStamped coil_pose;
  if(coils_.isLeftHigh())
  {
    coil_pose = coils_.getLeftPose();
  }else{
    coil_pose = coils_.getRightPose();
  }
  double delta_x, delta_y;

  for(int i=0; i<real_landmines_.size(); i++){
    delta_x = coil_pose.pose.position.x - real_landmines_[i].x;
    delta_y = coil_pose.pose.position.y - real_landmines_[i].y;
    if(pow(delta_x, 2)+pow(delta_y,2) <= pow(landmine_radius_,2) ){
      ROS_INFO("already known landmine");
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
  pause_pub_.publish(msg);
}

/**
 * @brief LandmineAnalyzer::reset
 */
void LandmineAnalyzer::reset()
{
  landmine_ = geometry_msgs::PolygonStamped();
  landmine_.header.frame_id = "minefield";
  sampling_ = false;
  possible_mine_found_ = false;
  max_signal_found_in_both_ = false;
  setScanning(false);
}
}
