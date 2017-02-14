/**
 *  This source file implements the LandmineAnalyzer class, which is
 *based on the ROSNode helper class. It controls the landmine_analyzer_node.
 *
 *  Version: 0.0.1
 *  Created on: 30/01/2017
 *  Modified on: 13/02/2017
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
LandmineAnalyzer::LandmineAnalyzer(ros::NodeHandle* nh) : ROSNode(nh, 30), tf_()
{
  ros::NodeHandle pnh("~");
  double threshold;
  pnh.param("threshold", threshold, COIL_SIGNAL_THRESHOLD);
  coils_.setThreshold(threshold);
  ROS_INFO("   Coil signal threshold: %lf", threshold);
  pnh.param("max_coil_signal", max_coil_singal_, MAX_COIL_SIGNAL);
  ROS_INFO("   Max coil signal: %lf", max_coil_singal_);
  pnh.param("alignment_tolerance", alignment_tolerance_, ALIGNMENT_TOLERANCE);
  ROS_INFO("   Max coil signal: %lf", alignment_tolerance_);
  pnh.param("sampling_end_interval", sampling_end_interval_, SAMPLING_END_INTERVAL);
  ROS_INFO("   Sampling end interval: %f [s]", sampling_end_interval_);
  pnh.param("minimal_signal_radius", min_signal_radius_, MIN_SIGNAL_RADIUS);
  ROS_INFO("   Minimal signal radius: %f", min_signal_radius_);
  pnh.param("maximal_signal_radius", max_signal_radius_, MAX_SIGNAL_RADIUS);
  ROS_INFO("   Max signal radius: %f", max_signal_radius_);
  coils_sub_ =
      nh->subscribe("/coils", 10, &LandmineAnalyzer::coilsCallback, this);
  set_fake_mine_pub_ =
      nh->advertise<geometry_msgs::PoseStamped>("/HRATC_FW/set_fake_mine", 1);
  set_mine_pub_ =
      nh->advertise<geometry_msgs::PoseStamped>("/HRATC_FW/set_mine", 1);
  polygon_pub_ =
      nh->advertise<geometry_msgs::PolygonStamped>("landmine/polygon", 10);
  pause_pub_ =
      nh->advertise<std_msgs::Bool>("pause_scanning", 1);
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
  set_fake_mine_pub_.shutdown();
  polygon_pub_.shutdown();
  pause_pub_.shutdown();
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
      if(possible_mine_found_)  // Se entrou nesta parte do código, os sensores foram saturados ao mesmo tempo, portanto analiza-se o raio do sinal para ver se é uma mina.
      {
        mine_center_.x = (p_max_left_.x + p_max_right_.x) / 2;
        mine_center_.y = (p_max_left_.y + p_max_right_.y) / 2;
        float radius(sqrt(pow(landmine_.polygon.points[0].x - mine_center_.x, 2) + pow(landmine_.polygon.points[0].y - mine_center_.y, 2)));
        ROS_INFO("   Signal radius = %f [meters]", radius);
        float area(M_PI * pow(radius, 2));
        ROS_INFO("   Signal area = %f [square meters]", area);
        if(radius >= min_signal_radius_ && radius <= max_signal_radius_)  // Se o raio tem os valores padrão para uma mina, a mina fora identificada.
        {
          publishLandminePose(mine_center_.x, mine_center_.y);
          ROS_INFO("   Real mine found!!!");
          ROS_INFO("   Mine location guess: [%f, %f]", mine_center_.x, mine_center_.y);
        }
        else  // Senão, temos um falso postivo, com sinal correto, mas raio pequeno demais.
        {
          publishFakeLandminePose(mine_center_.x, mine_center_.y, radius);
          ROS_INFO("   False mine found!!!");
          ROS_INFO("   False mine location guess: [%f, %f]", mine_center_.x, mine_center_.y);
        }
       }

      else  // Caso entre nesta parte do código, os sensores não foram saturados, ou seja, mesmo que o raio do sinal seja correto, é uma mina falsa pela baixa intensidade captada.
      {
        float divisor(0);
        for(int i = 0; i < landmine_.polygon.points.size(); i++)  // Como não fora setado um centro para o falso positivo, obtém-se seu valor pela média ponderada pela intensidade do sinal (colocada na coordenada z do landmine_.polygon.points - linhas 157 e 166), dos pontos encontrados na área de sinal elevado.
        {
          mine_center_.x += landmine_.polygon.points[i].x * landmine_.polygon.points[i].z;
          mine_center_.y += landmine_.polygon.points[i].y * landmine_.polygon.points[i].z;
          divisor += landmine_.polygon.points[i].z;
        }
        mine_center_.x /= divisor;
        mine_center_.y /= divisor;
        float radius(sqrt(pow(landmine_.polygon.points[0].x - mine_center_.x, 2) + pow(landmine_.polygon.points[0].y - mine_center_.y, 2)));
        ROS_INFO("   Signal radius = %f [meters]", radius);
        float area(M_PI * pow(radius, 2));
        ROS_INFO("   Signal area = %f [square meters]", area);
        publishFakeLandminePose(mine_center_.x, mine_center_.y, radius);  // Publica-se o falso positivo.
        ROS_INFO("   False mine found!!!");
        ROS_INFO("   False mine location guess: [%f, %f]", mine_center_.x, mine_center_.y);
      }
      reset();
//      if(possible_mine_found_ && radius >= min_signal_radius_)
//      {
//          publishLandminePose(mine_center_.x, mine_center_.y);
//          ROS_INFO("MINA REAL");
//      }else{
//        publishFakeLandminePose(mine_center_.x, mine_center_.y, radius);
//        ROS_INFO("MINA FALSA");
//      }
//      reset();

    }
    return;
  }
  if(!sampling_)
  {
    sampling_ = true;
    setScanning(true);

  }
  landmine_.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped coil_pose;
  geometry_msgs::Point32 p;
  if (coils_.isHighCoilSignalOnLeft())
  {
    coil_pose = getLeftCoilPose();
    p.x = coil_pose.pose.position.x;
    p.y = coil_pose.pose.position.y;
    p.z = coils_.getLeft(); // O peso (valor do sinal) no ponto foi salvo na cordenada z para se obter uma média ponderada.
    landmine_.polygon.points.push_back(p);
  }

  if (coils_.isHighCoilSignalOnRight())
  {
    coil_pose = getRightCoilPose();
    p.x = coil_pose.pose.position.x;
    p.y = coil_pose.pose.position.y;
    p.z = coils_.getRight();  // O peso (valor do sinal) no ponto foi salvo na cordenada z para se obter uma média ponderada.
    landmine_.polygon.points.push_back(p);
  }

  if (coils_.getLeft() >= max_coil_singal_ * alignment_tolerance_ &&
      coils_.getRight() >= max_coil_singal_ * alignment_tolerance_)
  {
    ROS_INFO("ENTROU!!!");
    p_max_left_.x = getLeftCoilPose().pose.position.x;
    p_max_left_.y = getLeftCoilPose().pose.position.y;
    p_max_right_.x = getRightCoilPose().pose.position.x;
    p_max_right_.y = getRightCoilPose().pose.position.y;
    possible_mine_found_ = true;
  }
  polygon_pub_.publish(landmine_);
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
  // ROS_INFO("[COILS CB] new reading: %s", coils_.c_str());
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

void LandmineAnalyzer::publishLandminePose(double x, double y) const
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "minefield";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = 0.0;
  msg.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  set_mine_pub_.publish(msg);
}

void LandmineAnalyzer::publishFakeLandminePose(double x, double y, double radius) const{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "minefield";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = radius;
  msg.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  set_fake_mine_pub_.publish(msg);
}

void LandmineAnalyzer::setScanning(bool scanning)
{
  std_msgs::Bool msg;
  msg.data = !scanning;
  pause_pub_.publish(msg);
}

void LandmineAnalyzer::reset()
{
  landmine_ = geometry_msgs::PolygonStamped();
  landmine_.header.frame_id = "minefield";
  sampling_ = false;
  possible_mine_found_ = false;
  setScanning(false);
}


}
