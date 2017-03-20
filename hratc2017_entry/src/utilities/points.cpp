/**
 *  This source file implements the Points helper class.
 *
 *  Version: 1.1.0
 *  Created on: 16/03/2017
 *  Modified on: 16/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/points.h"

namespace utilities
{

/**
 * @brief Points::getEuclidianDistance
 * @param p1
 * @param p2
 * @return
 */
double Points::getEuclidianDistance(const geometry_msgs::Point& p1,
                                          const geometry_msgs::Point& p2)
{
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

/**
 * @brief Points::getEuclidianDistance
 * @param p1
 * @param x2
 * @param y2
 * @return
 */
double Points::getEuclidianDistance(const geometry_msgs::Point& p1,
                                          double x2, double y2)
{
  sqrt(pow(p1.x - x2, 2) + pow(p1.y - y2, 2));
}

/**
 * @brief Points::getEuclidianDistance
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @return
 */
double Points::getEuclidianDistance(double x1, double y1, double x2,
                                          double y2)
{
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

/**
 * @brief Points::getMidstPoint
 * @param p1
 * @param p2
 * @return
 */
geometry_msgs::Point Points::getMidstPoint(const geometry_msgs::Point& p1,
                                                 const geometry_msgs::Point& p2)
{
  geometry_msgs::Point p(p1);
  p.x += p2.x;
  p.x /= 2;
  p.y += p2.y;
  p.y /= 2;
  return p;
}

/**
 * @brief Points::getMidstPoint
 * @param p1
 * @param x2
 * @param y2
 * @return
 */
geometry_msgs::Point Points::getMidstPoint(const geometry_msgs::Point& p1,
                                                 double x2, double y2)
{
  geometry_msgs::Point p(p1);
  p.x += x2;
  p.x /= 2;
  p.y += y2;
  p.y /= 2;
  return p;
}

/**
 * @brief Points::getMidstPoint
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @return
 */
geometry_msgs::Point Points::getMidstPoint(double x1, double y1,
                                                 double x2, double y2)
{
  geometry_msgs::Point p;
  p.x = (x1 + x2) / 2;
  p.y = (y1 + y2) / 2;
  return p;
}
}
