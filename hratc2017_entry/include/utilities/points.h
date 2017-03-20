/**
 *  This header file defines the Points helper class. It contains helper static
 *methods in order to facilitate geometry_msgs::Point objects manipulation.
 *
 *  Version: 1.1.0
 *  Created on: 16/03/2017
 *  Modified on: 16/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_POINTS_H_
#define _UTILITIES_POINTS_H_

#include <geometry_msgs/Point.h>

namespace utilities
{
class Points
{
public:
  static double getEuclidianDistance(const geometry_msgs::Point& p1,
                                     const geometry_msgs::Point& p2);
  static double getEuclidianDistance(const geometry_msgs::Point& p1, double x2,
                                     double y2);
  static double getEuclidianDistance(double x1, double y1, double x2,
                                     double y2);
  static geometry_msgs::Point getMidstPoint(const geometry_msgs::Point& p1,
                                            const geometry_msgs::Point& p2);
  static geometry_msgs::Point getMidstPoint(const geometry_msgs::Point& p1,
                                            double x2, double y2);
  static geometry_msgs::Point getMidstPoint(double x1, double y1, double x2,
                                            double y2);
};
}

#endif // _UTILITIES_POINTS_H_
