/**
 *  This header file defines the Coil class.
 *
 *  Version: 1.0.1
 *  Created on: 16/01/2017
 *  Modified on: 16/02/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_SENSORS_COIL_H_
#define _HRATC2017_SENSORS_COIL_H_

#include <list>
#include <sstream>

#define HIGH_COIL_SIGNAL_THRESHOLD 0.65
#define LOW_COIL_SIGNAL_THRESHOLD 0.45
#define COIL_SIGNAL_FILTER_NUMBER_OF_OBSERVATIONS 6

namespace hratc2017
{

class Coil
{
public:
  Coil(std::string frame_id, float low_threshold = LOW_COIL_SIGNAL_THRESHOLD,
       float high_threshold = HIGH_COIL_SIGNAL_THRESHOLD,
       int number_of_observations = COIL_SIGNAL_FILTER_NUMBER_OF_OBSERVATIONS);
  virtual ~Coil();
  std::string getFrameId() const;
  float getValue() const;
  void setLowThreshold(double low_threshold);
  void setHighThreshold(double high_threshold);
  void setNumberOfObservations(int number_of_observations);
  bool isLow() const;
  bool isHigh() const;
  std::string str() const;
  const char* c_str() const;
  void operator=(float value);

private:
  std::string frame_id_;
  float value_;
  float low_threshold_;
  float high_threshold_;
  int number_of_observations_;
  std::list<float> samples_;
};
}

#endif /* _HRATC2017_SENSORS_COIL_H_ */
