/**
 *  This header file defines the Coil class.
 *
 *  Version: 1.0.4
 *  Created on: 16/01/2017
 *  Modified on: 10/03/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _HRATC2017_SENSORS_COIL_H_
#define _HRATC2017_SENSORS_COIL_H_

#include <list>
#include <sstream>

#define DEFAULT_DERIVATIVE_SAMPLE_TIME 0.1
#define DEFAULT_LOW_COIL_SIGNAL_THRESHOLD 0.45
#define DEFAULT_HIGH_COIL_SIGNAL_THRESHOLD 0.65
#define DEFAULT_COIL_SIGNAL_FILTER_NUMBER_OF_OBSERVATIONS 6
#define DEFAULT_NUMBER_OF_DERIVATIVES 5

namespace hratc2017
{

class Coil
{
public:
  Coil(std::string frame_id, float sample_time = DEFAULT_DERIVATIVE_SAMPLE_TIME,
       float low_threshold = DEFAULT_LOW_COIL_SIGNAL_THRESHOLD,
       float high_threshold = DEFAULT_HIGH_COIL_SIGNAL_THRESHOLD,
       int number_of_observations =
           DEFAULT_COIL_SIGNAL_FILTER_NUMBER_OF_OBSERVATIONS);
  virtual ~Coil();
  std::string getFrameId() const;
  float getValue() const;
  float getDerivedValue() const;
  float getSampleTime() const;
  void setSampleTime(double sample_time);
  void setLowThreshold(double low_threshold);
  void setHighThreshold(double high_threshold);
  void setNumberOfObservations(int number_of_observations);
  void setNumberOfDerivatives(int number_of_derivatives);
  bool isLow() const;
  bool isHigh() const;
  void calculateDerivative();
  std::string str() const;
  const char* c_str() const;
  void operator=(float value);

private:
  std::string frame_id_;
  float value_;
  float last_value_;
  float sample_time_;
  float low_threshold_;
  float high_threshold_;
  int number_of_observations_;
  int number_of_derivatives_;
  std::list<float> samples_;
  std::list<float> derived_samples_;
};
}

#endif /* _HRATC2017_SENSORS_COIL_H_ */
