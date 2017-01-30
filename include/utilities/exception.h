/**
 * This header file defines the MathManipulator helper class.
 *
 *  Version: 1.4.0
 *  Created on: 21/05/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_EXCEPTION_H_
#define _UTILITIES_EXCEPTION_H_

#include <string>
#include <exception>

namespace utilities
{

class Exception : public std::exception
{
public:
  Exception(std::string message);
  Exception(const char* message);
  virtual const char* what() const throw();

private:
  const char* message_;
};
}

#endif // _UTILITIES_EXCEPTION_H_
