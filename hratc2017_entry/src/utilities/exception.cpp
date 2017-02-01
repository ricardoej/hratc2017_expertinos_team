/**
 *  This source file implements the Exception helper class.
 *
 *  Version: 1.4.0
 *  Created on: 21/05/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/exception.h"

namespace utilities
{

/**
 * @brief Exception::Exception
 * @param message
 */
Exception::Exception(std::string message)
  : message_(message.c_str())
{}

/**
 * @brief Exception::Exception
 * @param message
 */
Exception::Exception(const char *message)
  : message_(message)
{}

/**
 * @brief Exception::what
 * @return
 */
const char *Exception::what() const throw()
{
  return message_;
}

}
