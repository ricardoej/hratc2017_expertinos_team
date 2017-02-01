/**
 *  This header file defines the EnumConverter abstract class.
 *
 *  Version: 1.4.0
 *  Created on: 05/10/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_ENUM_CONVERTER_H_
#define _UTILITIES_ENUM_CONVERTER_H_

#include <string>
#include <vector>

namespace utilities
{

template <typename E> class EnumConverter
{
public:
  virtual ~EnumConverter();
  E getEnumerated() const;
  virtual E getEnumerated(int id) const = 0;
  virtual E getEnumerated(std::string nome) const = 0;
  int getId() const;
  virtual int getId(std::string nome) const = 0;
  virtual int getId(E enumerated) const = 0;
  std::string str() const;
  virtual std::string str(E enumerated) const = 0;
  const char* c_str() const;
  virtual const char* c_str(E enumerated) const = 0;
  void operator=(int id);
  void operator=(std::string nome);
  void operator=(E enumerated);

protected:
  EnumConverter(E enumerated);

private:
  E enumerated_;
};

/**
 * The implementation is used inside the header file because of
 * the use of templates. This is the most recomended usage.
 * Benefits: General usage.
 */

/**
 *
 */
template <typename E>
EnumConverter<E>::EnumConverter(E enumerated)
    : enumerated_(enumerated)
{
}

/**
 *
 */
template <typename E> EnumConverter<E>::~EnumConverter() {}

/**
 *
 */
template <typename E> E EnumConverter<E>::getEnumerated() const
{
  return enumerated_;
}

/**
 *
 */
template <typename E> int EnumConverter<E>::getId() const
{
  return getId(enumerated_);
}

/**
 *
 */
template <typename E> std::string EnumConverter<E>::str() const
{
  return str(enumerated_);
}

/**
 *
 */
template <typename E> const char* EnumConverter<E>::c_str() const
{
  return str().c_str();
}

/**
 *
 */
template <typename E> void EnumConverter<E>::operator=(int id)
{
  enumerated_ = getEnumerated(id);
}

/**
 *
 */
template <typename E> void EnumConverter<E>::operator=(std::string nome)
{
  enumerated_ = getEnumerated(nome);
}

/**
 *
 */
template <typename E> void EnumConverter<E>::operator=(E enumerated)
{
  enumerated_ = enumerated;
}
}

#endif // _UTILITIES_ENUM_CONVERTER_H_
