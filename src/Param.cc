/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <math.h>
#include <locale.h>
#include <boost/algorithm/string.hpp>
#include "sdf/Param.hh"

using namespace sdf;

class string_set : public boost::static_visitor<>
{
  public: explicit string_set(const std::string &_value)
    : value(_value)
          {}

  public: template <typename T>
          void operator()(T & _operand) const
          {
            _operand = boost::lexical_cast<T>(this->value);
          }
  public: std::string value;
};

class any_set : public boost::static_visitor<>
{
  public: explicit any_set(const boost::any &_value)
          {this->value = _value;}

  public: template <typename T>
          void operator()(T & _operand) const
          {
            _operand = boost::any_cast<T>(this->value);
          }
  public: boost::any value;
};

//////////////////////////////////////////////////
Param::Param(const std::string &_key, const std::string &_typeName,
             const std::string &_default, bool _required,
             const std::string &_description)
  : dataPtr(new ParamPrivate)
{
  this->dataPtr->key = _key;
  this->dataPtr->required = _required;
  this->dataPtr->typeName = _typeName;
  this->dataPtr->description = _description;
  this->dataPtr->set = false;

  if (this->dataPtr->typeName == "bool")
    this->Init<bool>(_default);
  else if (this->dataPtr->typeName == "int")
    this->Init<int>(_default);
  else if (this->dataPtr->typeName == "unsigned int")
    this->Init<unsigned int>(_default);
  else if (this->dataPtr->typeName == "uint64_t")
    this->Init<uint64_t>(_default);
  else if (this->dataPtr->typeName == "double")
    this->Init<double>(_default);
  else if (this->dataPtr->typeName == "float")
    this->Init<float>(_default);
  else if (this->dataPtr->typeName == "char")
    this->Init<char>(_default);
  else if (this->dataPtr->typeName == "std::string" ||
           this->dataPtr->typeName == "string")
  {
    this->Init<std::string>(_default);
  }
  else if (this->dataPtr->typeName == "sdf::Time" ||
           this->dataPtr->typeName == "time")
  {
    this->Init<sdf::Time>(_default);
  }
  else if (this->dataPtr->typeName == "sdf::Color" ||
           this->dataPtr->typeName == "color")
  {
    this->Init<sdf::Color>(_default);
  }
  else if (this->dataPtr->typeName == "ignition::math::Vector2i" ||
           this->dataPtr->typeName == "vector2i")
  {
    this->Init<ignition::math::Vector2i>(_default);
  }
  else if (this->dataPtr->typeName == "ignition::math::Vector2d" ||
           this->dataPtr->typeName == "vector2d")
  {
    this->Init<ignition::math::Vector2d>(_default);
  }
  else if (this->dataPtr->typeName == "ignition::math::Vector3d" ||
           this->dataPtr->typeName == "vector3")
  {
    this->Init<ignition::math::Vector3d>(_default);
  }
  else if (this->dataPtr->typeName == "ignition::math::Pose3d" ||
           this->dataPtr->typeName == "pose" ||
           this->dataPtr->typeName == "Pose")
  {
    this->Init<ignition::math::Pose3d>(_default);
  }
  else if (this->dataPtr->typeName == "ignition::math::Quaterniond" ||
           this->dataPtr->typeName == "quaternion")
  {
    this->Init<ignition::math::Quaterniond>(_default);
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  }
  /// \deprecated The following sdf::<types> are deprecated
  else if (this->dataPtr->typeName == "sdf::Vector2i" ||
           this->dataPtr->typeName == "vector2i")
  {
    sdferr << "sdf::Vector2i is deprecated. Use ignition::math::Vector2i\n";
    this->Init<sdf::Vector2i>(_default);
  }
  else if (this->dataPtr->typeName == "sdf::Vector2d" ||
           this->dataPtr->typeName == "vector2d")
  {
    sdferr << "sdf::Vector2d is deprecated. Use ignition::math::Vector2d\n";
    this->Init<sdf::Vector2d>(_default);
  }
  else if (this->dataPtr->typeName == "sdf::Vector3" ||
           this->dataPtr->typeName == "vector3")
  {
    sdferr << "sdf::Vector3 is deprecated. Use ignition::math::Vector3d\n";
    this->Init<sdf::Vector3>(_default);
  }
  else if (this->dataPtr->typeName == "sdf::Pose" ||
           this->dataPtr->typeName == "pose" ||
           this->dataPtr->typeName == "Pose")
  {
    sdferr << "sdf::Pose is deprecated. Use ignition::math::Pose3d\n";
    this->Init<sdf::Pose>(_default);
  }
  else if (this->dataPtr->typeName == "sdf::Quaternion" ||
           this->dataPtr->typeName == "quaternion")
  {
    sdferr << "sdf::Quaternion is deprecated. "
           << "Use ignition::math::Quaterniond\n";
    this->Init<sdf::Quaternion>(_default);
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif
  }
  else
    sdferr << "Unknown parameter type[" << this->dataPtr->typeName << "]\n";
}

//////////////////////////////////////////////////
Param::~Param()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

//////////////////////////////////////////////////
bool Param::GetAny(boost::any &_anyVal) const
{
  if (this->IsType<int>())
  {
    int ret = 0;
    if (!this->Get<int>(ret))
      return false;
    _anyVal = ret;
  }
  else if (this->IsType<uint64_t>())
  {
    uint64_t ret = 0;
    if (!this->Get<uint64_t>(ret))
      return false;
    _anyVal = ret;
  }
  else if (this->IsType<double>())
  {
    double ret = 0;
    if (!this->Get<double>(ret))
      return false;
    _anyVal = ret;
  }
  else if (this->IsType<float>())
  {
    float ret = 0;
    if (!this->Get<float>(ret))
      return false;
    _anyVal = ret;
  }
  else if (this->IsType<bool>())
  {
    bool ret = false;
    if (!this->Get<bool>(ret))
      return false;
    _anyVal = ret;
  }
  else if (this->IsType<std::string>())
  {
    std::string ret;
    if (!this->Get<std::string>(ret))
      return false;
    _anyVal = ret;
  }
  else if (this->IsType<unsigned int>())
  {
    unsigned int ret = 0;
    if (!this->Get<unsigned int>(ret))
      return false;
    _anyVal = ret;
  }
  else if (this->IsType<char>())
  {
    char ret = 0;
    if (!this->Get<char>(ret))
      return false;
    _anyVal = ret;
  }
  else if (this->IsType<sdf::Time>())
  {
    sdf::Time ret;
    if (!this->Get<sdf::Time>(ret))
      return false;
    _anyVal = ret;
  }
  else if (this->IsType<sdf::Color>())
  {
    sdf::Color ret;
    if (!this->Get<sdf::Color>(ret))
      return false;
    _anyVal = ret;
  }
  else if (this->IsType<ignition::math::Vector3d>())
  {
    ignition::math::Vector3d ret;
    if (!this->Get<ignition::math::Vector3d>(ret))
      return false;
    _anyVal = ret;
  }
  else if (this->IsType<ignition::math::Vector2i>())
  {
    ignition::math::Vector2i ret;
    if (!this->Get<ignition::math::Vector2i>(ret))
      return false;
    _anyVal = ret;
  }
  else if (this->IsType<ignition::math::Vector2d>())
  {
    ignition::math::Vector2d ret;
    if (!this->Get<ignition::math::Vector2d>(ret))
      return false;
    _anyVal = ret;
  }
  else if (this->IsType<ignition::math::Pose3d>())
  {
    ignition::math::Pose3d ret;
    if (!this->Get<ignition::math::Pose3d>(ret))
      return false;
    _anyVal = ret;
  }
  else if (this->IsType<ignition::math::Quaterniond>())
  {
    ignition::math::Quaterniond ret;
    if (!this->Get<ignition::math::Quaterniond>(ret))
      return false;
    _anyVal = ret;
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  }
  /// \deprecated The follow sdf Types are deprecated
  else if (this->IsType<sdf::Vector3>())
  {
    sdferr << "sdf::Vector3 is deprecated. Use ignition::math::Vector3d\n";
    sdf::Vector3 ret;
    if (!this->Get<sdf::Vector3>(ret))
      return false;
    _anyVal = ret;
  }
  else if (this->IsType<sdf::Vector2i>())
  {
    sdferr << "sdf::Vector2i is deprecated. Use ignition::math::Vector2i\n";
    sdf::Vector2i ret;
    if (!this->Get<sdf::Vector2i>(ret))
      return false;
    _anyVal = ret;
  }
  else if (this->IsType<sdf::Vector2d>())
  {
    sdferr << "sdf::Vector2d is deprecated. Use ignition::math::Vector2d\n";
    sdf::Vector2d ret;
    if (!this->Get<sdf::Vector2d>(ret))
      return false;
    _anyVal = ret;
  }
  else if (this->IsType<sdf::Pose>())
  {
    sdferr << "sdf::Pose is deprecated. Use ignition::math::Pose3d\n";
    sdf::Pose ret;
    if (!this->Get<sdf::Pose>(ret))
      return false;
    _anyVal = ret;
  }
  else if (this->IsType<sdf::Quaternion>())
  {
    sdferr << "sdf::Quaternion is deprecated. "
           << "Use ignition::math::Quaterniond\n";
    sdf::Quaternion ret;
    if (!this->Get<sdf::Quaternion>(ret))
      return false;
    _anyVal = ret;
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif
  }
  else
  {
    sdferr << "Type of parameter not known: [" << this->GetTypeName() << "]\n";
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
void Param::Update()
{
  if (this->dataPtr->updateFunc)
  {
    try
    {
      boost::apply_visitor(any_set(this->dataPtr->updateFunc()),
      this->dataPtr->value);
    }
    catch(boost::bad_lexical_cast &/*e*/)
    {
      sdferr << "Unable to set value using Update for key["
        << this->dataPtr->key << "]\n";
    }
  }
}

//////////////////////////////////////////////////
std::string Param::GetAsString() const
{
  return boost::lexical_cast<std::string>(this->dataPtr->value);
}

//////////////////////////////////////////////////
std::string Param::GetDefaultAsString() const
{
  return boost::lexical_cast<std::string>(this->dataPtr->defaultValue);
}

//////////////////////////////////////////////////
bool Param::SetFromString(const std::string &_value)
{
  // Under some circumstances, latin locales (es_ES or pt_BR) will return a
  // comma for decimal position instead of a dot, making the conversion
  // to fail. See bug #60 for more information. Force to use always C
  setlocale(LC_NUMERIC, "C");

  std::string str = _value;
  boost::trim(str);

  if (str.empty() && this->dataPtr->required)
  {
    sdferr << "Empty string used when setting a required parameter. Key["
      << this->GetKey() << "]\n";
    return false;
  }
  else if (str.empty())
  {
    this->dataPtr->value = this->dataPtr->defaultValue;
    return true;
  }

  std::string tmp(str);
  std::string lowerTmp(str);
  std::transform(lowerTmp.begin(), lowerTmp.end(), lowerTmp.begin(), ::tolower);

  // "true" and "false" doesn't work properly
  if (lowerTmp == "true")
    tmp = "1";
  else if (lowerTmp == "false")
    tmp = "0";

  bool isHex = tmp.compare(0, 2, "0x") == 0;

  try
  {
    // Try to use stoi and stoul for integers, and
    // stof and stod for floating point values.
    // Use boost lexical cast as a last resort.
    int numericBase = 10;
    if (isHex)
        numericBase = 16;

    if (this->dataPtr->typeName == "int")
      this->dataPtr->value = std::stoi(tmp, NULL, numericBase);
    else if (this->dataPtr->typeName == "unsigned int")
    {
      this->dataPtr->value = static_cast<unsigned int>(
          std::stoul(tmp, NULL, numericBase));
    }
    else if (this->dataPtr->typeName == "double")
      this->dataPtr->value = std::stod(tmp);
    else if (this->dataPtr->typeName == "float")
      this->dataPtr->value = std::stof(tmp);
    else
      boost::apply_visitor(string_set(tmp), this->dataPtr->value);
  }

  // Catch invalid argument exception from std::stoi/stoul/stod/stof
  catch(std::invalid_argument &)
  {
    sdferr << "Invalid argument. Unable to set value ["
      << str << " ] for key["
      << this->dataPtr->key << "].\n";
    return false;
  }
  // Catch out of range exception from std::stoi/stoul/stod/stof
  catch(std::out_of_range &)
  {
    sdferr << "Out of range. Unable to set value ["
      << str << " ] for key["
      << this->dataPtr->key << "].\n";
    return false;
  }
  // Catch boost lexical cast exceptions
  catch(boost::bad_lexical_cast &)
  {
    if (str == "inf" || str == "-inf")
    {
      // in this case, the parser complains, but seems to assign the
      // right values
      sdfmsg << "INFO [sdf::Param]: boost throws when lexical casting "
        << "inf's, but the values are usually passed through correctly\n";
    }
    else
    {
      sdferr << "Unable to set value [" <<  str
        << "] for key[" << this->dataPtr->key << "]\n";
      return false;
    }
  }

  this->dataPtr->set = true;
  return this->dataPtr->set;
}

//////////////////////////////////////////////////
void Param::Reset()
{
  this->dataPtr->value = this->dataPtr->defaultValue;
  this->dataPtr->set = false;
}

//////////////////////////////////////////////////
ParamPtr Param::Clone() const
{
  return ParamPtr(new Param(this->dataPtr->key, this->dataPtr->typeName,
      this->GetAsString(), this->dataPtr->required,
      this->dataPtr->description));
}

//////////////////////////////////////////////////
const std::type_info &Param::GetType() const
{
  return this->dataPtr->value.type();
}

//////////////////////////////////////////////////
const std::string &Param::GetTypeName() const
{
  return this->dataPtr->typeName;
}

/////////////////////////////////////////////////
void Param::SetDescription(const std::string &_desc)
{
  this->dataPtr->description = _desc;
}

/////////////////////////////////////////////////
std::string Param::GetDescription() const
{
  return this->dataPtr->description;
}

/////////////////////////////////////////////////
const std::string &Param::GetKey() const
{
  return this->dataPtr->key;
}

/////////////////////////////////////////////////
bool Param::GetRequired() const
{
  return this->dataPtr->required;
}

/////////////////////////////////////////////////
Param &Param::operator=(const Param &_param)
{
  this->dataPtr->value = _param.dataPtr->value;
  this->dataPtr->defaultValue  = _param.dataPtr->defaultValue;
  return *this;
}

/////////////////////////////////////////////////
bool Param::GetSet() const
{
  return this->dataPtr->set;
}
