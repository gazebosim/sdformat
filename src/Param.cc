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
/* Desc: Parameter class
 * Author: Nate Koenig
 * Date: 14 Aug 2008
 */

#include <math.h>
#include "sdf/Param.hh"

using namespace sdf;

class string_set : public boost::static_visitor<>
{
  public: string_set(const std::string &_value)
          {this->value = _value;}

  public: template <typename T>
          void operator()(T & _operand) const
          {
            _operand = boost::lexical_cast<T>(this->value);
          }
  public: std::string value;
};

class any_set : public boost::static_visitor<>
{
  public: any_set(const boost::any &_value)
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
{
  this->key = _key;
  this->required = _required;
  this->typeName = _typeName;
  this->description = _description;
  this->set = false;

  if (this->typeName == "bool")
    this->Init<bool>(_default);
  else if (this->typeName == "int")
    this->Init<int>(_default);
  else if (this->typeName == "unsigned int")
    this->Init<unsigned int>(_default);
  else if (this->typeName == "double")
    this->Init<double>(_default);
  else if (this->typeName == "float")
    this->Init<float>(_default);
  else if (this->typeName == "char")
    this->Init<char>(_default);
  else if (this->typeName == "std::string" ||
      this->typeName == "string")
    this->Init<std::string>(_default);
  else if (this->typeName == "sdf::Vector2i" ||
      this->typeName == "vector2i")
    this->Init<sdf::Vector2i>(_default);
  else if (this->typeName == "sdf::Vector2d" ||
      this->typeName == "vector2d")
    this->Init<sdf::Vector2d>(_default);
  else if (this->typeName == "sdf::Vector3" ||
       this->typeName == "vector3")
    this->Init<sdf::Vector3>(_default);
  else if (this->typeName == "sdf::Pose" ||
      this->typeName == "pose" || this->typeName == "Pose")
    this->Init<sdf::Pose>(_default);
  else if (this->typeName == "sdf::Quaternion" ||
      this->typeName == "quaternion")
    this->Init<sdf::Quaternion>(_default);
  else if (this->typeName == "sdf::Time" ||
      this->typeName == "time")
    this->Init<sdf::Time>(_default);
  else if (this->typeName == "sdf::Color" ||
      this->typeName == "color")
    this->Init<sdf::Color>(_default);
  else
    sdferr << "Unknown parameter type[" << this->typeName << "]\n";
}

//////////////////////////////////////////////////
Param::~Param()
{
}

//////////////////////////////////////////////////
void Param::Update()
{
  if (this->updateFunc)
  {
    try
    {
      boost::apply_visitor(any_set(this->updateFunc()), this->value);
    }
    catch(boost::bad_lexical_cast &e)
    {
      sdferr << "Unable to set value using Update for key["
        << this->key << "]\n";
    }
  }
}

//////////////////////////////////////////////////
std::string Param::GetAsString() const
{
  return boost::lexical_cast<std::string>(this->value);
}

//////////////////////////////////////////////////
std::string Param::GetDefaultAsString() const
{
  return boost::lexical_cast<std::string>(this->defaultValue);
}

//////////////////////////////////////////////////
bool Param::SetFromString(const std::string &_value)
{
  std::string str = _value;
  boost::trim(str);

  if (str.empty() && this->required)
  {
    sdferr << "Empty string used when setting a required parameter. Key["
      << this->GetKey() << "]\n";
    return false;
  }
  else if (str.empty())
  {
    this->value = this->defaultValue;
    return true;
  }

  std::string tmp(str);
  std::string lowerTmp(str);
  boost::to_lower(lowerTmp);

  // "true" and "false" doesn't work properly
  if (lowerTmp == "true")
    tmp = "1";
  else if (lowerTmp == "false")
    tmp = "0";

  try
  {
    boost::apply_visitor(string_set(tmp), this->value);
  }
  catch(boost::bad_lexical_cast &e)
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
        << "] for key[" << this->key << "]\n";
      return false;
    }
  }

  this->set = true;
  return this->set;
}

//////////////////////////////////////////////////
void Param::Reset()
{
  this->value = this->defaultValue;
  this->set = false;
}

//////////////////////////////////////////////////
boost::shared_ptr<Param> Param::Clone() const
{
  return boost::shared_ptr<Param>(new Param(this->key, this->typeName,
        this->GetAsString(), this->required, this->description));
}

//////////////////////////////////////////////////
const std::type_info &Param::GetType() const
{
  return this->value.type();
}

//////////////////////////////////////////////////
const std::string &Param::GetTypeName() const
{
  return this->typeName;
}

/////////////////////////////////////////////////
void Param::SetDescription(const std::string &_desc)
{
  this->description = _desc;
}

/////////////////////////////////////////////////
std::string Param::GetDescription() const
{
  return this->description;
}
