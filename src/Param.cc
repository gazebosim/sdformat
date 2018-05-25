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

#include <algorithm>
#include <cstdint>
#include <sstream>
#include <string>

#include <locale.h>
#include <math.h>

#include "sdf/Assert.hh"
#include "sdf/Param.hh"
#include "sdf/Types.hh"

using namespace sdf;

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

  SDF_ASSERT(this->ValueFromString(_default), "Invalid parameter");
  this->dataPtr->defaultValue = this->dataPtr->value;
}

//////////////////////////////////////////////////
Param::~Param()
{
}

//////////////////////////////////////////////////
bool Param::GetAny(boost::any &_anyVal) const
{
  if (this->IsType<int>())
  {
    int ret = 0;
    if (!this->Get<int>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<std::uint64_t>())
  {
    uint64_t ret = 0;
    if (!this->Get<std::uint64_t>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<double>())
  {
    double ret = 0;
    if (!this->Get<double>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<float>())
  {
    float ret = 0;
    if (!this->Get<float>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<bool>())
  {
    bool ret = false;
    if (!this->Get<bool>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<std::string>())
  {
    std::string ret;
    if (!this->Get<std::string>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<unsigned int>())
  {
    unsigned int ret = 0;
    if (!this->Get<unsigned int>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<char>())
  {
    char ret = 0;
    if (!this->Get<char>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<sdf::Time>())
  {
    sdf::Time ret;
    if (!this->Get<sdf::Time>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<ignition::math::Color>())
  {
    ignition::math::Color ret;
    if (!this->Get<ignition::math::Color>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<ignition::math::Vector3d>())
  {
    ignition::math::Vector3d ret;
    if (!this->Get<ignition::math::Vector3d>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<ignition::math::Vector2i>())
  {
    ignition::math::Vector2i ret;
    if (!this->Get<ignition::math::Vector2i>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<ignition::math::Vector2d>())
  {
    ignition::math::Vector2d ret;
    if (!this->Get<ignition::math::Vector2d>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<ignition::math::Pose3d>())
  {
    ignition::math::Pose3d ret;
    if (!this->Get<ignition::math::Pose3d>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<ignition::math::Quaterniond>())
  {
    ignition::math::Quaterniond ret;
    if (!this->Get<ignition::math::Quaterniond>(ret))
    {
      return false;
    }
    _anyVal = ret;
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
    catch(...)
    {
      sdferr << "Unable to set value using Update for key["
             << this->dataPtr->key << "]\n";
    }
  }
}

//////////////////////////////////////////////////
std::string Param::GetAsString() const
{
  std::stringstream ss;

  ss << this->dataPtr->value;
  return ss.str();
}

//////////////////////////////////////////////////
std::string Param::GetDefaultAsString() const
{
  std::stringstream ss;

  ss << this->dataPtr->defaultValue;
  return ss.str();
}

//////////////////////////////////////////////////
bool Param::ValueFromString(const std::string &_value)
{
  std::string tmp(_value);
  std::string lowerTmp(_value);
  std::transform(lowerTmp.begin(), lowerTmp.end(), lowerTmp.begin(),
      [](unsigned char c)
      {
        return static_cast<unsigned char>(std::tolower(c));
      });

  // "true" and "false" doesn't work properly
  if (lowerTmp == "true")
  {
    tmp = "1";
  }
  else if (lowerTmp == "false")
  {
    tmp = "0";
  }

  bool isHex = lowerTmp.compare(0, 2, "0x") == 0;

  try
  {
    // Try to use stoi and stoul for integers, and
    // stof and stod for floating point values.
    int numericBase = 10;
    if (isHex)
    {
      numericBase = 16;
    }

    if (this->dataPtr->typeName == "bool")
    {
      if (lowerTmp == "true" || lowerTmp == "1")
      {
        this->dataPtr->value = true;
      }
      else if (lowerTmp == "false" || lowerTmp == "0")
      {
        this->dataPtr->value = false;
      }
      else
      {
        sdferr << "Invalid boolean value\n";
        return false;
      }
    }
    else if (this->dataPtr->typeName == "char")
    {
      this->dataPtr->value = tmp[0];
    }
    else if (this->dataPtr->typeName == "std::string" ||
             this->dataPtr->typeName == "string")
    {
      this->dataPtr->value = tmp;
    }
    else if (this->dataPtr->typeName == "int")
    {
      this->dataPtr->value = std::stoi(tmp, nullptr, numericBase);
    }
    else if (this->dataPtr->typeName == "uint64_t")
    {
      std::stringstream ss(tmp);
      std::uint64_t u64tmp;

      ss >> u64tmp;
      this->dataPtr->value = u64tmp;
    }
    else if (this->dataPtr->typeName == "unsigned int")
    {
      this->dataPtr->value = static_cast<unsigned int>(
          std::stoul(tmp, nullptr, numericBase));
    }
    else if (this->dataPtr->typeName == "double")
    {
      this->dataPtr->value = std::stod(tmp);
    }
    else if (this->dataPtr->typeName == "float")
    {
      this->dataPtr->value = std::stof(tmp);
    }
    else if (this->dataPtr->typeName == "sdf::Time" ||
             this->dataPtr->typeName == "time")
    {
      std::stringstream ss(tmp);
      sdf::Time timetmp;

      ss >> timetmp;
      this->dataPtr->value = timetmp;
    }
    else if (this->dataPtr->typeName == "ignition::math::Color" ||
             this->dataPtr->typeName == "color")
    {
      std::stringstream ss(tmp);
      ignition::math::Color colortmp;

      ss >> colortmp;
      this->dataPtr->value = colortmp;
    }
    else if (this->dataPtr->typeName == "ignition::math::Vector2i" ||
             this->dataPtr->typeName == "vector2i")
    {
      std::stringstream ss(tmp);
      ignition::math::Vector2i vectmp;

      ss >> vectmp;
      this->dataPtr->value = vectmp;
    }
    else if (this->dataPtr->typeName == "ignition::math::Vector2d" ||
             this->dataPtr->typeName == "vector2d")
    {
      std::stringstream ss(tmp);
      ignition::math::Vector2d vectmp;

      ss >> vectmp;
      this->dataPtr->value = vectmp;
    }
    else if (this->dataPtr->typeName == "ignition::math::Vector3d" ||
             this->dataPtr->typeName == "vector3")
    {
      std::stringstream ss(tmp);
      ignition::math::Vector3d vectmp;

      ss >> vectmp;
      this->dataPtr->value = vectmp;
    }
    else if (this->dataPtr->typeName == "ignition::math::Pose3d" ||
             this->dataPtr->typeName == "pose" ||
             this->dataPtr->typeName == "Pose")
    {
      std::stringstream ss(tmp);
      ignition::math::Pose3d posetmp;

      ss >> posetmp;
      this->dataPtr->value = posetmp;
    }
    else if (this->dataPtr->typeName == "ignition::math::Quaterniond" ||
             this->dataPtr->typeName == "quaternion")
    {
      std::stringstream ss(tmp);
      ignition::math::Quaterniond quattmp;

      ss >> quattmp;
      this->dataPtr->value = quattmp;
    }
    else
    {
      sdferr << "Unknown parameter type[" << this->dataPtr->typeName << "]\n";
      return false;
    }
  }
  // Catch invalid argument exception from std::stoi/stoul/stod/stof
  catch(std::invalid_argument &)
  {
    sdferr << "Invalid argument. Unable to set value ["
           << _value << " ] for key["
           << this->dataPtr->key << "].\n";
    return false;
  }
  // Catch out of range exception from std::stoi/stoul/stod/stof
  catch(std::out_of_range &)
  {
    sdferr << "Out of range. Unable to set value ["
           << _value << " ] for key["
           << this->dataPtr->key << "].\n";
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool Param::SetFromString(const std::string &_value)
{
  // Under some circumstances, latin locales (es_ES or pt_BR) will return a
  // comma for decimal position instead of a dot, making the conversion
  // to fail. See bug #60 for more information. Force to use always C
  setlocale(LC_NUMERIC, "C");

  std::string str = sdf::trim(_value.c_str());

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

  if (!this->ValueFromString(str))
  {
    return false;
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
