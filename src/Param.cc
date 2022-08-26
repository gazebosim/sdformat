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
#include <locale>
#include <sstream>
#include <string>

#include <locale.h>
#include <math.h>

#include "sdf/Assert.hh"
#include "sdf/Param.hh"
#include "sdf/Types.hh"

using namespace sdf;

// For some locale, the decimal separator is not a point, but a
// comma. To avoid that the SDF parsing is influenced by the current
// global C or C++ locale, we define a custom std::stringstream variant
// that always uses the std::locale::classic() locale.
// See issues https://github.com/osrf/sdformat/issues/60
// and https://github.com/osrf/sdformat/issues/207 for more details.
namespace sdf
{
  inline namespace SDF_VERSION_NAMESPACE {
  class StringStreamClassicLocale : public std::stringstream
  {
    public: explicit StringStreamClassicLocale()
    {
      this->imbue(std::locale::classic());
    }

    public: explicit StringStreamClassicLocale(const std::string& str)
      : std::stringstream(str)
    {
      this->imbue(std::locale::classic());
    }
  };
  }
}

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

/////////////////////////////////////////////////
Param &Param::operator=(const Param &_param)
{
  this->dataPtr->value = _param.dataPtr->value;
  this->dataPtr->defaultValue  = _param.dataPtr->defaultValue;
  this->dataPtr->set  = _param.dataPtr->set;
  return *this;
}

//////////////////////////////////////////////////
bool Param::GetAny(std::any &_anyVal) const
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
  else if (this->IsType<gz::math::Color>())
  {
    gz::math::Color ret;
    if (!this->Get<gz::math::Color>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<gz::math::Vector3d>())
  {
    gz::math::Vector3d ret;
    if (!this->Get<gz::math::Vector3d>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<gz::math::Vector2i>())
  {
    gz::math::Vector2i ret;
    if (!this->Get<gz::math::Vector2i>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<gz::math::Vector2d>())
  {
    gz::math::Vector2d ret;
    if (!this->Get<gz::math::Vector2d>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<gz::math::Pose3d>())
  {
    gz::math::Pose3d ret;
    if (!this->Get<gz::math::Pose3d>(ret))
    {
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<gz::math::Quaterniond>())
  {
    gz::math::Quaterniond ret;
    if (!this->Get<gz::math::Quaterniond>(ret))
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
      std::any newValue = this->dataPtr->updateFunc();
      std::visit([&](auto &&arg)
        {
          using T = std::decay_t<decltype(arg)>;
          arg = std::any_cast<T>(newValue);
        }, this->dataPtr->value);
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
  StringStreamClassicLocale ss;

  ss << ParamStreamer{ this->dataPtr->value };
  return ss.str();
}

//////////////////////////////////////////////////
std::string Param::GetDefaultAsString() const
{
  StringStreamClassicLocale ss;

  ss << ParamStreamer{ this->dataPtr->defaultValue };
  return ss.str();
}

//////////////////////////////////////////////////
bool Param::ValueFromString(const std::string &_value)
{
  return this->dataPtr->ValueFromStringImpl(this->dataPtr->typeName,
                                           _value,
                                           this->dataPtr->value);
}

//////////////////////////////////////////////////
bool ParamPrivate::ValueFromStringImpl(const std::string &_typeName,
                                       const std::string &_valueStr,
                                       ParamVariant &_valueToSet) const
{
  // Under some circumstances, latin locales (es_ES or pt_BR) will return a
  // comma for decimal position instead of a dot, making the conversion
  // to fail. See bug #60 for more information. Force to use always C
  setlocale(LC_NUMERIC, "C");

  std::string tmp(_valueStr);
  std::string lowerTmp = lowercase(_valueStr);

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
    // stof and stod for scalar floating point values.
    int numericBase = 10;
    if (isHex)
    {
      numericBase = 16;
    }

    if (_typeName == "bool")
    {
      if (lowerTmp == "true" || lowerTmp == "1")
      {
        _valueToSet = true;
      }
      else if (lowerTmp == "false" || lowerTmp == "0")
      {
        _valueToSet = false;
      }
      else
      {
        sdferr << "Invalid boolean value\n";
        return false;
      }
    }
    else if (_typeName == "char")
    {
      _valueToSet = tmp[0];
    }
    else if (_typeName == "std::string" ||
             _typeName == "string")
    {
      _valueToSet = tmp;
    }
    else if (_typeName == "int")
    {
      _valueToSet = std::stoi(tmp, nullptr, numericBase);
    }
    else if (_typeName == "uint64_t")
    {
      StringStreamClassicLocale ss(tmp);
      std::uint64_t u64tmp;

      ss >> u64tmp;
      _valueToSet = u64tmp;
    }
    else if (_typeName == "unsigned int")
    {
      _valueToSet = static_cast<unsigned int>(
          std::stoul(tmp, nullptr, numericBase));
    }
    else if (_typeName == "double")
    {
      _valueToSet = std::stod(tmp);
    }
    else if (_typeName == "float")
    {
      _valueToSet = std::stof(tmp);
    }
    else if (_typeName == "sdf::Time" ||
             _typeName == "time")
    {
      StringStreamClassicLocale ss(tmp);
      sdf::Time timetmp;

      ss >> timetmp;
      _valueToSet = timetmp;
    }
    else if (_typeName == "gz::math::Angle" ||
             _typeName == "angle")
    {
      StringStreamClassicLocale ss(tmp);
      gz::math::Angle angletmp;

      ss >> angletmp;
      _valueToSet = angletmp;
    }
    else if (_typeName == "gz::math::Color" ||
             _typeName == "color")
    {
      StringStreamClassicLocale ss(tmp);
      gz::math::Color colortmp;

      ss >> colortmp;
      _valueToSet = colortmp;
    }
    else if (_typeName == "gz::math::Vector2i" ||
             _typeName == "vector2i")
    {
      StringStreamClassicLocale ss(tmp);
      gz::math::Vector2i vectmp;

      ss >> vectmp;
      _valueToSet = vectmp;
    }
    else if (_typeName == "gz::math::Vector2d" ||
             _typeName == "vector2d")
    {
      StringStreamClassicLocale ss(tmp);
      gz::math::Vector2d vectmp;

      ss >> vectmp;
      _valueToSet = vectmp;
    }
    else if (_typeName == "gz::math::Vector3d" ||
             _typeName == "vector3")
    {
      StringStreamClassicLocale ss(tmp);
      gz::math::Vector3d vectmp;

      ss >> vectmp;
      _valueToSet = vectmp;
    }
    else if (_typeName == "gz::math::Pose3d" ||
             _typeName == "pose" ||
             _typeName == "Pose")
    {
      StringStreamClassicLocale ss(tmp);
      gz::math::Pose3d posetmp;

      ss >> posetmp;
      _valueToSet = posetmp;
    }
    else if (_typeName == "gz::math::Quaterniond" ||
             _typeName == "quaternion")
    {
      StringStreamClassicLocale ss(tmp);
      gz::math::Quaterniond quattmp;

      ss >> quattmp;
      _valueToSet = quattmp;
    }
    else
    {
      sdferr << "Unknown parameter type[" << _typeName << "]\n";
      return false;
    }
  }
  // Catch invalid argument exception from std::stoi/stoul/stod/stof
  catch(std::invalid_argument &)
  {
    sdferr << "Invalid argument. Unable to set value ["
           << _valueStr << " ] for key["
           << this->key << "].\n";
    return false;
  }
  // Catch out of range exception from std::stoi/stoul/stod/stof
  catch(std::out_of_range &)
  {
    sdferr << "Out of range. Unable to set value ["
           << _valueStr << " ] for key["
           << this->key << "].\n";
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool Param::SetFromString(const std::string &_value)
{
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
  ParamPtr clone(new Param(this->dataPtr->key, this->dataPtr->typeName,
                            this->GetAsString(), this->dataPtr->required,
                            this->dataPtr->description));
  clone->dataPtr->set = this->dataPtr->set;
  return clone;
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
bool Param::GetSet() const
{
  return this->dataPtr->set;
}
