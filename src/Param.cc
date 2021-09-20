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
#include <cctype>
#include <cstdint>
#include <locale>
#include <sstream>
#include <string>
#include <vector>
#include <array>

#include <locale.h>
#include <math.h>

#include "sdf/Assert.hh"
#include "sdf/Param.hh"
#include "sdf/Types.hh"
#include "sdf/Element.hh"

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
  this->dataPtr->ignoreParentAttributes = false;
  this->dataPtr->defaultStrValue = _default;

  SDF_ASSERT(this->ValueFromString(_default), "Invalid parameter");
  this->dataPtr->defaultValue = this->dataPtr->value;
}

//////////////////////////////////////////////////
Param::Param(const std::string &_key, const std::string &_typeName,
             const std::string &_default, bool _required,
             const std::string &_minValue, const std::string &_maxValue,
             const std::string &_description)
    : Param(_key, _typeName, _default, _required, _description)
{
  if (!_minValue.empty())
  {
    SDF_ASSERT(
        this->dataPtr->ValueFromStringImpl(
            this->dataPtr->typeName,
            _minValue,
            this->dataPtr->minValue.emplace()),
        std::string("Invalid [min] parameter in SDFormat description of [") +
            _key + "]");
  }

  if (!_maxValue.empty())
  {
    SDF_ASSERT(
        this->dataPtr->ValueFromStringImpl(
            this->dataPtr->typeName,
            _maxValue,
            this->dataPtr->maxValue.emplace()),
        std::string("Invalid [max] parameter in SDFormat description of [") +
            _key + "]");
  }
}

//////////////////////////////////////////////////
Param::Param(const Param &_param)
    : dataPtr(std::make_unique<ParamPrivate>(*_param.dataPtr))
{
  // We don't want to copy the updateFunc
  this->dataPtr->updateFunc = nullptr;
}

//////////////////////////////////////////////////
Param::~Param()
{
}

/////////////////////////////////////////////////
Param &Param::operator=(const Param &_param)
{
  auto updateFuncCopy = this->dataPtr->updateFunc;
  *this = Param(_param);

  // Restore the update func
  this->dataPtr->updateFunc = updateFuncCopy;
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
  ss << this->dataPtr->strValue;
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
std::optional<std::string> Param::GetMinValueAsString() const
{
  if (this->dataPtr->minValue.has_value())
  {
    StringStreamClassicLocale ss;

    ss << ParamStreamer{ *this->dataPtr->minValue };
    return ss.str();
  }
  return std::nullopt;
}

//////////////////////////////////////////////////
std::optional<std::string> Param::GetMaxValueAsString() const
{
  if (this->dataPtr->maxValue.has_value())
  {
    StringStreamClassicLocale ss;

    ss << ParamStreamer{ *this->dataPtr->maxValue };
    return ss.str();
  }
  return std::nullopt;
}

//////////////////////////////////////////////////
/// \brief Helper function for Param::ValueFromString
/// \param[in] _input Input string.
/// \param[in] _key Key of the parameter, used for error message.
/// \param[out] _value This will be set with the parsed value.
/// \return True if parsing succeeded.
template <typename T>
bool ParseUsingStringStream(const std::string &_input, const std::string &_key,
                            ParamPrivate::ParamVariant &_value)
{
  StringStreamClassicLocale ss(_input);
  T _val;
  ss >> _val;
  if (ss.fail())
  {
    sdferr << "Unknown error. Unable to set value [" << _input << " ] for key["
           << _key << "]\n";
    return false;
  }
  _value = _val;
  return true;
}

//////////////////////////////////////////////////
/// \brief Helper function for Param::ValueFromString for parsing colors
/// This checks the color components specified in _input are rgb or rgba
/// (expects 3 or 4 values) and each value is between [0,1]. When only 3 values
/// are present (rgb), then alpha is set to 1.
/// \param[in] _input Input string.
/// \param[in] _key Key of the parameter, used for error message.
/// \param[out] _value This will be set with the parsed value.
/// \return True if parsing colors succeeded.
bool ParseColorUsingStringStream(const std::string &_input,
    const std::string &_key, ParamPrivate::ParamVariant &_value)
{
  StringStreamClassicLocale ss(_input);
  std::string token;
  std::vector<float> colors;
  float c;  // r,g,b,a values
  bool isValidColor = true;
  while (ss >> token)
  {
    try
    {
      c = std::stof(token);
      colors.push_back(c);
    }
    // Catch invalid argument exception from std::stof
    catch(std::invalid_argument &)
    {
      sdferr << "Invalid argument. Unable to set value ["<< token
             << "] for key [" << _key << "].\n";
      isValidColor = false;
      break;
    }
    // Catch out of range exception from std::stof
    catch(std::out_of_range &)
    {
      sdferr << "Out of range. Unable to set value [" << token
             << "] for key [" << _key << "].\n";
      isValidColor = false;
      break;
    }

    if (c < 0.0f || c > 1.0f)
    {
      isValidColor = false;
      break;
    }
  }

  size_t colorSize = colors.size();
  if (isValidColor && colorSize == 3u)
    colors.push_back(1.0f);
  else if (colorSize != 4u)
    isValidColor = false;

  if (isValidColor)
  {
    _value = ignition::math::Color(colors[0], colors[1], colors[2], colors[3]);
  }
  else
  {
    sdferr << "The value <" << _key <<
        ">" << _input << "</" << _key << "> is invalid.\n";
  }

  return isValidColor;
}

//////////////////////////////////////////////////
/// \brief Helper function for Param::ValueFromString for parsing pose. This
/// checks the pose components specified in _input are xyzrpy
/// (expects 6 values) and whether to parse the rotation values as degrees using
/// the parent element attributes.
/// \param[in] _input Input string.
/// \param[in] _key Key of the parameter, used for error message.
/// \param[in] _attributes Attributes associated to this pose.
/// \param[out] _value This will be set with the parsed value.
/// \return True if parsing pose succeeded.
bool ParsePoseUsingStringStream(const std::string &_input,
    const std::string &_key, const Param_V &_attributes,
    ParamPrivate::ParamVariant &_value)
{
  const bool defaultParseAsDegrees = false;
  bool parseAsDegrees = defaultParseAsDegrees;

  const std::string defaultRotationFormat = "euler_rpy";
  std::string rotationFormat = defaultRotationFormat;

  const std::size_t defaultDesiredSize = 6u;
  std::size_t desiredSize = defaultDesiredSize;

  std::string defaultValueStr = "0 0 0 0 0 0";

  for (const auto &p : _attributes)
  {
    const std::string key = p->GetKey();

    if (key == "degrees")
    {
      if (!p->Get<bool>(parseAsDegrees))
      {
        sdferr << "Invalid boolean value found for attribute "
            "//pose[@degrees].\n";
        return false;
      }
    }
    else if (key == "rotation_format")
    {
      rotationFormat = p->GetAsString();

      if (rotationFormat == "euler_rpy")
      {
        // Already the default, no modifications needed.
      }
      else if (rotationFormat == "quat_xyzw")
      {
        desiredSize = 7u;
        defaultValueStr = "0 0 0 0 0 0 1";
      }
      else
      {
        sdferr << "Undefined attribute //pose[@rotation_format='"
            << rotationFormat << "'], only 'euler_rpy' and 'quat_xyzw'"
            << " is supported.\n";
        return false;
      }
    }
  }

  if (rotationFormat == "quat_xyzw" && parseAsDegrees)
  {
    sdferr << "The attribute //pose[@degrees='true'] does not apply when "
        << "parsing quaternions, //pose[@rotation_format='quat_xyzw'].\n";
    return false;
  }

  std::string input = _input.empty() ? defaultValueStr : _input;
  StringStreamClassicLocale ss(input);
  std::string token;
  std::array<double, 7> values;
  std::size_t valueIndex = 0;
  double v;
  bool isValidPose = true;
  while (ss >> token)
  {
    try
    {
      v = std::stod(token);
    }
    // Catch invalid argument exception from std::stod
    catch(std::invalid_argument &)
    {
      sdferr << "Invalid argument. Unable to set value ["<< input
             << "] for key [" << _key << "].\n";
      isValidPose = false;
      break;
    }
    // Catch out of range exception from std::stod
    catch(std::out_of_range &)
    {
      sdferr << "Out of range. Unable to set value [" << token
             << "] for key [" << _key << "].\n";
      isValidPose = false;
      break;
    }

    if (!std::isfinite(v))
    {
      sdferr << "Pose values must be finite.\n";
      isValidPose = false;
      break;
    }

    if (valueIndex >= desiredSize)
    {
      sdferr << "The value for //pose[@rotation_format='" << rotationFormat
          << "'] must have " << desiredSize
          << " values, but more than that were found in '" << input << "'.\n";
      isValidPose = false;
      break;
    }

    values[valueIndex++] = v;
  }

  if (!isValidPose)
    return false;

  if (valueIndex != desiredSize)
  {
    sdferr << "The value for //pose[@rotation_format='" << rotationFormat
        << "'] must have " << desiredSize << " values, but " << valueIndex
        << " were found instead in '" << input << "'.\n";
    return false;
  }

  if (rotationFormat == "euler_rpy")
  {
    if (parseAsDegrees)
    {
      _value = ignition::math::Pose3d(values[0], values[1], values[2],
          IGN_DTOR(values[3]), IGN_DTOR(values[4]), IGN_DTOR(values[5]));
    }
    else
    {
      _value = ignition::math::Pose3d(values[0], values[1], values[2],
          values[3], values[4], values[5]);
    }
  }
  else
  {
    _value = ignition::math::Pose3d(values[0], values[1], values[2],
        values[6], values[3], values[4], values[5]);
  }

  return true;
}

//////////////////////////////////////////////////
bool Param::ValueFromString(const std::string &_value)
{
  if (!this->dataPtr->ValueFromStringImpl(
      this->dataPtr->typeName, _value, this->dataPtr->value))
    return false;

  this->dataPtr->strValue = _value;
  return true;
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
  std::string trimmed = sdf::trim(_valueStr);
  std::string tmp(trimmed);
  std::string lowerTmp = lowercase(trimmed);

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
      return ParseUsingStringStream<std::uint64_t>(tmp, this->key,
                                                   _valueToSet);
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
      return ParseUsingStringStream<sdf::Time>(tmp, this->key,
                                               _valueToSet);
    }
    else if (_typeName == "ignition::math::Angle" ||
             _typeName == "angle")
    {
      return ParseUsingStringStream<ignition::math::Angle>(
          tmp, this->key, _valueToSet);
    }
    else if (_typeName == "ignition::math::Color" ||
             _typeName == "color")
    {
      return ParseColorUsingStringStream(tmp, this->key, _valueToSet);
    }
    else if (_typeName == "ignition::math::Vector2i" ||
             _typeName == "vector2i")
    {
      return ParseUsingStringStream<ignition::math::Vector2i>(
          tmp, this->key, _valueToSet);
    }
    else if (_typeName == "ignition::math::Vector2d" ||
             _typeName == "vector2d")
    {
      return ParseUsingStringStream<ignition::math::Vector2d>(
          tmp, this->key, _valueToSet);
    }
    else if (_typeName == "ignition::math::Vector3d" ||
             _typeName == "vector3")
    {
      return ParseUsingStringStream<ignition::math::Vector3d>(
          tmp, this->key, _valueToSet);
    }
    else if (_typeName == "ignition::math::Pose3d" ||
             _typeName == "pose" ||
             _typeName == "Pose")
    {
      const ElementPtr p = this->parentElement.lock();
      if (!this->ignoreParentAttributes && p)
      {
        return ParsePoseUsingStringStream(
            tmp, this->key, p->GetAttributes(), _valueToSet);
      }
      return ParsePoseUsingStringStream(
          tmp, this->key, {}, _valueToSet);
    }
    else if (_typeName == "ignition::math::Quaterniond" ||
             _typeName == "quaternion")
    {
      return ParseUsingStringStream<ignition::math::Quaterniond>(
          tmp, this->key, _valueToSet);
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
bool Param::SetFromString(const std::string &_value,
                          bool _ignoreParentAttributes)
{
  this->dataPtr->ignoreParentAttributes = _ignoreParentAttributes;
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

  auto oldValue = this->dataPtr->value;
  if (!this->ValueFromString(str))
  {
    return false;
  }

  // Check if the value is permitted
  if (!this->ValidateValue())
  {
    this->dataPtr->value = oldValue;
    return false;
  }

  this->dataPtr->set = true;
  return this->dataPtr->set;
}

//////////////////////////////////////////////////
bool Param::SetFromString(const std::string &_value)
{
  return this->SetFromString(_value, false);
}

//////////////////////////////////////////////////
ElementPtr Param::GetParentElement() const
{
  return this->dataPtr->parentElement.lock();
}

//////////////////////////////////////////////////
bool Param::SetParentElement(ElementPtr _parentElement)
{
  auto prevParentElement = this->dataPtr->parentElement;

  this->dataPtr->parentElement = _parentElement;
  if (!this->Reparse())
  {
    this->dataPtr->parentElement = prevParentElement;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
void Param::Reset()
{
  this->dataPtr->value = this->dataPtr->defaultValue;
  this->dataPtr->strValue = this->dataPtr->defaultStrValue;
  this->dataPtr->set = false;
}

//////////////////////////////////////////////////
bool Param::Reparse()
{
  if (!this->dataPtr->ValueFromStringImpl(
      this->dataPtr->typeName, this->dataPtr->strValue, this->dataPtr->value))
  {
    if (const auto parentElement = this->dataPtr->parentElement.lock())
    {
      sdferr << "Failed to set value '" << this->dataPtr->strValue
          << "' to key [" << this->GetKey()
          << "] for new parent element of name '" << parentElement->GetName()
          << "', reverting to previous value '"
          << this->GetAsString() << "'.\n";
    }
    else
    {
      sdferr << "Failed to set value '" << this->dataPtr->strValue
          << "' to key [" << this->GetKey() << "] without a parent element, "
          << "reverting to previous value '" << this->GetAsString() << "'.\n";
    }
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
ParamPtr Param::Clone() const
{
  return std::make_shared<Param>(*this);
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

/////////////////////////////////////////////////
bool Param::IgnoresParentElementAttribute() const
{
  const auto parentElement = this->dataPtr->parentElement.lock();
  return !parentElement || this->dataPtr->ignoreParentAttributes;
}

/////////////////////////////////////////////////
bool Param::ValidateValue() const
{
  return std::visit(
      [this](const auto &_val) -> bool
      {
        using T = std::decay_t<decltype(_val)>;
        // cppcheck-suppress syntaxError
        // cppcheck-suppress unmatchedSuppression
        if constexpr (std::is_scalar_v<T>)
        {
          if (this->dataPtr->minValue.has_value())
          {
            if (_val < std::get<T>(*this->dataPtr->minValue))
            {
              sdferr << "The value [" << _val
                     << "] is less than the minimum allowed value of ["
                     << *this->GetMinValueAsString() << "] for key ["
                     << this->GetKey() << "]\n";
              return false;
            }
          }
          if (this->dataPtr->maxValue.has_value())
          {
            if (_val > std::get<T>(*this->dataPtr->maxValue))
            {
              sdferr << "The value [" << _val
                     << "] is greater than the maximum allowed value of ["
                     << *this->GetMaxValueAsString() << "] for key ["
                     << this->GetKey() << "]\n";
              return false;
            }
          }
        }
        return true;
      }, this->dataPtr->value);
}
