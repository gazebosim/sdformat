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
  sdf::Errors errors;
  this->dataPtr->Init(_key, _typeName, _default, _required,
                      errors, _description);

  if(!errors.empty())
  {
    for (unsigned int i = 0; i < errors.size() - 1; ++i)
    {
      sdferr << errors[i].Message() << "\n";
    }

    SDF_ASSERT(false, errors.back().Message());
  }
}

//////////////////////////////////////////////////
Param::Param(const std::string &_key, const std::string &_typeName,
             const std::string &_default, bool _required,
             sdf::Errors &_errors,
             const std::string &_description)
  : dataPtr(new ParamPrivate)
{
  this->dataPtr->Init(_key, _typeName, _default, _required,
                      _errors, _description);
}

//////////////////////////////////////////////////
Param::Param(const std::string &_key, const std::string &_typeName,
             const std::string &_default, bool _required,
             const std::string &_minValue, const std::string &_maxValue,
             const std::string &_description)
  : dataPtr(new ParamPrivate)
{
  sdf::Errors errors;
  this->dataPtr->Init(_key, _typeName, _default, _required, _minValue,
                      _maxValue, errors, _description);

  if(!errors.empty())
  {
    for (unsigned int i = 0; i < errors.size() - 1; ++i)
    {
      sdferr << errors[i].Message() << "\n";
    }

    SDF_ASSERT(false, errors.back().Message());
  }
}

//////////////////////////////////////////////////
Param::Param(const std::string &_key, const std::string &_typeName,
             const std::string &_default, bool _required,
             const std::string &_minValue, const std::string &_maxValue,
             sdf::Errors &_errors,
             const std::string &_description)
  : dataPtr(new ParamPrivate)
{
  this->dataPtr->Init(_key, _typeName, _default, _required, _minValue,
                      _maxValue, _errors, _description);
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
  sdf::Errors errors;
  this->GetAny(_anyVal, errors);
  if(!errors.empty())
  {
    sdferr << errors;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
bool Param::GetAny(std::any &_anyVal, sdf::Errors &_errors) const
{
  if (this->IsType<int>())
  {
    int ret = 0;
    if (!this->Get<int>(ret, _errors))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Could not get a parameter of type [int]"});
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<std::uint64_t>())
  {
    uint64_t ret = 0;
    if (!this->Get<std::uint64_t>(ret, _errors))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Could not get a parameter of type [uint64_t]"});
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<double>())
  {
    double ret = 0;
    if (!this->Get<double>(ret, _errors))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Could not get a parameter of type [double]"});
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<float>())
  {
    float ret = 0;
    if (!this->Get<float>(ret, _errors))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Could not get a parameter of type [float]"});
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<bool>())
  {
    bool ret = false;
    if (!this->Get<bool>(ret, _errors))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Could not get a parameter of type [bool]"});
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<std::string>())
  {
    std::string ret;
    if (!this->Get<std::string>(ret, _errors))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Could not get a parameter of type [std::string]"});
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<unsigned int>())
  {
    unsigned int ret = 0;
    if (!this->Get<unsigned int>(ret, _errors))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Could not get a parameter of type [unsigned int]"});
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<char>())
  {
    char ret = 0;
    if (!this->Get<char>(ret, _errors))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Could not get a parameter of type [char]"});
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<sdf::Time>())
  {
    sdf::Time ret;
    if (!this->Get<sdf::Time>(ret, _errors))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Could not get a parameter of type [char]"});
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<gz::math::Color>())
  {
    gz::math::Color ret;
    if (!this->Get<gz::math::Color>(ret, _errors))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Could not get a parameter of type [gz::math::Color]"});
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<gz::math::Vector3d>())
  {
    gz::math::Vector3d ret;
    if (!this->Get<gz::math::Vector3d>(ret, _errors))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Could not get a parameter of type [gz::math::Vector3d]"});
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<gz::math::Vector2i>())
  {
    gz::math::Vector2i ret;
    if (!this->Get<gz::math::Vector2i>(ret, _errors))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Could not get a parameter of type [gz::math::Vector2i]"});
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<gz::math::Vector2d>())
  {
    gz::math::Vector2d ret;
    if (!this->Get<gz::math::Vector2d>(ret, _errors))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Could not get a parameter of type [gz::math::Vector2d]"});
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<gz::math::Pose3d>())
  {
    gz::math::Pose3d ret;
    if (!this->Get<gz::math::Pose3d>(ret, _errors))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Could not get a parameter of type [gz::math::Pose3d]"});
      return false;
    }
    _anyVal = ret;
  }
  else if (this->IsType<gz::math::Quaterniond>())
  {
    gz::math::Quaterniond ret;
    if (!this->Get<gz::math::Quaterniond>(ret, _errors))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Could not get a parameter of type [gz::math::Quaterniond]"});
      return false;
    }
    _anyVal = ret;
  }
  else
  {
    _errors.push_back({ErrorCode::PARAMETER_ERROR,
        "Type of parameter not known: [" + this->GetTypeName() + "]"});
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
void Param::Update()
{
  sdf::Errors errors;
  this->Update(errors);
  if (!errors.empty())
    sdferr << errors;
}

//////////////////////////////////////////////////
void Param::Update(sdf::Errors &_errors)
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
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Unable to set value using Update for key["
          + this->dataPtr->key + "]"});
    }
  }
  else
  {
    _errors.push_back({ErrorCode::PARAMETER_ERROR,
        "[updateFunc] is not set."});
  }
}

//////////////////////////////////////////////////
std::string Param::GetAsString(const PrintConfig &_config) const
{
  sdf::Errors errors;
  std::string result = GetAsString(errors, _config);
  if (!errors.empty())
    sdferr << errors;
  return result;
}

//////////////////////////////////////////////////
std::string Param::GetAsString(sdf::Errors &_errors,
                               const PrintConfig &_config) const
{
  std::string valueStr;
  if (this->GetSet() &&
      this->dataPtr->StringFromValueImpl(_config,
                                         this->dataPtr->typeName,
                                         this->dataPtr->value,
                                         valueStr,
                                         _errors))
  {
    return valueStr;
  }

  return this->GetDefaultAsString(_errors, _config);
}

//////////////////////////////////////////////////
std::string Param::GetDefaultAsString(const PrintConfig &_config) const
{
  sdf::Errors errors;
  std::string result = this->GetDefaultAsString(errors, _config);
  if (!errors.empty())
    sdferr << errors;
  return result;
}

//////////////////////////////////////////////////
std::string Param::GetDefaultAsString(sdf::Errors &_errors,
                                      const PrintConfig &_config) const
{
  std::string defaultStr;
  if (this->dataPtr->StringFromValueImpl(
        _config,
        this->dataPtr->typeName,
        this->dataPtr->defaultValue,
        defaultStr,
        _errors))
  {
    return defaultStr;
  }

  _errors.push_back({ErrorCode::PARAMETER_ERROR,
      "Unable to get string from default value, "
      "using ParamStreamer instead."});

  StringStreamClassicLocale ss;
  ss << ParamStreamer{ this->dataPtr->defaultValue, _config.OutPrecision() };
  return ss.str();
}

//////////////////////////////////////////////////
std::optional<std::string> Param::GetMinValueAsString(
    const PrintConfig &_config) const
{
  sdf::Errors errors;
  auto result = GetMinValueAsString(errors, _config);
  if (!errors.empty())
    sdferr << errors;
  return result;
}

//////////////////////////////////////////////////
std::optional<std::string> Param::GetMinValueAsString(
    sdf::Errors &_errors,
    const PrintConfig &_config) const
{
  if (this->dataPtr->minValue.has_value())
  {
    std::string valueStr;
    if (!this->dataPtr->StringFromValueImpl(_config,
                                            this->dataPtr->typeName,
                                            this->dataPtr->minValue.value(),
                                            valueStr,
                                            _errors))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Unable to get min value as string."});
      return std::nullopt;
    }

    return valueStr;
  }
  return std::nullopt;
}

//////////////////////////////////////////////////
std::optional<std::string> Param::GetMaxValueAsString(
    const PrintConfig &_config) const
{
  sdf::Errors errors;
  auto result = GetMaxValueAsString(errors, _config);
  if (!errors.empty())
    sdferr << errors;
  return result;
}

//////////////////////////////////////////////////
std::optional<std::string> Param::GetMaxValueAsString(
    sdf::Errors &_errors,
    const PrintConfig &_config) const
{
  if (this->dataPtr->maxValue.has_value())
  {
    std::string valueStr;
    if (!this->dataPtr->StringFromValueImpl(_config,
                                            this->dataPtr->typeName,
                                            this->dataPtr->maxValue.value(),
                                            valueStr,
                                            _errors))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Unable to get max value as string."});
      return std::nullopt;
    }

    return valueStr;
  }
  return std::nullopt;
}

//////////////////////////////////////////////////
/// \brief Helper function for Param::ValueFromString
/// \param[in] _input Input string.
/// \param[in] _key Key of the parameter, used for error message.
/// \param[out] _value This will be set with the parsed value.
/// \param[out] _errors Vector of errors.
/// \return True if parsing succeeded.
template <typename T>
bool ParseUsingStringStream(const std::string &_input, const std::string &_key,
                            ParamPrivate::ParamVariant &_value,
                            sdf::Errors &_errors)
{
  StringStreamClassicLocale ss(_input);
  T _val;
  ss >> _val;
  if (ss.fail())
  {
    _errors.push_back({ErrorCode::PARAMETER_ERROR,
        "Unknown error. Unable to set value [" + _input + " ] for key["
           + _key + "]"});
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
/// \param[out] _errors Vector of errors.
/// \return True if parsing colors succeeded.
bool ParseColorUsingStringStream(const std::string &_input,
    const std::string &_key, ParamPrivate::ParamVariant &_value,
    sdf::Errors &_errors)
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
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Invalid argument. Unable to set value [" + token
          + "] for key [" + _key + "]."});
      isValidColor = false;
      break;
    }
    // Catch out of range exception from std::stof
    catch(std::out_of_range &)
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Out of range. Unable to set value [" + token
          + "] for key [" + _key + "]."});
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
    _value = gz::math::Color(colors[0], colors[1], colors[2], colors[3]);
  }
  else
  {
    _errors.push_back({ErrorCode::PARAMETER_ERROR,
        "The value <" + _key + ">" + _input + "</" + _key + "> is invalid."});
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
/// \param[out] _errors Vector of errors.
/// \return True if parsing pose succeeded.
bool ParsePoseUsingStringStream(const std::string &_input,
    const std::string &_key, const Param_V &_attributes,
    ParamPrivate::ParamVariant &_value,
    sdf::Errors &_errors)
{
  const bool defaultParseAsDegrees = false;
  bool parseAsDegrees = defaultParseAsDegrees;

  const std::string defaultRotationFormat = "euler_rpy";
  std::string rotationFormat = defaultRotationFormat;

  const std::size_t defaultDesiredSize = 6u;
  std::size_t desiredSize = defaultDesiredSize;

  for (const auto &p : _attributes)
  {
    const std::string key = p->GetKey();

    if (key == "degrees")
    {
      if (!p->Get<bool>(parseAsDegrees, _errors))
      {
        _errors.push_back({ErrorCode::PARAMETER_ERROR,
            "Invalid boolean value found for attribute "
            "//pose[@degrees]."});
        return false;
      }
    }
    else if (key == "rotation_format")
    {
      rotationFormat = p->GetAsString(_errors);

      if (rotationFormat == "euler_rpy")
      {
        // Already the default, no modifications needed.
      }
      else if (rotationFormat == "quat_xyzw")
      {
        desiredSize = 7u;
      }
      else
      {
        _errors.push_back({ErrorCode::PARAMETER_ERROR,
            "Undefined attribute //pose[@rotation_format='"
            + rotationFormat + "'], only 'euler_rpy' and 'quat_xyzw'"
            + " is supported."});
        return false;
      }
    }
  }

  if (rotationFormat == "quat_xyzw" && parseAsDegrees)
  {
    _errors.push_back({ErrorCode::PARAMETER_ERROR,
        "The attribute //pose[@degrees='true'] does not apply when "
        "parsing quaternions, //pose[@rotation_format='quat_xyzw']."});
    return false;
  }

  if (_input.empty())
  {
    _value = gz::math::Pose3d::Zero;
    return true;
  }

  StringStreamClassicLocale ss(_input);
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
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Invalid argument. Unable to set value [" + _input
          + "] for key [" + _key + "]."});
      isValidPose = false;
      break;
    }
    // Catch out of range exception from std::stod
    catch(std::out_of_range &)
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Out of range. Unable to set value [" + token
          + "] for key [" + _key + "]."});
      isValidPose = false;
      break;
    }

    if (!std::isfinite(v))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Pose values must be finite."});
      isValidPose = false;
      break;
    }

    if (valueIndex >= desiredSize)
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "The value for //pose[@rotation_format='" + rotationFormat
          + "'] must have " + std::to_string(desiredSize)
          + " values, but more than that were found in '" + _input + "'."});
      isValidPose = false;
      break;
    }

    values[valueIndex++] = v;
  }

  if (!isValidPose)
    return false;

  if (valueIndex != desiredSize)
  {
    _errors.push_back({ErrorCode::PARAMETER_ERROR,
        "The value for //pose[@rotation_format='" + rotationFormat
        + "'] must have " + std::to_string(desiredSize) + " values, but "
        + std::to_string(valueIndex) + " were found instead in '"
        + _input + "'."});
    return false;
  }

  if (rotationFormat == "euler_rpy")
  {
    if (parseAsDegrees)
    {
      _value = gz::math::Pose3d(values[0], values[1], values[2],
          GZ_DTOR(values[3]), GZ_DTOR(values[4]), GZ_DTOR(values[5]));
    }
    else
    {
      _value = gz::math::Pose3d(values[0], values[1], values[2],
          values[3], values[4], values[5]);
    }
  }
  else
  {
    _value = gz::math::Pose3d(values[0], values[1], values[2],
        values[6], values[3], values[4], values[5]);
  }

  return true;
}

//////////////////////////////////////////////////
void ParamPrivate::Init(const std::string &_key, const std::string &_typeName,
             const std::string &_default, bool _required,
             sdf::Errors &_errors,
             const std::string &_description)
{
  this->key = _key;
  this->required = _required;
  this->typeName = _typeName;
  this->description = _description;
  this->set = false;
  this->ignoreParentAttributes = false;
  this->defaultStrValue = _default;

  if(!(this->ValueFromStringImpl(
          this->typeName,
          _default,
          this->defaultValue,
          _errors)))
  {
    _errors.push_back({ErrorCode::PARAMETER_ERROR,
                     "Invalid parameter"});
    return;
  }
  this->value = this->defaultValue;
  this->strValue = std::nullopt;
}

//////////////////////////////////////////////////
void ParamPrivate::Init(const std::string &_key, const std::string &_typeName,
             const std::string &_default, bool _required,
             const std::string &_minValue, const std::string &_maxValue,
             sdf::Errors &_errors,
             const std::string &_description)
{
  this->Init(_key, _typeName, _default, _required, _errors, _description);
  if (!_minValue.empty())
  {
    if (!(this->ValueFromStringImpl(
            this->typeName,
            _minValue,
            this->minValue.emplace(),
            _errors)))
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Invalid [min] parameter in "
          "SDFormat description of [" + _key + "]"});
    }
  }

  if (!_maxValue.empty())
  {
    if(!(this->ValueFromStringImpl(
            this->typeName,
            _maxValue,
            this->maxValue.emplace(),
            _errors)))

    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Invalid [max] parameter in SDFormat description of [" +
          _key + "]"});
    }
  }
}

//////////////////////////////////////////////////
bool ParamPrivate::ValueFromStringImpl(const std::string &_typeName,
                                       const std::string &_valueStr,
                                       ParamVariant &_valueToSet,
                                       sdf::Errors &_errors) const
{
  // Under some circumstances, latin locales (es_ES or pt_BR) will return a
  // comma for decimal position instead of a dot, making the conversion
  // to fail. See bug #60 for more information. Force to use always C
  setlocale(LC_NUMERIC, "C");
  std::string trimmed = sdf::trim(_valueStr);
  std::string tmp(trimmed);
  std::string lowerTmp = lowercase(trimmed);

  // "true" and "false" doesn't work properly (except for string)
  if (_typeName != "string" && _typeName != "std::string")
  {
    if (lowerTmp == "true")
    {
      tmp = "1";
    }
    else if (lowerTmp == "false")
    {
      tmp = "0";
    }
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
        _errors.push_back({ErrorCode::PARAMETER_ERROR,
            "Invalid boolean value"});
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
                                                   _valueToSet, _errors);
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
                                               _valueToSet, _errors);
    }
    else if (_typeName == "gz::math::Angle" ||
             _typeName == "angle")
    {
      return ParseUsingStringStream<gz::math::Angle>(
          tmp, this->key, _valueToSet, _errors);
    }
    else if (_typeName == "gz::math::Color" ||
             _typeName == "color")
    {
      return ParseColorUsingStringStream(tmp, this->key, _valueToSet, _errors);
    }
    else if (_typeName == "gz::math::Vector2i" ||
             _typeName == "vector2i")
    {
      return ParseUsingStringStream<gz::math::Vector2i>(
          tmp, this->key, _valueToSet, _errors);
    }
    else if (_typeName == "gz::math::Vector2d" ||
             _typeName == "vector2d")
    {
      return ParseUsingStringStream<gz::math::Vector2d>(
          tmp, this->key, _valueToSet, _errors);
    }
    else if (_typeName == "gz::math::Vector3d" ||
             _typeName == "vector3")
    {
      return ParseUsingStringStream<gz::math::Vector3d>(
          tmp, this->key, _valueToSet, _errors);
    }
    else if (_typeName == "gz::math::Pose3d" ||
             _typeName == "pose" ||
             _typeName == "Pose")
    {
      const ElementPtr p = this->parentElement.lock();
      if (!this->ignoreParentAttributes && p)
      {
        return ParsePoseUsingStringStream(
            tmp, this->key, p->GetAttributes(), _valueToSet, _errors);
      }
      return ParsePoseUsingStringStream(
          tmp, this->key, {}, _valueToSet, _errors);
    }
    else if (_typeName == "gz::math::Quaterniond" ||
             _typeName == "quaternion")
    {
      return ParseUsingStringStream<gz::math::Quaterniond>(
          tmp, this->key, _valueToSet, _errors);
    }
    else
    {
      _errors.push_back({ErrorCode::UNKNOWN_PARAMETER_TYPE,
          "Unknown parameter type[" + _typeName + "]"});
      return false;
    }
  }
  // Catch invalid argument exception from std::stoi/stoul/stod/stof
  catch(std::invalid_argument &)
  {
    _errors.push_back({ErrorCode::PARAMETER_ERROR,
        "Invalid argument. Unable to set value ["
        + _valueStr + "] for key["
        + this->key + "]."});
    return false;
  }
  // Catch out of range exception from std::stoi/stoul/stod/stof
  catch(std::out_of_range &)
  {
    _errors.push_back({ErrorCode::PARAMETER_ERROR,
        "Out of range. Unable to set value ["
        + _valueStr + " ] for key["
        + this->key + "]."});
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
/// \brief Helper function for StringFromValueImpl for pose.
/// \param[in] _config Printing configuration for the output string.
/// \param[in] _parentAttributes Parent Element Attributes.
/// \param[in] _value The variant value of this pose.
/// \param[out] _valueStr The pose as a string.
/// \param[out] _errors Vector of errors.
/// \return True if the string was successfully retrieved from the pose, false
/// otherwise.
/////////////////////////////////////////////////
bool PoseStringFromValue(const PrintConfig &_config,
                         const Param_V &_parentAttributes,
                         const ParamPrivate::ParamVariant &_value,
                         std::string &_valueStr,
                         sdf::Errors &_errors)
{
  StringStreamClassicLocale ss;

  if (_config.OutPrecision() == std::numeric_limits<int>::max())
    ss << std::setprecision(std::numeric_limits<double>::max_digits10);
  else
    ss << std::setprecision(_config.OutPrecision());

  const gz::math::Pose3d *pose =
      std::get_if<gz::math::Pose3d>(&_value);
  if (!pose)
  {
    _errors.push_back({ErrorCode::PARAMETER_ERROR,
        "Unable to get pose value from variant."});
    return false;
  }

  const bool defaultInDegrees = false;
  bool inDegrees = defaultInDegrees;

  const std::string defaultRotationFormat = "euler_rpy";
  std::string rotationFormat = defaultRotationFormat;

  // When @degrees and @rotation_format attributes are not set, a single space
  // delimiter is used to prevent breaking behavior and tests.
  const std::string defaultPosRotDelimiter = " ";
  const std::string threeSpacedDelimiter = "   ";
  std::string posRotDelimiter = defaultPosRotDelimiter;

  const bool defaultSnapDegreesToInterval = false;
  bool snapDegreesToInterval = defaultSnapDegreesToInterval;

  // Checking parent Element Attributes for desired pose representations.
  for (const auto &p : _parentAttributes)
  {
    const std::string key = p->GetKey();

    if (key == "degrees")
    {
      if (!p->Get<bool>(inDegrees, _errors))
      {
        _errors.push_back({ErrorCode::PARAMETER_ERROR,
            "Unable to get //pose[@degrees] attribute as bool."});
        return false;
      }
      if (p->GetSet())
      {
        posRotDelimiter = threeSpacedDelimiter;
      }
    }
    else if (key == "rotation_format")
    {
      rotationFormat = p->GetAsString(_errors);
      if (p->GetSet())
      {
        posRotDelimiter = threeSpacedDelimiter;
      }
    }
  }

  // Checking PrintConfig for desired pose representations. This overrides
  // any parent Element Attributes.
  if (_config.RotationInDegrees())
  {
    inDegrees = true;
    rotationFormat = "euler_rpy";
    posRotDelimiter = threeSpacedDelimiter;
  }
  if (_config.RotationSnapToDegrees().has_value() &&
      _config.RotationSnapTolerance().has_value())
  {
    inDegrees = true;
    rotationFormat = "euler_rpy";
    snapDegreesToInterval = true;
    posRotDelimiter = threeSpacedDelimiter;
  }

  // Helper function that sanitizes zero values like '-0'
  auto sanitizeZero = [&_config](double _number)
  {
    StringStreamClassicLocale stream;
    if (std::fpclassify(_number) == FP_ZERO)
    {
      stream << 0;
    }
    else
    {
      if (_config.OutPrecision() == std::numeric_limits<int>::max())
        stream << std::setprecision(std::numeric_limits<double>::max_digits10);
      else
        stream << std::setprecision(_config.OutPrecision());

      stream << _number;
    }
    return stream.str();
  };

  // Returning pose string representations based on desired configurations.
  if (rotationFormat == "quat_xyzw" && inDegrees)
  {
    _errors.push_back({ErrorCode::PARAMETER_ERROR,
        "Invalid pose with //pose[@degrees='true'] and "
        "//pose[@rotation_format='quat_xyzw']."});
    return false;
  }
  else if (rotationFormat == "quat_xyzw")
  {
    ss << pose->Pos() << posRotDelimiter
       << sanitizeZero(pose->Rot().X()) << " "
       << sanitizeZero(pose->Rot().Y()) << " "
       << sanitizeZero(pose->Rot().Z()) << " "
       << sanitizeZero(pose->Rot().W());
    _valueStr = ss.str();
    return true;
  }
  else if (rotationFormat == "euler_rpy" && inDegrees && snapDegreesToInterval)
  {
    // Helper function that returns a snapped value if it is within the
    // tolerance of multiples of interval, otherwise the original value is
    // returned.
    auto snapToInterval =
        [](double _val, unsigned int _interval, double _tolerance)
    {
      double closestQuotient = std::round(_val / _interval);
      double distance = std::abs(_val - closestQuotient * _interval);
      if (distance < _tolerance)
      {
        return _interval * closestQuotient;
      }
      return _val;
    };

    const unsigned int interval = _config.RotationSnapToDegrees().value();
    const double tolerance = _config.RotationSnapTolerance().value();

    ss << pose->Pos() << posRotDelimiter
       << sanitizeZero(snapToInterval(
              GZ_RTOD(pose->Rot().Roll()), interval, tolerance)) << " "
       << sanitizeZero(snapToInterval(
              GZ_RTOD(pose->Rot().Pitch()), interval, tolerance)) << " "
       << sanitizeZero(snapToInterval(
              GZ_RTOD(pose->Rot().Yaw()), interval, tolerance));
    _valueStr = ss.str();
    return true;
  }
  else if (rotationFormat == "euler_rpy" && inDegrees)
  {
    ss << pose->Pos() << posRotDelimiter
       << sanitizeZero(GZ_RTOD(pose->Rot().Roll())) << " "
       << sanitizeZero(GZ_RTOD(pose->Rot().Pitch())) << " "
       << sanitizeZero(GZ_RTOD(pose->Rot().Yaw()));
    _valueStr = ss.str();
    return true;
  }

  ss << pose->Pos() << posRotDelimiter
     << sanitizeZero(pose->Rot().Roll()) << " "
     << sanitizeZero(pose->Rot().Pitch()) << " "
     << sanitizeZero(pose->Rot().Yaw());
  _valueStr = ss.str();
  return true;
}

/////////////////////////////////////////////////
bool ParamPrivate::StringFromValueImpl(
    const PrintConfig &_config,
    const std::string &_typeName,
    const ParamVariant &_value,
    std::string &_valueStr,
    sdf::Errors &_errors) const
{
  // This will be handled in a type specific manner
  if (_typeName == "bool")
  {
    const bool *val = std::get_if<bool>(&_value);
    if (!val)
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Unable to get bool value from variant."});
      return false;
    }

    _valueStr = *val ? "true" : "false";
    return true;
  }
  else if (_typeName == "gz::math::Pose3d" ||
      _typeName == "pose" ||
      _typeName == "Pose")
  {
    const ElementPtr p = this->parentElement.lock();
    if (!this->ignoreParentAttributes && p)
    {
      return PoseStringFromValue(
          _config, p->GetAttributes(), _value, _valueStr, _errors);
    }
    return PoseStringFromValue(_config, {}, _value, _valueStr, _errors);
  }

  StringStreamClassicLocale ss;
  ss << ParamStreamer{ _value, _config.OutPrecision() };
  _valueStr = ss.str();
  return true;
}

//////////////////////////////////////////////////
bool Param::SetFromString(const std::string &_value,
                          bool _ignoreParentAttributes)
{
  sdf::Errors errors;
  bool result = this->SetFromString(_value,
                          _ignoreParentAttributes,
                          errors);
  if (!errors.empty())
    sdferr << errors;
  return result;
}

//////////////////////////////////////////////////
bool Param::SetFromString(const std::string &_value,
                          bool _ignoreParentAttributes,
                          sdf::Errors &_errors)
{
  this->dataPtr->ignoreParentAttributes = _ignoreParentAttributes;
  std::string str = sdf::trim(_value.c_str());

  if (str.empty() && this->dataPtr->required)
  {
    _errors.push_back({ErrorCode::PARAMETER_ERROR,
        "Empty string used when setting a required parameter. Key["
        + this->GetKey() + "]"});
    return false;
  }
  else if (str.empty())
  {
    this->dataPtr->value = this->dataPtr->defaultValue;
    this->dataPtr->strValue = str;
    return true;
  }

  auto oldValue = this->dataPtr->value;
  if (!this->dataPtr->ValueFromStringImpl(this->dataPtr->typeName,
                                          str,
                                          this->dataPtr->value,
                                          _errors))
  {
    return false;
  }
  this->dataPtr->strValue = str;

  // Check if the value is permitted
  if (!this->ValidateValue(_errors))
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
  sdf::Errors errors;
  bool result = this->SetFromString(_value, false, errors);
  if (!errors.empty())
    sdferr << errors;
  return result;
}

//////////////////////////////////////////////////
bool Param::SetFromString(const std::string &_value, sdf::Errors &_errors)
{
  return this->SetFromString(_value, false, _errors);
}

//////////////////////////////////////////////////
ElementPtr Param::GetParentElement() const
{
  return this->dataPtr->parentElement.lock();
}

//////////////////////////////////////////////////
bool Param::SetParentElement(ElementPtr _parentElement)
{
  sdf::Errors errors;
  bool result = this->SetParentElement(_parentElement, errors);
  if (!errors.empty())
    sdferr << errors;
  return result;
}

//////////////////////////////////////////////////
bool Param::SetParentElement(ElementPtr _parentElement, sdf::Errors &_errors)
{
  auto prevParentElement = this->dataPtr->parentElement;

  this->dataPtr->parentElement = _parentElement;
  if (!this->Reparse(_errors))
  {
    this->dataPtr->parentElement = prevParentElement;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool Param::SetParentElementNoReparse(ElementPtr _parentElement)
{
  this->dataPtr->parentElement = _parentElement;
  return true;
}

//////////////////////////////////////////////////
void Param::Reset()
{
  this->dataPtr->value = this->dataPtr->defaultValue;
  this->dataPtr->strValue = std::nullopt;
  this->dataPtr->set = false;
}

//////////////////////////////////////////////////
bool Param::Reparse()
{
  sdf::Errors errors;
  bool result = this->Reparse(errors);
  if (!errors.empty())
    sdferr << errors;
  return result;
}

//////////////////////////////////////////////////
bool Param::Reparse(sdf::Errors &_errors)
{
  std::string strToReparse;
  if (this->dataPtr->strValue.has_value())
  {
    strToReparse = this->dataPtr->strValue.value();
  }
  // A default PrintConfig can be used here, as Reparse() is not called in the
  // code path from the 'gz sdf -p' command.
  else if (!this->dataPtr->StringFromValueImpl(PrintConfig(),
                                               this->dataPtr->typeName,
                                               this->dataPtr->defaultValue,
                                               strToReparse,
                                               _errors))
  {
    _errors.push_back({ErrorCode::PARAMETER_ERROR,
        "Failed to obtain string from default value during reparsing."});
    return false;
  }

  if (!this->dataPtr->ValueFromStringImpl(
      this->dataPtr->typeName, strToReparse, this->dataPtr->value, _errors))
  {
    if (const auto parentElement = this->dataPtr->parentElement.lock())
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Failed to set value '" + strToReparse
          + "' to key [" + this->GetKey()
          + "] for new parent element of name '" + parentElement->GetName()
          + "', reverting to previous value '"
          + this->GetAsString(_errors) + "'."});
    }
    else
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Failed to set value '" + strToReparse
          + "' to key [" + this->GetKey() + "] without a parent element, "
          + "reverting to previous value '" +
          this->GetAsString(_errors) + "'."});
    }
    return false;
  }
  // ValueFromStringImpl might make assumptions as to what the default value
  // should be, so if strToReparse is empty, assign the correct default value.
  if (strToReparse.empty())
  {
    this->dataPtr->value = this->dataPtr->defaultValue;
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
  sdf::Errors errors;
  bool result = this->ValidateValue(errors);
  if (!errors.empty())
    sdferr << errors;
  return result;
}

/////////////////////////////////////////////////
bool Param::ValidateValue(sdf::Errors &_errors) const
{
  return std::visit(
      [this, &_errors](const auto &_val) -> bool
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
              std::ostringstream oss;
              oss << "The value [" << _val
                  << "] is less than the minimum allowed value of ["
                  << *this->GetMinValueAsString(_errors) << "] for key ["
                  << this->GetKey() << "]";
              _errors.push_back({ErrorCode::PARAMETER_ERROR, oss.str()});
              return false;
            }
          }
          if (this->dataPtr->maxValue.has_value())
          {
            if (_val > std::get<T>(*this->dataPtr->maxValue))
            {
              std::ostringstream oss;
              oss << "The value [" << _val
                  << "] is greater than the maximum allowed value of ["
                  << *this->GetMaxValueAsString(_errors) << "] for key ["
                  << this->GetKey() << "]";
              _errors.push_back({ErrorCode::PARAMETER_ERROR, oss.str()});
              return false;
            }
          }
        }
        return true;
      }, this->dataPtr->value);
}
