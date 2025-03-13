/*
 * Copyright 2021 Open Source Robotics Foundation
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
#include <limits>
#include <sstream>

#include "sdf/PrintConfig.hh"
#include "sdf/Console.hh"
#include "Utils.hh"

using namespace sdf;

/////////////////////////////////////////////////
class PrintConfig::Implementation
{
  /// \brief True if rotation in poses are to be printed in degrees.
  public: bool rotationInDegrees = false;

  /// \brief The interval in degrees, which rotation in poses shall snap to,
  /// if they are within the tolerance value of rotationSnapTolerance.
  public: std::optional<unsigned int> rotationSnapToDegrees = std::nullopt;

  /// \brief The tolerance which is used to determine whether snapping of
  /// rotation in poses happen.
  public: std::optional<double> rotationSnapTolerance = std::nullopt;

  /// \brief True to preserve <include> tags, false to expand.
  public: bool preserveIncludes = false;

  /// \brief The output stream's precision. By default, uses maximum precision.
  public: int outPrecision = std::numeric_limits<int>::max();
};

/////////////////////////////////////////////////
PrintConfig::PrintConfig()
    : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
void PrintConfig::SetRotationInDegrees(bool _value)
{
  this->dataPtr->rotationInDegrees = _value;
}

/////////////////////////////////////////////////
bool PrintConfig::RotationInDegrees() const
{
  return this->dataPtr->rotationInDegrees;
}

/////////////////////////////////////////////////
void PrintConfig::SetPreserveIncludes(bool _preserve)
{
  this->dataPtr->preserveIncludes = _preserve;
}

/////////////////////////////////////////////////
bool PrintConfig::PreserveIncludes() const
{
  return this->dataPtr->preserveIncludes;
}

/////////////////////////////////////////////////
bool PrintConfig::SetRotationSnapToDegrees(unsigned int _interval,
                                           double _tolerance)
{
  sdf::Errors errors;
  bool result = this->SetRotationSnapToDegrees(_interval,
                                               _tolerance,
                                               errors);
  throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
bool PrintConfig::SetRotationSnapToDegrees(unsigned int _interval,
                                           double _tolerance,
                                           sdf::Errors &_errors)
{
  if (_interval == 0 || _interval > 360)
  {
    std::stringstream ss;
    ss << "Interval value to snap to must be larger than 0, and less than "
       << "or equal to 360.";
    _errors.push_back({ErrorCode::ROTATION_SNAP_CONFIG_ERROR, ss.str()});
    return false;
  }

  if (_tolerance <= 0 || _tolerance > 360 ||
      _tolerance >= static_cast<double>(_interval))
  {
    std::stringstream ss;
    ss << "Tolerance must be larger than 0, less than or equal to "
       << "360, and less than the provided interval.";
    _errors.push_back({ErrorCode::ROTATION_SNAP_CONFIG_ERROR, ss.str()});
    return false;
  }

  this->dataPtr->rotationSnapToDegrees = _interval;
  this->dataPtr->rotationSnapTolerance = _tolerance;
  return true;
}

/////////////////////////////////////////////////
std::optional<unsigned int> PrintConfig::RotationSnapToDegrees() const
{
  return this->dataPtr->rotationSnapToDegrees;
}

/////////////////////////////////////////////////
std::optional<double> PrintConfig::RotationSnapTolerance() const
{
  return this->dataPtr->rotationSnapTolerance;
}

/////////////////////////////////////////////////
void PrintConfig::SetOutPrecision(int _precision)
{
  this->dataPtr->outPrecision = _precision;
}

/////////////////////////////////////////////////
int PrintConfig::OutPrecision() const
{
  return this->dataPtr->outPrecision;
}

/////////////////////////////////////////////////
bool PrintConfig::operator==(const PrintConfig &_config) const
{
  if (this->RotationInDegrees() == _config.RotationInDegrees() &&
      this->RotationSnapToDegrees() == _config.RotationSnapToDegrees() &&
      this->RotationSnapTolerance() == _config.RotationSnapTolerance() &&
      this->PreserveIncludes() == _config.PreserveIncludes() &&
      this->OutPrecision() == _config.OutPrecision())
  {
    return true;
  }
  return false;
}
