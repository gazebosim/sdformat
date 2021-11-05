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

#include "sdf/PrintConfig.hh"
#include "sdf/Console.hh"

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
};

/////////////////////////////////////////////////
PrintConfig::PrintConfig()
    : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
void PrintConfig::SetRotationInDegrees(bool _value)
{
  this->dataPtr->rotationInDegrees = _value;
}

/////////////////////////////////////////////////
bool PrintConfig::GetRotationInDegrees() const
{
  return this->dataPtr->rotationInDegrees;
}

/////////////////////////////////////////////////
bool PrintConfig::SetRotationSnapToDegrees(unsigned int _interval,
                                           double _tolerance)
{
  if (_interval == 0 || _interval > 360)
  {
    sdferr << "Interval value to snap to must be larger than 0, and less than "
           << "or equal to 360.\n";
    return false;
  }

  if (_tolerance <= 0 || _tolerance > 360)
  {
    sdferr << "Tolerance must be larger than 0, and less than or equal to "
           << "360.\n";
    return false;
  }

  this->dataPtr->rotationSnapToDegrees = _interval;
  this->dataPtr->rotationSnapTolerance = _tolerance;
  return true;
}

/////////////////////////////////////////////////
std::optional<unsigned int> PrintConfig::GetRotationSnapToDegrees() const
{
  return this->dataPtr->rotationSnapToDegrees;
}

/////////////////////////////////////////////////
std::optional<double> PrintConfig::GetRotationSnapTolerance() const
{
  return this->dataPtr->rotationSnapTolerance;
}
