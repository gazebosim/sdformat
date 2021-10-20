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
#include "sdf/Assert.hh"

using namespace sdf;

/////////////////////////////////////////////////
class PrintConfig::Implementation
{
  public: bool rotationInDegrees = false;

  public: std::optional<unsigned int> rotationSnapToDegrees = std::nullopt;
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
void PrintConfig::SetRotationSnapToDegrees(unsigned int _value)
{
  SDF_ASSERT((_value > 0 && _value <= 360),
      "interval value to snap to must be larger than 0, "
      "and less than or equal to 360");
  this->dataPtr->rotationSnapToDegrees = _value;
}

/////////////////////////////////////////////////
std::optional<unsigned int> PrintConfig::GetRotationSnapToDegrees() const
{
  return this->dataPtr->rotationSnapToDegrees;
}
