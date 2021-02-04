/*
 * Copyright 2019 Open Source Robotics Foundation
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
#include <array>
#include <string>
#include "sdf/Altimeter.hh"

using namespace sdf;

/// \brief Private altimeter data.
class sdf::Altimeter::Implementation
{
  /// \brief The vertical position noise value.
  public: Noise verticalPositionNoise;

  /// \brief The vertical velocity noise value.
  public: Noise verticalVelocityNoise;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

//////////////////////////////////////////////////
Altimeter::Altimeter()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

//////////////////////////////////////////////////
Errors Altimeter::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <altimeter> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "altimeter")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Altimeter, but the provided SDF element is "
        "not a <altimeter>."});
    return errors;
  }

  // Load the noise values.
  if (_sdf->HasElement("vertical_position"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("vertical_position");
    if (elem->HasElement("noise"))
      this->dataPtr->verticalPositionNoise.Load(elem->GetElement("noise"));
  }

  if (_sdf->HasElement("vertical_velocity"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("vertical_velocity");
    if (elem->HasElement("noise"))
      this->dataPtr->verticalVelocityNoise.Load(elem->GetElement("noise"));
  }

  return errors;
}

//////////////////////////////////////////////////
const Noise &Altimeter::VerticalPositionNoise() const
{
  return this->dataPtr->verticalPositionNoise;
}

//////////////////////////////////////////////////
void Altimeter::SetVerticalPositionNoise(const Noise &_noise)
{
  this->dataPtr->verticalPositionNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &Altimeter::VerticalVelocityNoise() const
{
  return this->dataPtr->verticalVelocityNoise;
}

//////////////////////////////////////////////////
void Altimeter::SetVerticalVelocityNoise(const Noise &_noise)
{
  this->dataPtr->verticalVelocityNoise = _noise;
}

//////////////////////////////////////////////////
sdf::ElementPtr Altimeter::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
bool Altimeter::operator!=(const Altimeter &_alt) const
{
  return !(*this == _alt);
}

//////////////////////////////////////////////////
bool Altimeter::operator==(const Altimeter &_alt) const
{
  if (this->dataPtr->verticalPositionNoise !=
      _alt.dataPtr->verticalPositionNoise)
  {
    return false;
  }

  if (this->dataPtr->verticalVelocityNoise !=
      _alt.dataPtr->verticalVelocityNoise)
  {
    return false;
  }

  return true;
}
