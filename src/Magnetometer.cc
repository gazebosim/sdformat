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
#include "sdf/Magnetometer.hh"

using namespace sdf;

/// \brief Private magnetometer data.
class sdf::Magnetometer::Implementation
{
  /// \brief The magnetometer noise values.
  public: std::array<Noise, 3> noise;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

//////////////////////////////////////////////////
Magnetometer::Magnetometer()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

//////////////////////////////////////////////////
Errors Magnetometer::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <magnetometer> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "magnetometer")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Magnetometer, but the provided SDF element is "
        "not a <magnetometer>."});
    return errors;
  }

  std::vector<std::string> names = {"x", "y", "z"};

  // Load the noise values.
  for (size_t i = 0; i < names.size(); ++i)
  {
    if (_sdf->HasElement(names[i]))
    {
      sdf::ElementPtr elem = _sdf->GetElement(names[i]);
      if (elem->HasElement("noise"))
        this->dataPtr->noise[i].Load(elem->GetElement("noise"));
    }
  }

  return errors;
}

//////////////////////////////////////////////////
const Noise &Magnetometer::XNoise() const
{
  return this->dataPtr->noise[0];
}

//////////////////////////////////////////////////
void Magnetometer::SetXNoise(const Noise &_noise)
{
  this->dataPtr->noise[0] = _noise;
}

//////////////////////////////////////////////////
const Noise &Magnetometer::YNoise() const
{
  return this->dataPtr->noise[1];
}

//////////////////////////////////////////////////
void Magnetometer::SetYNoise(const Noise &_noise)
{
  this->dataPtr->noise[1] = _noise;
}

//////////////////////////////////////////////////
const Noise &Magnetometer::ZNoise() const
{
  return this->dataPtr->noise[2];
}

//////////////////////////////////////////////////
void Magnetometer::SetZNoise(const Noise &_noise)
{
  this->dataPtr->noise[2] = _noise;
}

//////////////////////////////////////////////////
sdf::ElementPtr Magnetometer::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
bool Magnetometer::operator!=(const Magnetometer &_mag) const
{
  return !(*this == _mag);
}

//////////////////////////////////////////////////
bool Magnetometer::operator==(const Magnetometer &_mag) const
{
  for (size_t i = 0; i < this->dataPtr->noise.size(); ++i)
  {
    if (this->dataPtr->noise[i] != _mag.dataPtr->noise[i])
    {
      return false;
    }
  }

  return true;
}
