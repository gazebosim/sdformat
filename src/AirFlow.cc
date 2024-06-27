/*
 * Copyright 2023 Open Source Robotics Foundation
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

#include "sdf/AirFlow.hh"
#include "sdf/parser.hh"

using namespace sdf;

/// \brief Private AirFlow data.
class sdf::AirFlow::Implementation
{
  /// \brief The speed noise.
  public: Noise speed_noise;

  /// \brief The direction noise.
  public: Noise direction_noise;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

//////////////////////////////////////////////////
AirFlow::AirFlow()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

//////////////////////////////////////////////////
Errors AirFlow::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <air_flow> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "air_flow")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load an Air Pressure Sensor, but the provided SDF "
        "element is not a <air_flow>."});
    return errors;
  }

  // Load the noise values.
  if (_sdf->HasElement("speed"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("speed");
    if (elem->HasElement("noise"))
      this->dataPtr->speed_noise.Load(elem->GetElement("noise"));
  }

  // Load the noise values.
  if (_sdf->HasElement("direction"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("direction");
    if (elem->HasElement("noise"))
      this->dataPtr->direction_noise.Load(elem->GetElement("noise"));
  }

  return errors;
}

//////////////////////////////////////////////////
sdf::ElementPtr AirFlow::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
bool AirFlow::operator!=(const AirFlow &_air) const
{
  return !(*this == _air);
}

//////////////////////////////////////////////////
bool AirFlow::operator==(const AirFlow &_air) const
{
  bool speed_noise = this->dataPtr->speed_noise ==
                    _air.dataPtr->speed_noise;
  bool dir_noise = this->dataPtr->direction_noise ==
                    _air.dataPtr->direction_noise;

  return speed_noise && dir_noise;
}

//////////////////////////////////////////////////
const Noise &AirFlow::SpeedNoise() const
{
  return this->dataPtr->speed_noise;
}

//////////////////////////////////////////////////
void AirFlow::SetSpeedNoise(const Noise &_noise)
{
  this->dataPtr->speed_noise = _noise;
}

//////////////////////////////////////////////////
const Noise &AirFlow::DirectionNoise() const
{
  return this->dataPtr->direction_noise;
}

//////////////////////////////////////////////////
void AirFlow::SetDirectionNoise(const Noise &_noise)
{
  this->dataPtr->direction_noise = _noise;
}

/////////////////////////////////////////////////
sdf::ElementPtr AirFlow::ToElement() const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("air_flow.sdf", elem);

  sdf::ElementPtr speedElem = elem->GetElement("speed");
  sdf::ElementPtr speedNoiseElem = speedElem->GetElement("noise");
  speedNoiseElem->Copy(this->dataPtr->speed_noise.ToElement());

  sdf::ElementPtr directionElem = elem->GetElement("direction");
  sdf::ElementPtr directionNoiseElem = directionElem->GetElement("noise");
  directionNoiseElem->Copy(this->dataPtr->direction_noise.ToElement());

  return elem;
}
