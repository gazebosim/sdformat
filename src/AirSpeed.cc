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

#include "sdf/AirSpeed.hh"
#include "sdf/parser.hh"

using namespace sdf;

/// \brief Private AirSpeed data.
class sdf::AirSpeed::Implementation
{
  /// \brief The differential pressure noise.
  public: Noise noise;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

//////////////////////////////////////////////////
AirSpeed::AirSpeed()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

//////////////////////////////////////////////////
Errors AirSpeed::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <air_speed> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "air_speed")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load an Air Pressure Sensor, but the provided SDF "
        "element is not a <air_speed>."});
    return errors;
  }

  // Load the noise values.
  if (_sdf->HasElement("pressure"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("pressure");
    if (elem->HasElement("noise"))
      this->dataPtr->noise.Load(elem->GetElement("noise"));
  }

  return errors;
}

//////////////////////////////////////////////////
sdf::ElementPtr AirSpeed::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
bool AirSpeed::operator!=(const AirSpeed &_air) const
{
  return !(*this == _air);
}

//////////////////////////////////////////////////
bool AirSpeed::operator==(const AirSpeed &_air) const
{
  return this->dataPtr->noise == _air.dataPtr->noise;
}

//////////////////////////////////////////////////
const Noise &AirSpeed::PressureNoise() const
{
  return this->dataPtr->noise;
}

//////////////////////////////////////////////////
void AirSpeed::SetPressureNoise(const Noise &_noise)
{
  this->dataPtr->noise = _noise;
}

/////////////////////////////////////////////////
sdf::ElementPtr AirSpeed::ToElement() const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("air_speed.sdf", elem);

  sdf::ElementPtr pressureElem = elem->GetElement("pressure");
  sdf::ElementPtr noiseElem = pressureElem->GetElement("noise");
  noiseElem->Copy(this->dataPtr->noise.ToElement());

  return elem;
}

/////////////////////////////////////////////////
inline std::string_view AirSpeed::SchemaFile() 
{
    static char kSchemaFile[] = "air_speed.sdf";
    return kSchemaFile;
}

