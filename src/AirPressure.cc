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
#include "sdf/AirPressure.hh"
#include "sdf/parser.hh"
#include "Utils.hh"

using namespace sdf;

/// \brief Private airPressure data.
class sdf::AirPressure::Implementation
{
  /// \brief The pressure noise.
  public: Noise noise;

  /// \brief The reference altitude.
  public: double referenceAltitude = 0.0;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

//////////////////////////////////////////////////
AirPressure::AirPressure()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

//////////////////////////////////////////////////
Errors AirPressure::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <airPressure> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "air_pressure")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load an Air Pressure Sensor, but the provided SDF "
        "element is not a <air_pressure>."});
    return errors;
  }

  // Load the noise values.
  if (_sdf->HasElement("pressure"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("pressure", errors);
    if (elem->HasElement("noise"))
      this->dataPtr->noise.Load(elem->GetElement("noise", errors));
  }

  this->dataPtr->referenceAltitude = _sdf->Get<double>(
      errors, "reference_altitude", this->dataPtr->referenceAltitude).first;

  return errors;
}

//////////////////////////////////////////////////
sdf::ElementPtr AirPressure::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
bool AirPressure::operator!=(const AirPressure &_air) const
{
  return !(*this == _air);
}

//////////////////////////////////////////////////
bool AirPressure::operator==(const AirPressure &_air) const
{
  return this->dataPtr->noise == _air.dataPtr->noise &&
         gz::math::equal(this->dataPtr->referenceAltitude,
                               _air.dataPtr->referenceAltitude);
}

//////////////////////////////////////////////////
double AirPressure::ReferenceAltitude() const
{
  return this->dataPtr->referenceAltitude;
}

//////////////////////////////////////////////////
void AirPressure::SetReferenceAltitude(double _ref)
{
  this->dataPtr->referenceAltitude = _ref;
}

//////////////////////////////////////////////////
const Noise &AirPressure::PressureNoise() const
{
  return this->dataPtr->noise;
}

//////////////////////////////////////////////////
void AirPressure::SetPressureNoise(const Noise &_noise)
{
  this->dataPtr->noise = _noise;
}

/////////////////////////////////////////////////
sdf::ElementPtr AirPressure::ToElement() const
{
  sdf::Errors errors;
  auto result = this->ToElement(errors);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
sdf::ElementPtr AirPressure::ToElement(sdf::Errors &_errors) const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile(std::string(this->SchemaFile()), elem);

  elem->GetElement("reference_altitude", _errors)->Set<double>(
      _errors, this->ReferenceAltitude());
  sdf::ElementPtr pressureElem = elem->GetElement("pressure", _errors);
  sdf::ElementPtr noiseElem = pressureElem->GetElement("noise", _errors);
  noiseElem->Copy(this->dataPtr->noise.ToElement(_errors), _errors);

  return elem;
}

/////////////////////////////////////////////////
inline std::string_view AirPressure::SchemaFile() 
{
    static char kSchemaFile[] = "air_pressure.sdf";
    return kSchemaFile;
}

