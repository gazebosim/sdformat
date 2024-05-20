/*
 * Copyright 2024 CogniPilot Foundation
 * Copyright 2024 Open Source Robotics Foundation
 * Copyright 2024 Rudis Laboratories
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
#include <optional>
#include <sstream>
#include <utility>

#include <gz/math/Inertial.hh>
#include <gz/math/Cone.hh>
#include "sdf/Cone.hh"
#include "sdf/parser.hh"
#include "Utils.hh"

using namespace sdf;

// Private data class
class sdf::Cone::Implementation
{
  // A cone with a length of 1 meter and radius if 0.5 meters.
  public: gz::math::Coned cone{1.0, 0.5};

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Cone::Cone()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Cone::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a cone, but the provided SDF "
        "element is null."});
    return errors;
  }

  // We need a cone child element
  if (_sdf->GetName() != "cone")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a cone geometry, but the provided SDF "
        "element is not a <cone>."});
    return errors;
  }

  {
    std::pair<double, bool> pair = _sdf->Get<double>(errors, "radius",
        this->dataPtr->cone.Radius());

    if (!pair.second)
    {
      std::stringstream ss;
      ss << "Invalid <radius> data for a <cone> geometry. "
         << "Using a radius of "
         << this->dataPtr->cone.Radius() << ".";
      errors.push_back({ErrorCode::ELEMENT_INVALID, ss.str()});
    }
    this->dataPtr->cone.SetRadius(pair.first);
  }

  {
    std::pair<double, bool> pair = _sdf->Get<double>(errors, "length",
        this->dataPtr->cone.Length());

    if (!pair.second)
    {
      std::stringstream ss;
      ss << "Invalid <length> data for a <cone> geometry. "
         << "Using a length of "
         << this->dataPtr->cone.Length() << ".";
      errors.push_back({ErrorCode::ELEMENT_INVALID, ss.str()});
    }
    this->dataPtr->cone.SetLength(pair.first);
  }

  return errors;
}

//////////////////////////////////////////////////
double Cone::Radius() const
{
  return this->dataPtr->cone.Radius();
}

//////////////////////////////////////////////////
void Cone::SetRadius(double _radius)
{
  this->dataPtr->cone.SetRadius(_radius);
}

//////////////////////////////////////////////////
double Cone::Length() const
{
  return this->dataPtr->cone.Length();
}

//////////////////////////////////////////////////
void Cone::SetLength(double _length)
{
  this->dataPtr->cone.SetLength(_length);
}

/////////////////////////////////////////////////
sdf::ElementPtr Cone::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
const gz::math::Coned &Cone::Shape() const
{
  return this->dataPtr->cone;
}

/////////////////////////////////////////////////
gz::math::Coned &Cone::Shape()
{
  return this->dataPtr->cone;
}

std::optional<gz::math::Inertiald> Cone::CalculateInertial(double _density)
{
  gz::math::Material material = gz::math::Material(_density);
  this->dataPtr->cone.SetMat(material);

  auto coneMassMatrix = this->dataPtr->cone.MassMatrix();

  if (!coneMassMatrix)
  {
    return std::nullopt;
  }
  else
  {
    gz::math::Inertiald coneInertial;
    coneInertial.SetMassMatrix(coneMassMatrix.value());
    return std::make_optional(coneInertial);
  }
}

/////////////////////////////////////////////////
sdf::ElementPtr Cone::ToElement() const
{
  sdf::Errors errors;
  auto result = this->ToElement(errors);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
sdf::ElementPtr Cone::ToElement(sdf::Errors &_errors) const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("cone_shape.sdf", elem);

  sdf::ElementPtr radiusElem = elem->GetElement("radius", _errors);
  radiusElem->Set<double>(_errors, this->Radius());

  sdf::ElementPtr lengthElem = elem->GetElement("length", _errors);
  lengthElem->Set<double>(_errors, this->Length());

  return elem;
}
