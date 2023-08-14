/*
 * Copyright 2018 Open Source Robotics Foundation
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
#include <sstream>
#include <optional>

#include <gz/math/Inertial.hh>
#include "sdf/Cylinder.hh"
#include "sdf/parser.hh"
#include "Utils.hh"

using namespace sdf;

// Private data class
class sdf::Cylinder::Implementation
{
  // A cylinder with a length of 1 meter and radius if 0.5 meters.
  public: gz::math::Cylinderd cylinder{1.0, 0.5};

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Cylinder::Cylinder()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Cylinder::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a cylinder, but the provided SDF "
        "element is null."});
    return errors;
  }

  // We need a cylinder child element
  if (_sdf->GetName() != "cylinder")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a cylinder geometry, but the provided SDF "
        "element is not a <cylinder>."});
    return errors;
  }

  {
    std::pair<double, bool> pair = _sdf->Get<double>(errors, "radius",
        this->dataPtr->cylinder.Radius());

    if (!pair.second)
    {
      std::stringstream ss;
      ss << "Invalid <radius> data for a <cylinder> geometry. "
         << "Using a radius of "
         << this->dataPtr->cylinder.Radius() << ".";
      errors.push_back({ErrorCode::ELEMENT_INVALID, ss.str()});
    }
    this->dataPtr->cylinder.SetRadius(pair.first);
  }

  {
    std::pair<double, bool> pair = _sdf->Get<double>(errors, "length",
        this->dataPtr->cylinder.Length());

    if (!pair.second)
    {
      std::stringstream ss;
      ss << "Invalid <length> data for a <cylinder> geometry. "
         << "Using a length of "
         << this->dataPtr->cylinder.Length() << ".";
      errors.push_back({ErrorCode::ELEMENT_INVALID, ss.str()});
    }
    this->dataPtr->cylinder.SetLength(pair.first);
  }

  return errors;
}

//////////////////////////////////////////////////
double Cylinder::Radius() const
{
  return this->dataPtr->cylinder.Radius();
}

//////////////////////////////////////////////////
void Cylinder::SetRadius(const double _radius)
{
  this->dataPtr->cylinder.SetRadius(_radius);
}

//////////////////////////////////////////////////
double Cylinder::Length() const
{
  return this->dataPtr->cylinder.Length();
}

//////////////////////////////////////////////////
void Cylinder::SetLength(const double _length)
{
  this->dataPtr->cylinder.SetLength(_length);
}

/////////////////////////////////////////////////
sdf::ElementPtr Cylinder::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
const gz::math::Cylinderd &Cylinder::Shape() const
{
  return this->dataPtr->cylinder;
}

/////////////////////////////////////////////////
gz::math::Cylinderd &Cylinder::Shape()
{
  return this->dataPtr->cylinder;
}

std::optional<gz::math::Inertiald> Cylinder::CalculateInertial(double _density)
{
  gz::math::Material material = gz::math::Material(_density);
  this->dataPtr->cylinder.SetMat(material);

  auto cylinderMassMatrix = this->dataPtr->cylinder.MassMatrix();

  if (!cylinderMassMatrix)
  {
    return std::nullopt;
  }
  else
  {
    gz::math::Inertiald cylinderInertial;
    cylinderInertial.SetMassMatrix(cylinderMassMatrix.value());
    return std::make_optional(cylinderInertial);
  }
}

/////////////////////////////////////////////////
sdf::ElementPtr Cylinder::ToElement() const
{
  sdf::Errors errors;
  auto result = this->ToElement(errors);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
sdf::ElementPtr Cylinder::ToElement(sdf::Errors &_errors) const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("cylinder_shape.sdf", elem);

  sdf::ElementPtr radiusElem = elem->GetElement("radius", _errors);
  radiusElem->Set<double>(_errors, this->Radius());

  sdf::ElementPtr lengthElem = elem->GetElement("length", _errors);
  lengthElem->Set<double>(_errors, this->Length());

  return elem;
}
