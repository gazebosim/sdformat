/*
 * Copyright 2020 Open Source Robotics Foundation
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

#include <gz/math/Material.hh>
#include <gz/math/Inertial.hh>
#include "sdf/Capsule.hh"
#include "sdf/Console.hh"
#include "sdf/parser.hh"
#include "Utils.hh"

using namespace sdf;

// Private data class
class sdf::Capsule::Implementation
{
  // A capsule with a length of 1 meter and radius if 0.5 meters.
  public: gz::math::Capsuled capsule{1.0, 0.5};

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Capsule::Capsule()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Capsule::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a capsule, but the provided SDF "
        "element is null."});
    return errors;
  }

  // We need a capsule child element
  if (_sdf->GetName() != "capsule")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a capsule geometry, but the provided SDF "
        "element is not a <capsule>."});
    return errors;
  }

  {
    std::pair<double, bool> pair = _sdf->Get<double>(errors, "radius",
        this->dataPtr->capsule.Radius());

    if (!pair.second)
    {
      std::stringstream ss;
      ss << "Invalid <radius> data for a <capsule> geometry. "
         << "Using a radius of "
         << this->dataPtr->capsule.Radius() << ".";
      errors.push_back({ErrorCode::ELEMENT_INVALID, ss.str()});
    }
    else
    {
      if (pair.first <= 0)
      {
        sdfwarn << "Value of <radius> is negative. "
            << "Using default value of 0.5.\n";
        pair.first = 0.5;
      }
      this->dataPtr->capsule.SetRadius(pair.first);
    }
  }

  {
    std::pair<double, bool> pair = _sdf->Get<double>(errors, "length",
        this->dataPtr->capsule.Length());

    if (!pair.second)
    {
      std::stringstream ss;
      ss << "Invalid <length> data for a <capsule> geometry. "
         << "Using a length of "
         << this->dataPtr->capsule.Length() << ".";
      errors.push_back({ErrorCode::ELEMENT_INVALID, ss.str()});
    }
    else
    {
      if (pair.first <= 0)
      {
        sdfwarn << "Value of <length> is negative. "
            << "Using default value of 1.\n";
        pair.first = 1.0;
      }
      this->dataPtr->capsule.SetLength(pair.first);
    }
  }

  return errors;
}

//////////////////////////////////////////////////
double Capsule::Radius() const
{
  return this->dataPtr->capsule.Radius();
}

//////////////////////////////////////////////////
void Capsule::SetRadius(const double _radius)
{
  this->dataPtr->capsule.SetRadius(_radius);
}

//////////////////////////////////////////////////
double Capsule::Length() const
{
  return this->dataPtr->capsule.Length();
}

//////////////////////////////////////////////////
void Capsule::SetLength(const double _length)
{
  this->dataPtr->capsule.SetLength(_length);
}

/////////////////////////////////////////////////
sdf::ElementPtr Capsule::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
const gz::math::Capsuled &Capsule::Shape() const
{
  return this->dataPtr->capsule;
}

/////////////////////////////////////////////////
gz::math::Capsuled &Capsule::Shape()
{
  return this->dataPtr->capsule;
}

/////////////////////////////////////////////////
std::optional<gz::math::Inertiald> Capsule::CalculateInertial(double _density)
{
  gz::math::Material material = gz::math::Material(_density);
  this->dataPtr->capsule.SetMat(material);

  auto capsuleMassMatrix = this->dataPtr->capsule.MassMatrix();

  if(!capsuleMassMatrix)
  {
    return std::nullopt;
  }
  else
  {
    gz::math::Inertiald capsuleInertial;
    capsuleInertial.SetMassMatrix(capsuleMassMatrix.value());
    return std::make_optional(capsuleInertial);
  }
}

/////////////////////////////////////////////////
gz::math::AxisAlignedBox Capsule::AxisAlignedBox() const
{
  auto halfSize = this->Radius() * gz::math::Vector3d::One +
    this->Length() / 2 * gz::math::Vector3d::UnitZ;
  return gz::math::AxisAlignedBox(-halfSize, halfSize);
}

/////////////////////////////////////////////////
sdf::ElementPtr Capsule::ToElement() const
{
  sdf::Errors errors;
  auto result = this->ToElement(errors);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
sdf::ElementPtr Capsule::ToElement(sdf::Errors &_errors) const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("capsule_shape.sdf", elem);

  sdf::ElementPtr radiusElem = elem->GetElement("radius", _errors);
  radiusElem->Set(_errors, this->Radius());

  sdf::ElementPtr lengthElem = elem->GetElement("length", _errors);
  lengthElem->Set(_errors, this->Length());

  return elem;
}
