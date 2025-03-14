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
#include <optional>
#include <gz/math/Inertial.hh>
#include <gz/math/Material.hh>

#include "sdf/parser.hh"
#include "sdf/Sphere.hh"
#include "Utils.hh"

using namespace sdf;

// Private data class
class sdf::Sphere::Implementation
{
  /// \brief Representation of the sphere
  public: gz::math::Sphered sphere{1.0};

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Sphere::Sphere()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Sphere::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a sphere, but the provided SDF element is null."});
    return errors;
  }

  // We need a sphere element
  if (_sdf->GetName() != "sphere")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a sphere geometry, but the provided SDF "
        "element is not a <sphere>."});
    return errors;
  }

  if (_sdf->HasElement("radius"))
  {
    std::pair<double, bool> pair = _sdf->Get<double>(errors, "radius",
        this->dataPtr->sphere.Radius());

    if (!pair.second)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Invalid <radius> data for a <sphere> geometry. "
          "Using a radius of 1.0."});
    }
    this->dataPtr->sphere.SetRadius(pair.first);
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Sphere geometry is missing a <radius> child element. "
        "Using a radius of 1.0."});
  }

  return errors;
}

//////////////////////////////////////////////////
double Sphere::Radius() const
{
  return this->dataPtr->sphere.Radius();
}

//////////////////////////////////////////////////
void Sphere::SetRadius(const double _radius)
{
  this->dataPtr->sphere.SetRadius(_radius);
}

/////////////////////////////////////////////////
const gz::math::Sphered &Sphere::Shape() const
{
  return this->dataPtr->sphere;
}

/////////////////////////////////////////////////
gz::math::Sphered &Sphere::Shape()
{
  return this->dataPtr->sphere;
}


/////////////////////////////////////////////////
sdf::ElementPtr Sphere::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
std::optional<gz::math::Inertiald> Sphere::CalculateInertial(double _density)
{
  gz::math::Material material = gz::math::Material(_density);
  this->dataPtr->sphere.SetMaterial(material);

  auto sphereMassMatrix = this->dataPtr->sphere.MassMatrix();

  if (!sphereMassMatrix)
  {
    return std::nullopt;
  }
  else
  {
    gz::math::Inertiald sphereInertial;
    sphereInertial.SetMassMatrix(sphereMassMatrix.value());
    return std::make_optional(sphereInertial);
  }
}

/////////////////////////////////////////////////
gz::math::AxisAlignedBox Sphere::AxisAlignedBox() const
{
  auto halfSize = this->Radius() * gz::math::Vector3d::One;
  return gz::math::AxisAlignedBox(-halfSize, halfSize);
}

/////////////////////////////////////////////////
sdf::ElementPtr Sphere::ToElement() const
{
  sdf::Errors errors;
  auto result = this->ToElement(errors);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
sdf::ElementPtr Sphere::ToElement(sdf::Errors &_errors) const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("sphere_shape.sdf", elem);

  sdf::ElementPtr radiusElem = elem->GetElement("radius", _errors);
  radiusElem->Set<double>(_errors, this->Radius());

  return elem;
}
