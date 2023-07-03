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
#include <sstream>
#include "sdf/Ellipsoid.hh"
#include "sdf/parser.hh"

using namespace sdf;

// Private data class
class sdf::Ellipsoid::Implementation
{
  /// \brief An ellipsoid with all three radii of 1 meter
  public: gz::math::Ellipsoidd ellipsoid{gz::math::Vector3d::One};

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Ellipsoid::Ellipsoid()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Ellipsoid::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a ellipsoid, but the provided SDF "
        "element is null."});
    return errors;
  }

  // We need a ellipsoid child element
  if (_sdf->GetName() != "ellipsoid")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a ellipsoid geometry, but the provided SDF "
        "element is not a <ellipsoid>."});
    return errors;
  }

  if (_sdf->HasElement("radii"))
  {
    std::pair<gz::math::Vector3d, bool> pair =
      _sdf->Get<gz::math::Vector3d>(
        "radii", this->dataPtr->ellipsoid.Radii());

    if (!pair.second)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Invalid <radii> data for a <ellipsoid> geometry. "
          "Using a radii of 1, 1, 1 "});
    }
    this->dataPtr->ellipsoid.SetRadii(pair.first);
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Ellipsoid geometry is missing a <radii> child element. "
        "Using a radii of 1, 1, 1."});
  }

  return errors;
}

//////////////////////////////////////////////////
gz::math::Vector3d Ellipsoid::Radii() const
{
  return this->dataPtr->ellipsoid.Radii();
}

//////////////////////////////////////////////////
void Ellipsoid::SetRadii(const gz::math::Vector3d &_radii)
{
  this->dataPtr->ellipsoid.SetRadii(_radii);
}

/////////////////////////////////////////////////
sdf::ElementPtr Ellipsoid::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
const gz::math::Ellipsoidd &Ellipsoid::Shape() const
{
  return this->dataPtr->ellipsoid;
}

/////////////////////////////////////////////////
gz::math::Ellipsoidd &Ellipsoid::Shape()
{
  return this->dataPtr->ellipsoid;
}

/////////////////////////////////////////////////
std::optional< gz::math::MassMatrix3d > Ellipsoid::MassMatrix(const double _density)
{
  gz::math::Material material = gz::math::Material(_density);
  this->dataPtr->ellipsoid.SetMat(material);
  return this->dataPtr->ellipsoid.MassMatrix();
}

/////////////////////////////////////////////////
sdf::ElementPtr Ellipsoid::ToElement() const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("ellipsoid_shape.sdf", elem);

  sdf::ElementPtr radiiElem = elem->GetElement("radii");
  radiiElem->Set(this->Radii());

  return elem;
}
