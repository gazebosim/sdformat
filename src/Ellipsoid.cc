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

using namespace sdf;

// Private data class
class sdf::EllipsoidPrivate
{
  // An ellipsoid with a three radiuses of 1 meter
  public: ignition::math::Ellipsoidd ellipsoid{ignition::math::Vector3d::One};

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Ellipsoid::Ellipsoid()
  : dataPtr(new EllipsoidPrivate)
{
}

/////////////////////////////////////////////////
Ellipsoid::~Ellipsoid()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
Ellipsoid::Ellipsoid(const Ellipsoid &_ellipsoid)
  : dataPtr(new EllipsoidPrivate)
{
  *this->dataPtr = *_ellipsoid.dataPtr;
}

//////////////////////////////////////////////////
Ellipsoid::Ellipsoid(Ellipsoid &&_ellipsoid) noexcept
  : dataPtr(std::exchange(_ellipsoid.dataPtr, nullptr))
{
}

/////////////////////////////////////////////////
Ellipsoid &Ellipsoid::operator=(const Ellipsoid &_ellipsoid)
{
  return *this = Ellipsoid(_ellipsoid);
}

/////////////////////////////////////////////////
Ellipsoid &Ellipsoid::operator=(Ellipsoid &&_ellipsoid)
{
  std::swap(this->dataPtr, _ellipsoid.dataPtr);
  return *this;
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
    std::pair<ignition::math::Vector3d, bool> pair =
      _sdf->Get<ignition::math::Vector3d>("radii", this->dataPtr->ellipsoid.Radii());

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
ignition::math::Vector3d Ellipsoid::Radii() const
{
  return this->dataPtr->ellipsoid.Radii();
}

//////////////////////////////////////////////////
void Ellipsoid::SetRadii(const ignition::math::Vector3d &_radii)
{
  this->dataPtr->ellipsoid.SetRadii(_radii);
}

/////////////////////////////////////////////////
sdf::ElementPtr Ellipsoid::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
const ignition::math::Ellipsoidd &Ellipsoid::Shape() const
{
  return this->dataPtr->ellipsoid;
}

/////////////////////////////////////////////////
ignition::math::Ellipsoidd &Ellipsoid::Shape()
{
  return this->dataPtr->ellipsoid;
}
