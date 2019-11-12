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
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>
#include "sdf/Plane.hh"

using namespace sdf;

// Private data class
class sdf::PlanePrivate
{
  // Plane normal
  public: ignition::math::Vector3d normal = ignition::math::Vector3d::UnitZ;

  // Size of the plane
  public: ignition::math::Vector2d size = ignition::math::Vector2d::One;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Plane::Plane()
  : dataPtr(new PlanePrivate)
{
}

/////////////////////////////////////////////////
Plane::~Plane()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
Plane::Plane(const Plane &_plane)
  : dataPtr(new PlanePrivate)
{
  this->dataPtr->normal = _plane.dataPtr->normal;
  this->dataPtr->size = _plane.dataPtr->size;
  this->dataPtr->sdf = _plane.dataPtr->sdf;
}

/////////////////////////////////////////////////
Plane &Plane::operator=(const Plane &_plane)
{
  if (!this->dataPtr)
  {
    this->dataPtr = new PlanePrivate;
  }
  this->dataPtr->normal = _plane.dataPtr->normal;
  this->dataPtr->size = _plane.dataPtr->size;
  this->dataPtr->sdf = _plane.dataPtr->sdf;
  return *this;
}

//////////////////////////////////////////////////
Plane::Plane(Plane &&_plane) noexcept
{
  this->dataPtr = _plane.dataPtr;
  _plane.dataPtr = nullptr;
}

/////////////////////////////////////////////////
Plane &Plane::operator=(Plane &&_plane)
{
  this->dataPtr = _plane.dataPtr;
  _plane.dataPtr = nullptr;
  return *this;
}

/////////////////////////////////////////////////
Errors Plane::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a plane, but the provided SDF "
        "element is null."});
    return errors;
  }

  // We need a plane element
  if (_sdf->GetName() != "plane")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a plane geometry, but the provided SDF "
        "element is not a <plane>."});
    return errors;
  }

  if (_sdf->HasElement("normal"))
  {
    std::pair<ignition::math::Vector3d, bool> pair =
      _sdf->Get<ignition::math::Vector3d>("normal", this->dataPtr->normal);

    if (!pair.second)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Invalid <normal> data for a <plane> geometry. "
          "Using a normal of 0, 0, 1."});
    }
    this->SetNormal(pair.first);
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Plane geometry is missing a <normal> child element. "
        "Using a normal of 0, 0, 1."});
  }

  if (_sdf->HasElement("size"))
  {
    std::pair<ignition::math::Vector2d, bool> pair =
      _sdf->Get<ignition::math::Vector2d>("size", this->dataPtr->size);

    if (!pair.second)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Invalid <size> data for a <plane> geometry. "
          "Using a size of 1, 1."});
    }
    this->dataPtr->size = pair.first;
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Plane geometry is missing a <size> child element. "
        "Using a size of 1, 1."});
  }

  return errors;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Plane::Normal() const
{
  return this->dataPtr->normal;
}

//////////////////////////////////////////////////
void Plane::SetNormal(const ignition::math::Vector3d &_normal)
{
  this->dataPtr->normal = _normal;
  this->dataPtr->normal.Normalize();
}

//////////////////////////////////////////////////
ignition::math::Vector2d Plane::Size() const
{
  return this->dataPtr->size;
}

//////////////////////////////////////////////////
void Plane::SetSize(const ignition::math::Vector2d &_size)
{
  this->dataPtr->size = _size;
}

/////////////////////////////////////////////////
sdf::ElementPtr Plane::Element() const
{
  return this->dataPtr->sdf;
}
