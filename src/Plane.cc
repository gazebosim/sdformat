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
#include "sdf/Geometry.hh"
#include "sdf/Plane.hh"

using namespace sdf;

// Private data class
class sdf::PlanePrivate
{
  // Plane normal
  public: ignition::math::Vector3d normal = ignition::math::Vector3d::UnitZ;

  // Size of the plane
  public: ignition::math::Vector2d size = ignition::math::Vector2d::One;
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

/////////////////////////////////////////////////
Errors Plane::Load(ElementPtr _sdf)
{
  Errors errors;

  // Check that the provided SDF element is a <geometry>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "geometry")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Geometry, but the provided SDF element is not a "
        "<geometry>."});
    return errors;
  }

  // We need a plane child element
  if (!_sdf->HasElement("plane"))
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a plane geometry, but the provided <geometry> "
        "SDF element does not contain a <plane> element."});
    return errors;
  }

  ElementPtr sdf = _sdf->GetElement("plane");

  if (sdf->HasElement("normal"))
  {
    std::pair<ignition::math::Vector3d, bool> pair =
      sdf->Get<ignition::math::Vector3d>("normal", this->dataPtr->normal);

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

  if (sdf->HasElement("size"))
  {
    std::pair<ignition::math::Vector2d, bool> pair =
      sdf->Get<ignition::math::Vector2d>("size", this->dataPtr->size);

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
