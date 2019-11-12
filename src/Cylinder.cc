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
#include "sdf/Cylinder.hh"

using namespace sdf;

// Private data class
class sdf::CylinderPrivate
{
  // Radius of the cylinder
  public: double radius = 1.0;

  // Length of the cylinder
  public: double length = 1.0;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Cylinder::Cylinder()
  : dataPtr(new CylinderPrivate)
{
}

/////////////////////////////////////////////////
Cylinder::~Cylinder()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
Cylinder::Cylinder(const Cylinder &_cylinder)
  : dataPtr(new CylinderPrivate)
{
  this->dataPtr->radius = _cylinder.dataPtr->radius;
  this->dataPtr->length = _cylinder.dataPtr->length;
  this->dataPtr->sdf = _cylinder.dataPtr->sdf;
}

/////////////////////////////////////////////////
Cylinder &Cylinder::operator=(const Cylinder &_cylinder)
{
  if (!this->dataPtr)
  {
    this->dataPtr = new CylinderPrivate;
  }
  this->dataPtr->radius = _cylinder.dataPtr->radius;
  this->dataPtr->length = _cylinder.dataPtr->length;
  this->dataPtr->sdf = _cylinder.dataPtr->sdf;
  return *this;
}

//////////////////////////////////////////////////
Cylinder::Cylinder(Cylinder &&_cylinder) noexcept
{
  this->dataPtr = _cylinder.dataPtr;
  _cylinder.dataPtr = nullptr;
}

/////////////////////////////////////////////////
Cylinder &Cylinder::operator=(Cylinder &&_cylinder)
{
  this->dataPtr = _cylinder.dataPtr;
  _cylinder.dataPtr = nullptr;
  return *this;
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

  if (_sdf->HasElement("radius"))
  {
    std::pair<double, bool> pair = _sdf->Get<double>("radius",
        this->dataPtr->radius);

    if (!pair.second)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Invalid <radius> data for a <cylinder> geometry. "
          "Using a radius of 1."});
    }
    this->dataPtr->radius = pair.first;
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Cylinder geometry is missing a <radius> child element. "
        "Using a radius of 1."});
  }

  if (_sdf->HasElement("length"))
  {
    std::pair<double, bool> pair = _sdf->Get<double>("length",
        this->dataPtr->length);

    if (!pair.second)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Invalid <length> data for a <cylinder> geometry. "
          "Using a length of 1."});
    }
    this->dataPtr->length = pair.first;
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Cylinder geometry is missing a <length> child element. "
        "Using a length of 1."});
  }

  return errors;
}

//////////////////////////////////////////////////
double Cylinder::Radius() const
{
  return this->dataPtr->radius;
}

//////////////////////////////////////////////////
void Cylinder::SetRadius(const double _radius)
{
  this->dataPtr->radius = _radius;
}

//////////////////////////////////////////////////
double Cylinder::Length() const
{
  return this->dataPtr->length;
}

//////////////////////////////////////////////////
void Cylinder::SetLength(const double _length)
{
  this->dataPtr->length = _length;
}

/////////////////////////////////////////////////
sdf::ElementPtr Cylinder::Element() const
{
  return this->dataPtr->sdf;
}
