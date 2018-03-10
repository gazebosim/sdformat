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
#include "sdf/Geometry.hh"
#include "sdf/Cylinder.hh"

using namespace sdf;

// Private data class
class sdf::CylinderPrivate
{
  // Radius of the cylinder
  public: double radius = 0.5;

  // Length of the cylinder
  public: double length = 1.0;
};

/////////////////////////////////////////////////
Cylinder::Cylinder()
  : dataPtr(new CylinderPrivate)
{
  this->SetType(GeometryType::CYLINDER);
}

/////////////////////////////////////////////////
Cylinder::~Cylinder()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
Errors Cylinder::Load(ElementPtr _sdf)
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

  // We need a cylinder child element
  if (!_sdf->HasElement("cylinder"))
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a cylinder geometry, but the provided <geometry> "
        "SDF element does not contain a <cylinder> element."});
    return errors;
  }

  ElementPtr sdf = _sdf->GetElement("cylinder");

  if (sdf->HasElement("radius"))
  {
    std::pair<double, bool> pair = sdf->Get<double>("radius",
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

  if (sdf->HasElement("length"))
  {
    std::pair<double, bool> pair = sdf->Get<double>("length",
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
