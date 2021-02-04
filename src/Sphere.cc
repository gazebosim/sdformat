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
#include "sdf/Sphere.hh"

using namespace sdf;

// Private data class
class sdf::Sphere::Implementation
{
  /// \brief Representation of the sphere
  public: ignition::math::Sphered sphere{1.0};

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Sphere::Sphere()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
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
    std::pair<double, bool> pair = _sdf->Get<double>("radius",
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
const ignition::math::Sphered &Sphere::Shape() const
{
  return this->dataPtr->sphere;
}

/////////////////////////////////////////////////
ignition::math::Sphered &Sphere::Shape()
{
  return this->dataPtr->sphere;
}


/////////////////////////////////////////////////
sdf::ElementPtr Sphere::Element() const
{
  return this->dataPtr->sdf;
}
