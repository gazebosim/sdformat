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
#include "sdf/Cylinder.hh"

using namespace sdf;

// Private data class
class sdf::Cylinder::Implementation
{
  // A cylinder with a length of 1 meter and radius if 0.5 meters.
  public: ignition::math::Cylinderd cylinder{1.0, 0.5};

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Cylinder::Cylinder()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
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
    std::pair<double, bool> pair = _sdf->Get<double>("radius",
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
    std::pair<double, bool> pair = _sdf->Get<double>("length",
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
const ignition::math::Cylinderd &Cylinder::Shape() const
{
  return this->dataPtr->cylinder;
}

/////////////////////////////////////////////////
ignition::math::Cylinderd &Cylinder::Shape()
{
  return this->dataPtr->cylinder;
}
