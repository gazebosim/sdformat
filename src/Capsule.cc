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
#include "sdf/Capsule.hh"

using namespace sdf;

// Private data class
class sdf::CapsulePrivate
{
  // A capsule with a length of 1 meter and radius if 0.5 meters.
  public: ignition::math::Capsuled capsule{1.0, 0.5};

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Capsule::Capsule()
  : dataPtr(new CapsulePrivate)
{
}

/////////////////////////////////////////////////
Capsule::~Capsule()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
Capsule::Capsule(const Capsule &_capsule)
  : dataPtr(new CapsulePrivate)
{
  this->dataPtr->capsule = _capsule.dataPtr->capsule;
  this->dataPtr->sdf = _capsule.dataPtr->sdf;
}

//////////////////////////////////////////////////
Capsule::Capsule(Capsule &&_capsule) noexcept
  : dataPtr(std::exchange(_capsule.dataPtr, nullptr))
{
}

/////////////////////////////////////////////////
Capsule &Capsule::operator=(const Capsule &_capsule)
{
  return *this = Capsule(_capsule);
}

/////////////////////////////////////////////////
Capsule &Capsule::operator=(Capsule &&_capsule)
{
  std::swap(this->dataPtr, _capsule.dataPtr);
  return *this;
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

  if (_sdf->HasElement("radius"))
  {
    std::pair<double, bool> pair = _sdf->Get<double>("radius",
        this->dataPtr->capsule.Radius());

    if (!pair.second)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Invalid <radius> data for a <capsule> geometry. "
          "Using a radius of 1."});
    }
    this->dataPtr->capsule.SetRadius(pair.first);
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Capsule geometry is missing a <radius> child element. "
        "Using a radius of 1."});
  }

  if (_sdf->HasElement("length"))
  {
    std::pair<double, bool> pair = _sdf->Get<double>("length",
        this->dataPtr->capsule.Length());

    if (!pair.second)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Invalid <length> data for a <capsule> geometry. "
          "Using a length of 1."});
    }
    this->dataPtr->capsule.SetLength(pair.first);
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Capsule geometry is missing a <length> child element. "
        "Using a length of 1."});
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
const ignition::math::Capsuled &Capsule::Shape() const
{
  return this->dataPtr->capsule;
}

/////////////////////////////////////////////////
ignition::math::Capsuled &Capsule::Shape()
{
  return this->dataPtr->capsule;
}
