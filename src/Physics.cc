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
#include <ignition/math/Vector3.hh>

#include "sdf/Physics.hh"
#include "Utils.hh"

using namespace sdf;

/// \brief Private data
class sdf::Physics::Implementation
{
  /// \brief Profile name
  public: std::string name {""};

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief True indicates that this is the default profile.
  public: bool isDefault {false};

  /// \brief Dynamics engine type.
  public: std::string type {"ode"};

  /// \brief Maximum step size.
  public: double stepSize {0.001};

  /// \brief Desired realtime factor.
  public: double rtf {1.0};
};

/////////////////////////////////////////////////
Physics::Physics()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Physics::Load(sdf::ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <physics>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "physics")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Physics, but the provided SDF element is not a "
        "<physics>."});
    return errors;
  }

  // Read the physics's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A physics name is required, but the name is not set."});
  }

  // Check that the physics's name is valid
  if (isReservedName(this->dataPtr->name))
  {
    errors.push_back({ErrorCode::RESERVED_NAME,
                     "The supplied physics name [" + this->dataPtr->name +
                     "] is reserved."});
  }

  this->dataPtr->isDefault =
    _sdf->Get<bool>("default", this->dataPtr->isDefault).first;

  std::pair<std::string, bool> stringPair =
    _sdf->Get<std::string>("type", this->dataPtr->type);
  if (!stringPair.second)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "The physics element[" + this->dataPtr->name + "] is missing "
        "child element <type>."});
  }
  this->dataPtr->type = stringPair.first;

  std::pair<double, bool> doublePair =
    _sdf->Get<double>("max_step_size", this->dataPtr->stepSize);
  if (!doublePair.second)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "The physics element[" + this->dataPtr->name + "] is missing "
        "child element <max_step_size>."});
  }
  this->dataPtr->stepSize = doublePair.first;

  doublePair = _sdf->Get<double>("real_time_factor", this->dataPtr->rtf);
  if (!doublePair.second)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "The physics element[" + this->dataPtr->name + "] is missing "
        "child element <real_time_factor>."});
  }
  this->dataPtr->rtf = doublePair.first;

  return errors;
}

/////////////////////////////////////////////////
std::string Physics::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Physics::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
sdf::ElementPtr Physics::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
bool Physics::IsDefault() const
{
  return this->dataPtr->isDefault;
}

/////////////////////////////////////////////////
void Physics::SetDefault(const bool _default)
{
  this->dataPtr->isDefault = _default;
}

/////////////////////////////////////////////////
std::string Physics::EngineType() const
{
  return this->dataPtr->type;
}

/////////////////////////////////////////////////
void Physics::SetEngineType(const std::string &_type)
{
  this->dataPtr->type = _type;
}

/////////////////////////////////////////////////
double Physics::MaxStepSize() const
{
  return this->dataPtr->stepSize;
}

/////////////////////////////////////////////////
void Physics::SetMaxStepSize(const double _step)
{
  this->dataPtr->stepSize = _step;
}

/////////////////////////////////////////////////
double Physics::RealTimeFactor() const
{
  return this->dataPtr->rtf;
}

/////////////////////////////////////////////////
void Physics::SetRealTimeFactor(const double _factor)
{
  this->dataPtr->rtf = _factor;
}
