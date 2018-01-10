/*
 * Copyright 2017 Open Source Robotics Foundation
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
#include <iostream>

#include "sdf/World.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::WorldPrivate
{
  /// \brief Name of the world.
  public: std::string name = "";

  /// \brief Audio device name
  public: std::string audioDevice = "default";

  /// \brief Linear velocity of wind.
  public: ignition::math::Vector3d windLinearVelocity =
           ignition::math::Vector3d::Zero;

  /// \brief Gravity vector.
  public: ignition::math::Vector3d gravity =
           ignition::math::Vector3d(0, 0, -9.80665);

  /// \brief Magnetic field.
  public: ignition::math::Vector3d magneticField =
           ignition::math::Vector3d(5.5645e-6, 22.8758e-6, -42.3884e-6);
};

/////////////////////////////////////////////////
World::World()
  : dataPtr(new WorldPrivate)
{
}

/////////////////////////////////////////////////
World::~World()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
World::World(World &&_world)
{
  this->dataPtr = _world.dataPtr;
  _world.dataPtr = nullptr;
}

/////////////////////////////////////////////////
Errors World::Load(sdf::ElementPtr _sdf)
{
  Errors errors;

  // Check that the provided SDF element is a <world>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "world")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a World, but the provided SDF element is not a "
        "<world>."});
    return errors;
  }

  // Read the world's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A world name is required, but the name is not set."});
  }

  // Read the audio element
  if (_sdf->HasElement("audio"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("audio");
    this->dataPtr->audioDevice = elem->Get<std::string>("device",
        this->dataPtr->audioDevice).first;
  }

  // Read the wind element
  if (_sdf->HasElement("wind"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("wind");
    this->dataPtr->windLinearVelocity =
      elem->Get<ignition::math::Vector3d>("linear_velocity",
          this->dataPtr->windLinearVelocity).first;
  }

  // Read gravity.
  this->dataPtr->gravity = _sdf->Get<ignition::math::Vector3d>("gravity",
        this->dataPtr->gravity).first;

  // Read the magnetic field.
  this->dataPtr->magneticField =
    _sdf->Get<ignition::math::Vector3d>("magnetic_field",
        this->dataPtr->magneticField).first;

  return errors;
}

/////////////////////////////////////////////////
std::string World::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void World::SetName(const std::string &_name) const
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
std::string World::AudioDevice() const
{
  return this->dataPtr->audioDevice;
}

/////////////////////////////////////////////////
void World::SetAudioDevice(const std::string &_device)
{
  this->dataPtr->audioDevice = _device;
}

/////////////////////////////////////////////////
ignition::math::Vector3d World::WindLinearVelocity() const
{
  return this->dataPtr->windLinearVelocity;
}

/////////////////////////////////////////////////
void World::SetWindLinearVelocity(const ignition::math::Vector3d &_wind)
{
  this->dataPtr->windLinearVelocity = _wind;
}

/////////////////////////////////////////////////
ignition::math::Vector3d World::Gravity() const
{
  return this->dataPtr->gravity;
}

/////////////////////////////////////////////////
void World::SetGravity(const ignition::math::Vector3d &_gravity)
{
  this->dataPtr->gravity = _gravity;
}

/////////////////////////////////////////////////
ignition::math::Vector3d World::MagneticField() const
{
  return this->dataPtr->magneticField;
}

/////////////////////////////////////////////////
void World::SetMagneticField(const ignition::math::Vector3d &_mag)
{
  this->dataPtr->magneticField = _mag;
}
