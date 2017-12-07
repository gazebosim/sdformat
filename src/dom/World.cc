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
#include "Utils.hh"
#include "sdf/dom/Model.hh"
#include "sdf/dom/World.hh"

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

  /// \brief All the models that belong to the world
  public: std::map<std::string, Model> models;

  /// \brief All the lights that belong to the world
  public: std::map<std::string, Light> lights;
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
bool World::Load(sdf::ElementPtr _sdf)
{
  // Check that the provided SDF element is a <world>
  if (_sdf->GetName() != "world")
  {
    std::cerr << "Attempting to load a World, but the provided "
      << "SDF element is not a <world>\n";

    // This is an error that cannot be recovered, so return false.
    return false;
  }

  bool result = true;

  // Read the world's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    std::cerr << "A world name is required, but is not set.\n";
    result = false;
  }

  // Read all the models
  result = result && loadModels(_sdf, this->dataPtr->models);

  // Read all the lights
  result = result && loadLights(_sdf, this->dataPtr->lights);

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
      elem->Get<ignition::math::Vector3d>("wind",
          this->dataPtr->windLinearVelocity).first;
  }

  // Read gravity.
  this->dataPtr->gravity = _sdf->Get<ignition::math::Vector3d>("gravity",
        this->dataPtr->gravity).first;

  // Read the magnetic field.
  this->dataPtr->magneticField =
    _sdf->Get<ignition::math::Vector3d>("magnetic_field",
        this->dataPtr->magneticField).first;

  return result;
}

/////////////////////////////////////////////////
void World::Print(const std::string &_prefix) const
{
  std::cout << _prefix << "# World: " << this->Name() << "\n"
            << _prefix << "  * Model count: "
            << this->dataPtr->models.size() << "\n"
            << _prefix << "  * Light count: "
            << this->dataPtr->models.size() << "\n";

  for (auto const &model: this->dataPtr->models)
  {
    model.second.Print(_prefix + "  ");
  }

  for (auto const &light: this->dataPtr->lights)
  {
    light.second.Print(_prefix + "  ");
  }
}

/////////////////////////////////////////////////
std::string World::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void World::Name(const std::string &_name) const
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
void World::MagneticField(const ignition::math::Vector3d &_mag)
{
  this->dataPtr->magneticField = _mag;
}

/////////////////////////////////////////////////
size_t World::ModelCount() const
{
  return this->dataPtr->models.size();
}

/////////////////////////////////////////////////
size_t World::LightCount() const
{
  return this->dataPtr->lights.size();
}
