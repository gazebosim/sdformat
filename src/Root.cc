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
#include <string>
#include <vector>
#include <utility>

#include "sdf/Actor.hh"
#include "sdf/Light.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"
#include "sdf/parser.hh"
#include "sdf/sdf_config.h"
#include "Utils.hh"

using namespace sdf;

/// \brief Private data for Root
class sdf::RootPrivate
{
  /// \brief Version string
  public: std::string version = "";

  /// \brief The worlds specified under the root SDF element
  public: std::vector<World> worlds;

  /// \brief The models specified under the root SDF element
  public: std::vector<Model> models;

  /// \brief The lights specified under the root SDF element
  public: std::vector<Light> lights;

  /// \brief The actors specified under the root SDF element
  public: std::vector<Actor> actors;

  /// \brief The SDF element pointer generated during load.
  public: ElementPtr sdf;
};

/////////////////////////////////////////////////
Root::Root()
  : dataPtr(new RootPrivate)
{
}

/////////////////////////////////////////////////
Root::~Root()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
Errors Root::Load(const std::string &_filename)
{
  Errors errors;

  // Read an SDF file, and store the result in sdfParsed.
  SDFPtr sdfParsed = readFile(_filename, errors);

  // Return if we were not able to read the file.
  if (!sdfParsed)
  {
    errors.push_back(
        {ErrorCode::FILE_READ, "Unable to read file:" + _filename});
    return errors;
  }

  Errors loadErrors = this->Load(sdfParsed);
  errors.insert(errors.end(), loadErrors.begin(), loadErrors.end());

  return errors;
}

/////////////////////////////////////////////////
Errors Root::LoadSdfString(const std::string &_sdf)
{
  Errors errors;
  SDFPtr sdfParsed(new SDF());
  init(sdfParsed);

  // Read an SDF string, and store the result in sdfParsed.
  if (!readString(_sdf, sdfParsed, errors))
  {
    errors.push_back(
        {ErrorCode::STRING_READ, "Unable to SDF string: " + _sdf});
    return errors;
  }

  Errors loadErrors = this->Load(sdfParsed);
  errors.insert(errors.end(), loadErrors.begin(), loadErrors.end());

  return errors;
}

/////////////////////////////////////////////////
Errors Root::Load(SDFPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf->Root();

  // Get the SDF version.
  std::pair<std::string, bool> versionPair =
    this->dataPtr->sdf->Get<std::string>("version", SDF_VERSION);

  // Check that the version exists. Exit if the version is missing.
  // readFile will fail if the version is missing, so this
  // check should never be triggered.
  if (!versionPair.second)
  {
    errors.push_back(
        {ErrorCode::ATTRIBUTE_MISSING, "SDF does not have a version."});
    return errors;
  }

  // Check that the version is 1.7, since this is assumed by the DOM API
  if ("1.7" != versionPair.first)
  {
    errors.push_back(
        {ErrorCode::ATTRIBUTE_INVALID,
        "SDF version attribute[" + versionPair.first + "] should match "
        "the latest version[1.7] when loading DOM objects."});
    return errors;
  }

  this->dataPtr->version = versionPair.first;

  // Read all the worlds
  if (this->dataPtr->sdf->HasElement("world"))
  {
    ElementPtr elem = this->dataPtr->sdf->GetElement("world");
    while (elem)
    {
      World world;

      Errors worldErrors = world.Load(elem);
      // Attempt to load the world
      if (worldErrors.empty())
      {
        // Check that the world's name does not exist.
        if (this->WorldNameExists(world.Name()))
        {
          errors.push_back({ErrorCode::DUPLICATE_NAME,
                "World with name[" + world.Name() + "] already exists."
                " Each world must have a unique name. Skipping this world."});
        }
        else
        {
          this->dataPtr->worlds.push_back(std::move(world));
        }
      }
      else
      {
        std::move(worldErrors.begin(), worldErrors.end(),
                  std::back_inserter(errors));
        errors.push_back({ErrorCode::ELEMENT_INVALID,
                          "Failed to load a world."});
      }
      elem = elem->GetNextElement("world");
    }
  }

  // Load all the models.
  Errors modelLoadErrors = loadUniqueRepeated<Model>(this->dataPtr->sdf,
      "model", this->dataPtr->models);
  errors.insert(errors.end(), modelLoadErrors.begin(), modelLoadErrors.end());

  // Load all the lights.
  Errors lightLoadErrors = loadUniqueRepeated<Light>(this->dataPtr->sdf,
      "light", this->dataPtr->lights);
  errors.insert(errors.end(), lightLoadErrors.begin(), lightLoadErrors.end());

  // Load all the actors.
  Errors actorLoadErrors = loadUniqueRepeated<Actor>(this->dataPtr->sdf,
      "actor", this->dataPtr->actors);
  errors.insert(errors.end(), actorLoadErrors.begin(), actorLoadErrors.end());

  return errors;
}

/////////////////////////////////////////////////
std::string Root::Version() const
{
  return this->dataPtr->version;
}

/////////////////////////////////////////////////
void Root::SetVersion(const std::string &_version)
{
  this->dataPtr->version = _version;
}

/////////////////////////////////////////////////
uint64_t Root::WorldCount() const
{
  return this->dataPtr->worlds.size();
}

/////////////////////////////////////////////////
const World *Root::WorldByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->worlds.size())
    return &this->dataPtr->worlds[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Root::WorldNameExists(const std::string &_name) const
{
  for (auto const &w : this->dataPtr->worlds)
  {
    if (w.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
uint64_t Root::ModelCount() const
{
  return this->dataPtr->models.size();
}

/////////////////////////////////////////////////
const Model *Root::ModelByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->models.size())
    return &this->dataPtr->models[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Root::ModelNameExists(const std::string &_name) const
{
  for (auto const &m : this->dataPtr->models)
  {
    if (m.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
uint64_t Root::LightCount() const
{
  return this->dataPtr->lights.size();
}

/////////////////////////////////////////////////
const Light *Root::LightByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->lights.size())
    return &this->dataPtr->lights[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Root::LightNameExists(const std::string &_name) const
{
  for (auto const &m : this->dataPtr->lights)
  {
    if (m.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
uint64_t Root::ActorCount() const
{
  return this->dataPtr->actors.size();
}

/////////////////////////////////////////////////
const Actor *Root::ActorByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->actors.size())
    return &this->dataPtr->actors[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Root::ActorNameExists(const std::string &_name) const
{
  for (auto const &m : this->dataPtr->actors)
  {
    if (m.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
ElementPtr Root::Element() const
{
  return this->dataPtr->sdf;
}
