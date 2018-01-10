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
#include <map>

#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "sdf/sdf_config.h"
#include "sdf/parser.hh"

using namespace sdf;

/// \brief Private data for sdf::Root
class sdf::RootPrivate
{
  /// \brief Version string
  public: std::string version = "";

  /// \brief The worlds specified under the root SDF element
  public: std::vector<World> worlds;
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
  sdf::SDFPtr sdfParsed = sdf::readFile(_filename, errors);

  // Return if we were not able to read the file.
  if (!sdfParsed)
  {
    errors.push_back(
        {ErrorCode::FILE_READ, "Unable to read file:" + _filename});
    return errors;
  }

  sdf::ElementPtr sdf = sdfParsed->Root();

  // Get the SDF version.
  std::pair<std::string, bool> versionPair =
    sdf->Get<std::string>("version", SDF_VERSION);

  // Check that the version exists. Exit if the version is missing.
  // sdf::readFile will fail if the version is missing, so this
  // check should never be triggered.
  if (!versionPair.second)
  {
    errors.push_back(
        {ErrorCode::ATTRIBUTE_MISSING, "SDF does not have a version."});
    return errors;
  }

  this->dataPtr->version = versionPair.first;

  // Read all the worlds
  if (sdf->HasElement("world"))
  {
    sdf::ElementPtr elem = sdf->GetElement("world");
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
