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
#include <variant>
#include <vector>
#include <utility>

#include "sdf/Actor.hh"
#include "sdf/Error.hh"
#include "sdf/Light.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"
#include "sdf/parser.hh"
#include "sdf/sdf_config.h"
#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "Utils.hh"

using namespace sdf;

/// \brief Private data for sdf::Root
class sdf::Root::Implementation
{
  /// \brief Version string
  public: std::string version = "";

  /// \brief The worlds specified under the root SDF element
  public: std::vector<World> worlds;

  /// \brief A model, light or actor under the root SDF element
  public: std::variant<std::monostate, sdf::Model, sdf::Light, sdf::Actor>
              modelLightOrActor;

  /// \brief Frame Attached-To Graphs constructed when loading Worlds.
  public: std::vector<sdf::ScopedGraph<FrameAttachedToGraph>>
              worldFrameAttachedToGraphs;

  /// \brief Frame Attached-To Graphs constructed when loading a Model.
  public: sdf::ScopedGraph<FrameAttachedToGraph> modelFrameAttachedToGraph;

  /// \brief Pose Relative-To Graphs constructed when loading Worlds.
  public: std::vector<sdf::ScopedGraph<PoseRelativeToGraph>>
              worldPoseRelativeToGraphs;

  /// \brief Pose Relative-To Graph constructed when loading a Model.
  public: sdf::ScopedGraph<PoseRelativeToGraph> modelPoseRelativeToGraph;

  /// \brief The SDF element pointer generated during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
template <typename T>
sdf::ScopedGraph<FrameAttachedToGraph> createFrameAttachedToGraph(
    const T &_domObj, sdf::Errors &_errors)
{
  auto frameGraph = sdf::ScopedGraph<FrameAttachedToGraph>(
      std::make_shared<FrameAttachedToGraph>());

  sdf::Errors buildErrors =
      sdf::buildFrameAttachedToGraph(frameGraph, &_domObj);
  _errors.insert(_errors.end(), buildErrors.begin(), buildErrors.end());

  sdf::Errors validateErrors = sdf::validateFrameAttachedToGraph(frameGraph);
  _errors.insert(_errors.end(), validateErrors.begin(), validateErrors.end());

  return frameGraph;
}

/////////////////////////////////////////////////
template <typename T>
sdf::ScopedGraph<FrameAttachedToGraph> addFrameAttachedToGraph(
    std::vector<sdf::ScopedGraph<sdf::FrameAttachedToGraph>> &_graphList,
    const T &_domObj, sdf::Errors &_errors)
{
  auto frameGraph = createFrameAttachedToGraph(_domObj, _errors);
  _graphList.push_back(frameGraph);

  return frameGraph;
}

/////////////////////////////////////////////////
template <typename T>
ScopedGraph<PoseRelativeToGraph> createPoseRelativeToGraph(
    const T &_domObj, Errors &_errors)
{
  auto poseGraph = ScopedGraph<PoseRelativeToGraph>(
      std::make_shared<sdf::PoseRelativeToGraph>());

  Errors buildErrors = buildPoseRelativeToGraph(poseGraph, &_domObj);
  _errors.insert(_errors.end(), buildErrors.begin(), buildErrors.end());

  Errors validateErrors = validatePoseRelativeToGraph(poseGraph);
  _errors.insert(_errors.end(), validateErrors.begin(), validateErrors.end());

  return poseGraph;
}

/////////////////////////////////////////////////
template <typename T>
ScopedGraph<PoseRelativeToGraph> addPoseRelativeToGraph(
    std::vector<sdf::ScopedGraph<sdf::PoseRelativeToGraph>> &_graphList,
    const T &_domObj, Errors &_errors)
{
  auto poseGraph = createPoseRelativeToGraph(_domObj, _errors);
  _graphList.push_back(poseGraph);

  return poseGraph;
}

/////////////////////////////////////////////////
Root::Root()
  : dataPtr(ignition::utils::MakeUniqueImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Root::Load(const std::string &_filename)
{
  return this->Load(_filename, ParserConfig::GlobalConfig());
}

/////////////////////////////////////////////////
Errors Root::Load(const std::string &_filename, const ParserConfig &_config)
{
  Errors errors;

  // Read an SDF file, and store the result in sdfParsed.
  SDFPtr sdfParsed = readFile(_filename, _config, errors);

  // Return if we were not able to read the file.
  if (!sdfParsed)
  {
    errors.push_back(
        {ErrorCode::FILE_READ, "Unable to read file:" + _filename});
    return errors;
  }

  Errors loadErrors = this->Load(sdfParsed, _config);
  errors.insert(errors.end(), loadErrors.begin(), loadErrors.end());

  return errors;
}

/////////////////////////////////////////////////
Errors Root::LoadSdfString(const std::string &_sdf)
{
  return this->LoadSdfString(_sdf, ParserConfig::GlobalConfig());
}

/////////////////////////////////////////////////
Errors Root::LoadSdfString(const std::string &_sdf, const ParserConfig &_config)
{
  Errors errors;
  SDFPtr sdfParsed(new SDF());
  init(sdfParsed);

  // Read an SDF string, and store the result in sdfParsed.
  if (!readString(_sdf, _config, sdfParsed, errors))
  {
    errors.push_back(
        {ErrorCode::STRING_READ, "Unable to read SDF string: " + _sdf});
    return errors;
  }

  Errors loadErrors = this->Load(sdfParsed, _config);
  errors.insert(errors.end(), loadErrors.begin(), loadErrors.end());

  return errors;
}

/////////////////////////////////////////////////
Errors Root::Load(SDFPtr _sdf)
{
  return this->Load(_sdf, ParserConfig::GlobalConfig());
}

/////////////////////////////////////////////////
Errors Root::Load(SDFPtr _sdf, const ParserConfig &_config)
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

  // Check that the version is the latest, since this is assumed by the DOM API
  if (SDF_PROTOCOL_VERSION != versionPair.first)
  {
    errors.push_back(
        {ErrorCode::ATTRIBUTE_INVALID,
        "SDF version attribute[" + versionPair.first + "] should match "
        "the latest version[" + SDF_PROTOCOL_VERSION + "] when loading DOM "
        "objects."});
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

      Errors worldErrors = world.Load(elem, _config);

      // Build the graphs.
      auto frameAttachedToGraph = addFrameAttachedToGraph(
          this->dataPtr->worldFrameAttachedToGraphs, world, worldErrors);
      world.SetFrameAttachedToGraph(frameAttachedToGraph);

      auto poseRelativeToGraph = addPoseRelativeToGraph(
          this->dataPtr->worldPoseRelativeToGraphs, world, worldErrors);
      world.SetPoseRelativeToGraph(poseRelativeToGraph);

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
      }
      else
      {
        std::move(worldErrors.begin(), worldErrors.end(),
                  std::back_inserter(errors));
        errors.push_back({ErrorCode::ELEMENT_INVALID,
                          "Failed to load a world."});
      }

      this->dataPtr->worlds.push_back(std::move(world));
      elem = elem->GetNextElement("world");
    }
  }

  // Load all the models.
  std::vector<sdf::Model> models;
  Errors modelLoadErrors = loadUniqueRepeated<sdf::Model>(
      this->dataPtr->sdf, "model", models, _config);
  errors.insert(errors.end(), modelLoadErrors.begin(), modelLoadErrors.end());
  if (!models.empty())
  {
    if (models.size() > 1)
    {
      errors.emplace_back(
          ErrorCode::ELEMENT_INCORRECT_TYPE,
          "Root object can only contain one model. Using the first one found");
    }
    this->dataPtr->modelLightOrActor = std::move(models.front());
    sdf::Model &model = std::get<sdf::Model>(this->dataPtr->modelLightOrActor);
    // Build the graphs.
    this->dataPtr->modelFrameAttachedToGraph =
        createFrameAttachedToGraph(model, errors);

    model.SetFrameAttachedToGraph(this->dataPtr->modelFrameAttachedToGraph);

    this->dataPtr->modelPoseRelativeToGraph =
        createPoseRelativeToGraph(model, errors);
    model.SetPoseRelativeToGraph(this->dataPtr->modelPoseRelativeToGraph);
  }

  // Load all the lights.
  std::vector<sdf::Light> lights;
  Errors lightLoadErrors =
      loadUniqueRepeated<sdf::Light>(this->dataPtr->sdf, "light", lights);
  errors.insert(errors.end(), lightLoadErrors.begin(), lightLoadErrors.end());
  if (!lights.empty())
  {
    if (lights.size() > 1)
    {
      errors.emplace_back(
          ErrorCode::ELEMENT_INCORRECT_TYPE,
          "Root object can only contain one light. Using the first one found.");
    }
    if (std::holds_alternative<std::monostate>(
            this->dataPtr->modelLightOrActor))
    {
      this->dataPtr->modelLightOrActor = std::move(lights.front());
    }
    else
    {
      errors.emplace_back(
          ErrorCode::ELEMENT_INCORRECT_TYPE,
          "Root object can only contain one of model, light or actor, but "
          "found more than one type of element. Skipping this light.");
    }
  }

  // Load all the actors.
  std::vector<sdf::Actor> actors;
  Errors actorLoadErrors =
      loadUniqueRepeated<sdf::Actor>(this->dataPtr->sdf, "actor", actors);
  errors.insert(errors.end(), actorLoadErrors.begin(), actorLoadErrors.end());
  if (!actors.empty())
  {
    if (actors.size() > 1)
    {
      errors.emplace_back(
          ErrorCode::ELEMENT_INCORRECT_TYPE,
          "Root object can only contain one actor. Using the first one found.");
    }
    if (std::holds_alternative<std::monostate>(
            this->dataPtr->modelLightOrActor))
    {
      this->dataPtr->modelLightOrActor = std::move(actors.front());
    }
    else
    {
      errors.emplace_back(
          ErrorCode::ELEMENT_INCORRECT_TYPE,
          "Root object can only contain one of model, light or actor, but "
          "found more than one type of element. Skipping this actor.");
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

/////////////////////////////////////////////////
const Model *Root::Model() const
{
  return std::get_if<sdf::Model>(&this->dataPtr->modelLightOrActor);
}

/////////////////////////////////////////////////
const Light *Root::Light() const
{
  return std::get_if<sdf::Light>(&this->dataPtr->modelLightOrActor);
}

/////////////////////////////////////////////////
const Actor *Root::Actor() const
{
  return std::get_if<sdf::Actor>(&this->dataPtr->modelLightOrActor);
}

/////////////////////////////////////////////////
sdf::ElementPtr Root::Element() const
{
  return this->dataPtr->sdf;
}
