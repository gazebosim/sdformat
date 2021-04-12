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
#include <unordered_set>
#include <vector>
#include <optional>
#include <ignition/math/Vector3.hh>

#include "sdf/Actor.hh"
#include "sdf/Frame.hh"
#include "sdf/InterfaceElements.hh"
#include "sdf/InterfaceModel.hh"
#include "sdf/InterfaceModelPoseGraph.hh"
#include "sdf/Light.hh"
#include "sdf/Model.hh"
#include "sdf/ParserConfig.hh"
#include "sdf/Physics.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"
#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::World::Implementation
{
  /// \brief Optional atmosphere model.
  public: std::optional<sdf::Atmosphere> atmosphere;

  /// \brief Audio device name
  public: std::string audioDevice = "default";

  /// \brief Gravity vector.
  public: ignition::math::Vector3d gravity =
           ignition::math::Vector3d(0, 0, -9.80665);

  /// \brief Optional Gui parameters.
  public: std::optional<sdf::Gui> gui;

  /// \brief Optional Scene parameters.
  public: std::optional<sdf::Scene> scene;

  /// \brief The frames specified in this world.
  public: std::vector<Frame> frames;

  /// \brief The lights specified in this world.
  public: std::vector<Light> lights;

  /// \brief The actors specified in this world.
  public: std::vector<Actor> actors;

  /// \brief Magnetic field.
  public: ignition::math::Vector3d magneticField =
           ignition::math::Vector3d(5.5645e-6, 22.8758e-6, -42.3884e-6);

  /// \brief The models specified in this world.
  public: std::vector<Model> models;

  /// \brief The interface models specified in this world.
  public: std::vector<std::pair<sdf::NestedInclude, sdf::InterfaceModelPtr>>
      interfaceModels;

  /// \brief Name of the world.
  public: std::string name = "";

  /// \brief The physics profiles specified in this world.
  public: std::vector<Physics> physics;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief Linear velocity of wind.
  public: ignition::math::Vector3d windLinearVelocity =
           ignition::math::Vector3d::Zero;

  /// \brief Scoped Frame Attached-To graph that points to a graph owned
  /// by this world.
  public: sdf::ScopedGraph<sdf::FrameAttachedToGraph> frameAttachedToGraph;

  /// \brief Scoped Pose Relative-To graph that points to a graph owned by this
  /// world.
  public: sdf::ScopedGraph<sdf::PoseRelativeToGraph> poseRelativeToGraph;
};


/////////////////////////////////////////////////
World::World()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
  this->dataPtr->physics.emplace_back(Physics());
}

/////////////////////////////////////////////////
Errors World::Load(sdf::ElementPtr _sdf)
{
  return this->Load(_sdf, ParserConfig::GlobalConfig());
}

/////////////////////////////////////////////////
Errors World::Load(sdf::ElementPtr _sdf, const ParserConfig &_config)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

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

  // Check that the world's name is valid
  if (isReservedName(this->dataPtr->name))
  {
    errors.push_back({ErrorCode::RESERVED_NAME,
                     "The supplied world name [" + this->dataPtr->name +
                     "] is reserved."});
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

  // Read the atmosphere element
  if (_sdf->HasElement("atmosphere"))
  {
    this->dataPtr->atmosphere.emplace();
    Errors atmosphereLoadErrors =
      this->dataPtr->atmosphere->Load(_sdf->GetElement("atmosphere"));
    errors.insert(errors.end(), atmosphereLoadErrors.begin(),
        atmosphereLoadErrors.end());
  }

  // Read gravity.
  this->dataPtr->gravity = _sdf->Get<ignition::math::Vector3d>("gravity",
        this->dataPtr->gravity).first;

  // Read the magnetic field.
  this->dataPtr->magneticField =
    _sdf->Get<ignition::math::Vector3d>("magnetic_field",
        this->dataPtr->magneticField).first;

  if (!_sdf->HasUniqueChildNames())
  {
    sdfwarn << "Non-unique names detected in XML children of world with name["
            << this->Name() << "].\n";
  }

  // Set of implicit and explicit frame names in this model for tracking
  // name collisions
  std::unordered_set<std::string> frameNames;

  // Load all the models.
  Errors modelLoadErrors =
      loadUniqueRepeated<Model>(_sdf, "model", this->dataPtr->models, _config);
  errors.insert(errors.end(), modelLoadErrors.begin(), modelLoadErrors.end());

  // Models are loaded first, and loadUniqueRepeated ensures there are no
  // duplicate names, so these names can be added to frameNames without
  // checking uniqueness.
  for (const auto &model : this->dataPtr->models)
  {
    frameNames.insert(model.Name());
  }

  // Load included models via the interface API
  Errors interfaceModelLoadErrors = loadIncludedInterfaceModels(
      _sdf, _config, this->dataPtr->interfaceModels);
  errors.insert(errors.end(), interfaceModelLoadErrors.begin(),
      interfaceModelLoadErrors.end());

  for (const auto &ifaceModelPair : this->dataPtr->interfaceModels)
  {
    frameNames.insert(ifaceModelPair.second->Name()).second;
  }

  // Load all the physics.
  if (_sdf->HasElement("physics"))
  {
    this->dataPtr->physics.clear();
    Errors physicsLoadErrors = loadUniqueRepeated<Physics>(_sdf, "physics",
        this->dataPtr->physics);
    errors.insert(errors.end(), physicsLoadErrors.begin(),
        physicsLoadErrors.end());
  }

  // Load all the actors.
  Errors actorLoadErrors = loadUniqueRepeated<Actor>(_sdf, "actor",
      this->dataPtr->actors);
  errors.insert(errors.end(), actorLoadErrors.begin(), actorLoadErrors.end());

  // Load all the lights.
  Errors lightLoadErrors = loadUniqueRepeated<Light>(_sdf, "light",
      this->dataPtr->lights);
  errors.insert(errors.end(), lightLoadErrors.begin(), lightLoadErrors.end());

  // Load all the frames.
  Errors frameLoadErrors = loadUniqueRepeated<Frame>(_sdf, "frame",
      this->dataPtr->frames);
  errors.insert(errors.end(), frameLoadErrors.begin(), frameLoadErrors.end());

  // Check frames for name collisions and modify and warn if so.
  for (auto &frame : this->dataPtr->frames)
  {
    std::string frameName = frame.Name();
    if (frameNames.count(frameName) > 0)
    {
      frameName += "_frame";
      int i = 0;
      while (frameNames.count(frameName) > 0)
      {
        frameName = frame.Name() + "_frame" + std::to_string(i++);
      }
      sdfwarn << "Frame with name [" << frame.Name() << "] "
              << "in world with name [" << this->Name() << "] "
              << "has a name collision, changing frame name to ["
              << frameName << "].\n";
      frame.SetName(frameName);
    }
    frameNames.insert(frameName);
  }

  // Load the Gui
  if (_sdf->HasElement("gui"))
  {
    this->dataPtr->gui.emplace();
    Errors guiLoadErrors = this->dataPtr->gui->Load(_sdf->GetElement("gui"));
    errors.insert(errors.end(), guiLoadErrors.begin(), guiLoadErrors.end());
  }

  // Load the Scene
  if (_sdf->HasElement("scene"))
  {
    this->dataPtr->scene.emplace();
    Errors sceneLoadErrors =
        this->dataPtr->scene->Load(_sdf->GetElement("scene"));
    errors.insert(errors.end(), sceneLoadErrors.begin(), sceneLoadErrors.end());
  }

  return errors;
}

/////////////////////////////////////////////////
std::string World::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void World::SetName(const std::string &_name)
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

/////////////////////////////////////////////////
uint64_t World::ModelCount() const
{
  return this->dataPtr->models.size();
}

/////////////////////////////////////////////////
const Model *World::ModelByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->models.size())
    return &this->dataPtr->models[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool World::ModelNameExists(const std::string &_name) const
{
  return nullptr != this->ModelByName(_name);
}

/////////////////////////////////////////////////
const Model *World::ModelByName(const std::string &_name) const
{
  auto index = _name.find("::");
  const std::string nextModelName = _name.substr(0, index);
  const Model *nextModel = nullptr;

  for (auto const &m : this->dataPtr->models)
  {
    if (m.Name() == nextModelName)
    {
      nextModel = &m;
      break;
    }
  }

  if (nullptr != nextModel && index != std::string::npos)
  {
    return nextModel->ModelByName(_name.substr(index + 2));
  }
  return nextModel;
}

/////////////////////////////////////////////////
const sdf::Atmosphere *World::Atmosphere() const
{
  return optionalToPointer(this->dataPtr->atmosphere);
}

/////////////////////////////////////////////////
void World::SetAtmosphere(const sdf::Atmosphere &_atmosphere)
{
  this->dataPtr->atmosphere = _atmosphere;
}

/////////////////////////////////////////////////
const sdf::Gui *World::Gui() const
{
  return optionalToPointer(this->dataPtr->gui);
}

/////////////////////////////////////////////////
void World::SetGui(const sdf::Gui &_gui)
{
  this->dataPtr->gui = _gui;
}

/////////////////////////////////////////////////
const sdf::Scene *World::Scene() const
{
  return optionalToPointer(this->dataPtr->scene);
}

/////////////////////////////////////////////////
void World::SetScene(const sdf::Scene &_scene)
{
  this->dataPtr->scene = _scene;
}

/////////////////////////////////////////////////
sdf::ElementPtr World::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
uint64_t World::FrameCount() const
{
  return this->dataPtr->frames.size();
}

/////////////////////////////////////////////////
const Frame *World::FrameByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->frames.size())
    return &this->dataPtr->frames[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool World::FrameNameExists(const std::string &_name) const
{
  return nullptr != this->FrameByName(_name);
}

/////////////////////////////////////////////////
const Frame *World::FrameByName(const std::string &_name) const
{
  auto index = _name.rfind("::");
  if (index != std::string::npos)
  {
    const Model *model = this->ModelByName(_name.substr(0, index));
    if (nullptr != model)
    {
      return model->FrameByName(_name.substr(index + 2));
    }

    // The nested model name preceding the last "::" could not be found.
    // For now, try to find a link that matches _name exactly.
    // When "::" are reserved and not allowed in names, then uncomment
    // the following line to return a nullptr.
    // return nullptr;
  }

  for (auto const &f : this->dataPtr->frames)
  {
    if (f.Name() == _name)
    {
      return &f;
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
uint64_t World::LightCount() const
{
  return this->dataPtr->lights.size();
}

/////////////////////////////////////////////////
const Light *World::LightByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->lights.size())
    return &this->dataPtr->lights[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool World::LightNameExists(const std::string &_name) const
{
  for (auto const &l : this->dataPtr->lights)
  {
    if (l.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
uint64_t World::ActorCount() const
{
  return this->dataPtr->actors.size();
}

/////////////////////////////////////////////////
const Actor *World::ActorByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->actors.size())
    return &this->dataPtr->actors[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool World::ActorNameExists(const std::string &_name) const
{
  for (auto const &a : this->dataPtr->actors)
  {
    if (a.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

//////////////////////////////////////////////////
uint64_t World::PhysicsCount() const
{
  return this->dataPtr->physics.size();
}

//////////////////////////////////////////////////
const Physics *World::PhysicsByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->physics.size())
    return &this->dataPtr->physics[_index];
  return nullptr;
}

//////////////////////////////////////////////////
const Physics *World::PhysicsDefault() const
{
  if (!this->dataPtr->physics.empty())
  {
    for (const Physics &physics : this->dataPtr->physics)
    {
      if (physics.IsDefault())
        return &physics;
    }

    return &this->dataPtr->physics.at(0);
  }

  return nullptr;
}

//////////////////////////////////////////////////
bool World::PhysicsNameExists(const std::string &_name) const
{
  for (const Physics &physics : this->dataPtr->physics)
  {
    if (physics.Name() == _name)
    {
      return true;
    }
  }

  return false;
}

/////////////////////////////////////////////////
uint64_t World::InterfaceModelCount() const
{
  return this->dataPtr->interfaceModels.size();
}

/////////////////////////////////////////////////
InterfaceModelConstPtr World::InterfaceModelByIndex(
    const uint64_t _index) const
{
  if (_index < this->dataPtr->interfaceModels.size())
    return this->dataPtr->interfaceModels[_index].second;
  return nullptr;
}

/////////////////////////////////////////////////
const NestedInclude *World::InterfaceModelNestedIncludeByIndex(
    const uint64_t _index) const
{
  if (_index < this->dataPtr->interfaceModels.size())
    return &this->dataPtr->interfaceModels[_index].first;
  return nullptr;
}

/////////////////////////////////////////////////
void World::SetPoseRelativeToGraph(sdf::ScopedGraph<PoseRelativeToGraph> _graph)
{
  this->dataPtr->poseRelativeToGraph = _graph;

  for (auto &model : this->dataPtr->models)
  {
    model.SetPoseRelativeToGraph(this->dataPtr->poseRelativeToGraph);
  }
  for (auto &ifaceModelPair : this->dataPtr->interfaceModels)
  {
    ifaceModelPair.second->InvokeRespostureFunction(
        this->dataPtr->poseRelativeToGraph);
  }
  for (auto &frame : this->dataPtr->frames)
  {
    frame.SetPoseRelativeToGraph(this->dataPtr->poseRelativeToGraph);
  }
  for (auto &light : this->dataPtr->lights)
  {
    light.SetXmlParentName("world");
    light.SetPoseRelativeToGraph(this->dataPtr->poseRelativeToGraph);
  }
}

/////////////////////////////////////////////////
void World::SetFrameAttachedToGraph(
    sdf::ScopedGraph<FrameAttachedToGraph> _graph)
{
  this->dataPtr->frameAttachedToGraph = _graph;

  for (auto &frame : this->dataPtr->frames)
  {
    frame.SetFrameAttachedToGraph(this->dataPtr->frameAttachedToGraph);
  }
  for (auto &model : this->dataPtr->models)
  {
    model.SetFrameAttachedToGraph(this->dataPtr->frameAttachedToGraph);
  }
}
