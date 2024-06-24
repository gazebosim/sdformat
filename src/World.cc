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
#include <gz/math/Vector3.hh>

#include "sdf/Actor.hh"
#include "sdf/Frame.hh"
#include "sdf/InterfaceElements.hh"
#include "sdf/InterfaceModel.hh"
#include "sdf/InterfaceModelPoseGraph.hh"
#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"
#include "sdf/Light.hh"
#include "sdf/Model.hh"
#include "sdf/ParserConfig.hh"
#include "sdf/Physics.hh"
#include "sdf/Plugin.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"
#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "Utils.hh"
#include "sdf/parser.hh"

using namespace sdf;

class sdf::World::Implementation
{
  /// \brief Populate sphericalCoordinates
  /// \param[in] _elem `<spherical_coordinates>` element
  /// \return Errors, if any.
  public: Errors LoadSphericalCoordinates(sdf::ElementPtr _elem);

  /// \brief Required atmosphere model.
  public: sdf::Atmosphere atmosphere;

  /// \brief Audio device name
  public: std::string audioDevice = "default";

  /// \brief Gravity vector.
  public: gz::math::Vector3d gravity =
           gz::math::Vector3d(0, 0, -9.80665);

  /// \brief Optional Gui parameters.
  public: std::optional<sdf::Gui> gui;

  /// \brief Required scene parameters.
  public: sdf::Scene scene;

  /// \brief The frames specified in this world.
  public: std::vector<Frame> frames;

  /// \brief The joints specified in this world.
  public: std::vector<Joint> joints;

  /// \brief The lights specified in this world.
  public: std::vector<Light> lights;

  /// \brief The actors specified in this world.
  public: std::vector<Actor> actors;

  /// \brief Magnetic field.
  public: gz::math::Vector3d magneticField =
           gz::math::Vector3d(5.5645e-6, 22.8758e-6, -42.3884e-6);

  /// \brief Spherical coordinates
  public: std::optional<gz::math::SphericalCoordinates>
      sphericalCoordinates;

  /// \brief The models specified in this world.
  public: std::vector<Model> models;

  /// \brief The interface models specified in this world.
  public: std::vector<std::pair<sdf::NestedInclude,
          sdf::InterfaceModelConstPtr>> interfaceModels;

  /// \brief Name of the world.
  public: std::string name = "";

  /// \brief The physics profiles specified in this world.
  public: std::vector<Physics> physics;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief Linear velocity of wind.
  public: gz::math::Vector3d windLinearVelocity =
           gz::math::Vector3d::Zero;

  /// \brief Scoped Frame Attached-To graph that points to a graph owned
  /// by this world.
  public: sdf::ScopedGraph<sdf::FrameAttachedToGraph> frameAttachedToGraph;

  /// \brief Scoped Pose Relative-To graph that points to a graph owned by this
  /// world.
  public: sdf::ScopedGraph<sdf::PoseRelativeToGraph> poseRelativeToGraph;

  /// \brief World plugins.
  public: sdf::Plugins plugins;
};

/////////////////////////////////////////////////
World::World()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
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
      elem->Get<gz::math::Vector3d>("linear_velocity",
          this->dataPtr->windLinearVelocity).first;
  }

  // Read the atmosphere element
  if (_sdf->HasElement("atmosphere"))
  {
    Errors atmosphereLoadErrors =
      this->dataPtr->atmosphere.Load(_sdf->GetElement("atmosphere"));
    errors.insert(errors.end(), atmosphereLoadErrors.begin(),
        atmosphereLoadErrors.end());
  }

  // Read gravity.
  this->dataPtr->gravity = _sdf->Get<gz::math::Vector3d>("gravity",
        this->dataPtr->gravity).first;

  // Read the magnetic field.
  this->dataPtr->magneticField =
    _sdf->Get<gz::math::Vector3d>("magnetic_field",
        this->dataPtr->magneticField).first;

  // Read the spherical coordinates
  if (_sdf->HasElement("spherical_coordinates"))
  {
    Errors sphericalCoordsErrors = this->dataPtr->LoadSphericalCoordinates(
        _sdf->GetElement("spherical_coordinates"));
    errors.insert(errors.end(), sphericalCoordsErrors.begin(),
        sphericalCoordsErrors.end());
  }

  for (const auto &[name, size] :
       _sdf->CountNamedElements("", Element::NameUniquenessExceptions()))
  {
    if (size > 1)
    {
      std::stringstream ss;
      ss << "Non-unique name[" << name << "] detected " << size
         << " times in XML children of world with name[" << this->Name()
         << "].";
      Error err(ErrorCode::WARNING, ss.str());
      enforceConfigurablePolicyCondition(
          _config.WarningsPolicy(), err, errors);
    }
  }

  std::unordered_set<std::string> modelNames;
  std::unordered_set<std::string> jointNames;
  std::unordered_set<std::string> explicitFrameNames;
  auto recordUniqueName = [&errors](std::unordered_set<std::string>& nameList,
                                      const std::string &_elementName,
                                      const std::string &_name)
  {
    if (nameList.count(_name) > 0)
    {
      errors.emplace_back(ErrorCode::DUPLICATE_NAME,
          _elementName + " with name[" + _name + "] already exists.");
      return false;
    }
    nameList.insert(_name);
    return true;
  };

  // Set of implicit and explicit frame names in this world for tracking
  // name collisions. This is used to handle name clashes in old versions of
  // SDFormat where sibling elements were allowed to have duplicate names.
  std::unordered_set<std::string> implicitFrameNames;

  // Load included models via the interface API
  Errors interfaceModelLoadErrors = loadIncludedInterfaceModels(
      _sdf, _config, this->dataPtr->interfaceModels);
  errors.insert(errors.end(), interfaceModelLoadErrors.begin(),
      interfaceModelLoadErrors.end());

  for (const auto &ifaceModelPair : this->dataPtr->interfaceModels)
  {
    implicitFrameNames.insert(ifaceModelPair.second->Name());
  }

  for (auto elem = _sdf->GetFirstElement(); elem; elem = elem->GetNextElement())
  {
    const std::string elementName = elem->GetName();
    if (elementName == "model")
    {
      auto model = loadSingle<Model>(errors, elem, _config);
      if (!recordUniqueName(modelNames, elementName, model.Name()))
      {
        continue;
      }
      implicitFrameNames.insert(model.Name());
      if (model.IsMerged())
      {
        sdf::Frame proxyFrame = model.PrepareForMerge(errors, "world");
        this->AddFrame(proxyFrame);

        for (uint64_t fi = 0; fi < model.FrameCount(); ++fi)
        {
          this->dataPtr->frames.push_back(std::move(*model.FrameByIndex(fi)));
        }

        for (uint64_t ji = 0; ji < model.JointCount(); ++ji)
        {
          this->dataPtr->joints.push_back(std::move(*model.JointByIndex(ji)));
        }

        for (uint64_t mi = 0; mi < model.ModelCount(); ++mi)
        {
          this->dataPtr->models.push_back(std::move(*model.ModelByIndex(mi)));
        }

        for (uint64_t imi = 0; imi < model.InterfaceModelCount(); ++imi)
        {
          InterfaceModelConstPtr ifaceModel = model.InterfaceModelByIndex(imi);
          NestedInclude nestedInclude =
              *model.InterfaceModelNestedIncludeByIndex(imi);
          this->dataPtr->interfaceModels.emplace_back(std::move(nestedInclude),
                                                      ifaceModel);
        }

        // TODO(azeey) Support Merge-included interface models when `World`
        // supports them.
      }
      else
      {
        this->dataPtr->models.push_back(std::move(model));
      }
    }
    else if (elementName == "joint")
    {
      auto joint = loadSingle<Joint>(errors, elem);
      if (!recordUniqueName(jointNames, elementName, joint.Name()))
      {
        continue;
      }
      this->dataPtr->joints.push_back(std::move(joint));
    }
    else if (elementName == "frame")
    {
      auto frame = loadSingle<Frame>(errors, elem);
      if (!recordUniqueName(explicitFrameNames, elementName, frame.Name()))
      {
        continue;
      }
      this->dataPtr->frames.push_back(std::move(frame));
    }
  }

  // Check frames for name collisions and modify and warn if so.
  for (auto &frame : this->dataPtr->frames)
  {
    std::string frameName = frame.Name();
    if (implicitFrameNames.count(frameName) > 0)
    {
      frameName += "_frame";
      int i = 0;
      while (implicitFrameNames.count(frameName) > 0)
      {
        frameName = frame.Name() + "_frame" + std::to_string(i++);
      }
      std::stringstream ss;
      ss << "Frame with name [" << frame.Name() << "] "
          << "in world with name [" << this->Name() << "] "
          << "has a name collision, changing frame name to ["
          << frameName << "].\n";
      Error err(ErrorCode::WARNING, ss.str());
      enforceConfigurablePolicyCondition(
          _config.WarningsPolicy(), err, errors);
      frame.SetName(frameName);
    }
    implicitFrameNames.insert(frameName);
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
    Errors sceneLoadErrors =
        this->dataPtr->scene.Load(_sdf->GetElement("scene"), _config);
    errors.insert(errors.end(), sceneLoadErrors.begin(), sceneLoadErrors.end());
  }

  // Load the world plugins
  Errors pluginErrors = loadRepeated<Plugin>(_sdf, "plugin",
    this->dataPtr->plugins);
  errors.insert(errors.end(), pluginErrors.begin(), pluginErrors.end());

  return errors;
}

/////////////////////////////////////////////////
Errors World::ValidateGraphs() const
{
  Errors errors =
      validateFrameAttachedToGraph(this->dataPtr->frameAttachedToGraph);
  Errors poseErrors =
      validatePoseRelativeToGraph(this->dataPtr->poseRelativeToGraph);
  errors.insert(errors.end(), poseErrors.begin(), poseErrors.end());
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
gz::math::Vector3d World::WindLinearVelocity() const
{
  return this->dataPtr->windLinearVelocity;
}

/////////////////////////////////////////////////
void World::SetWindLinearVelocity(const gz::math::Vector3d &_wind)
{
  this->dataPtr->windLinearVelocity = _wind;
}

/////////////////////////////////////////////////
gz::math::Vector3d World::Gravity() const
{
  return this->dataPtr->gravity;
}

/////////////////////////////////////////////////
void World::SetGravity(const gz::math::Vector3d &_gravity)
{
  this->dataPtr->gravity = _gravity;
}

/////////////////////////////////////////////////
gz::math::Vector3d World::MagneticField() const
{
  return this->dataPtr->magneticField;
}

/////////////////////////////////////////////////
void World::SetMagneticField(const gz::math::Vector3d &_mag)
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
Model *World::ModelByIndex(uint64_t _index)
{
  return const_cast<Model*>(
      static_cast<const World*>(this)->ModelByIndex(_index));
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
Model *World::ModelByName(const std::string &_name)
{
  return const_cast<Model*>(
      static_cast<const World*>(this)->ModelByName(_name));
}

/////////////////////////////////////////////////
const sdf::Atmosphere *World::Atmosphere() const
{
  return &(this->dataPtr->atmosphere);
}

/////////////////////////////////////////////////
void World::SetAtmosphere(const sdf::Atmosphere &_atmosphere)
{
  this->dataPtr->atmosphere = _atmosphere;
}

/////////////////////////////////////////////////
const gz::math::SphericalCoordinates *
    World::SphericalCoordinates() const
{
  return optionalToPointer(this->dataPtr->sphericalCoordinates);
}

/////////////////////////////////////////////////
void World::SetSphericalCoordinates(const gz::math::SphericalCoordinates
    &_sphericalCoordinates)
{
  this->dataPtr->sphericalCoordinates = _sphericalCoordinates;
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
  return &(this->dataPtr->scene);
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
Frame *World::FrameByIndex(uint64_t _index)
{
  return const_cast<Frame*>(
      static_cast<const World*>(this)->FrameByIndex(_index));
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
Frame *World::FrameByName(const std::string &_name)
{
  return const_cast<Frame*>(
      static_cast<const World*>(this)->FrameByName(_name));
}

/////////////////////////////////////////////////
uint64_t World::JointCount() const
{
  return this->dataPtr->joints.size();
}

/////////////////////////////////////////////////
const Joint *World::JointByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->joints.size())
    return &this->dataPtr->joints[_index];
  return nullptr;
}

/////////////////////////////////////////////////
Joint *World::JointByIndex(uint64_t _index)
{
  return const_cast<Joint*>(
      static_cast<const World*>(this)->JointByIndex(_index));
}

/////////////////////////////////////////////////
const Joint *World::JointByName(const std::string &_name) const
{
  auto index = _name.rfind("::");
  if (index != std::string::npos)
  {
    const Model *model = this->ModelByName(_name.substr(0, index));
    if (nullptr != model)
    {
      return model->JointByName(_name.substr(index + 2));
    }

    // The nested model name preceding the last "::" could not be found.
    // Since "::" are reserved and not allowed in names, return a nullptr.
    return nullptr;
  }

  for (auto const &f : this->dataPtr->joints)
  {
    if (f.Name() == _name)
    {
      return &f;
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
Joint *World::JointByName(const std::string &_name)
{
  return const_cast<Joint*>(
      static_cast<const World*>(this)->JointByName(_name));
}

/////////////////////////////////////////////////
bool World::JointNameExists(const std::string &_name) const
{
  return nullptr != this->JointByName(_name);
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
Light *World::LightByIndex(uint64_t _index)
{
  return const_cast<Light*>(
      static_cast<const World*>(this)->LightByIndex(_index));
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
Actor *World::ActorByIndex(uint64_t _index)
{
  return const_cast<Actor*>(
      static_cast<const World*>(this)->ActorByIndex(_index));
}

/////////////////////////////////////////////////
bool World::ActorNameExists(const std::string &_name) const
{
  return nullptr != this->ActorByName(_name);
}

/////////////////////////////////////////////////
const Actor *World::ActorByName(const std::string &_name) const
{
  for (const Actor &a : this->dataPtr->actors)
  {
    if (a.Name() == _name)
    {
      return &a;
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
Actor *World::ActorByName(const std::string &_name)
{
  return const_cast<Actor*>(
      static_cast<const World*>(this)->ActorByName(_name));
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
Physics *World::PhysicsByIndex(uint64_t _index)
{
  return const_cast<Physics*>(
      static_cast<const World*>(this)->PhysicsByIndex(_index));
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
    ifaceModelPair.second->InvokeRepostureFunction(
        this->dataPtr->poseRelativeToGraph, {});
  }
  for (auto &frame : this->dataPtr->frames)
  {
    frame.SetPoseRelativeToGraph(this->dataPtr->poseRelativeToGraph);
  }
  for (auto &joint : this->dataPtr->joints)
  {
    joint.SetPoseRelativeToGraph(this->dataPtr->poseRelativeToGraph);
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
  for (auto &joint : this->dataPtr->joints)
  {
    joint.SetFrameAttachedToGraph(this->dataPtr->frameAttachedToGraph);
  }
  for (auto &model : this->dataPtr->models)
  {
    model.SetFrameAttachedToGraph(this->dataPtr->frameAttachedToGraph);
  }
}

/////////////////////////////////////////////////
Errors World::Implementation::LoadSphericalCoordinates(
    sdf::ElementPtr _elem)
{
  Errors errors;

  if (_elem->GetName() != "spherical_coordinates")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load <spherical_coordinates>, but the provided SDF "
        "element is a <" + _elem->GetName() + ">."});
    return errors;
  }

  // Get elements
  auto surfaceModel =
      gz::math::SphericalCoordinates::SurfaceType::EARTH_WGS84;
  if (!_elem->HasElement("surface_model"))
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Missing required element <surface_model>"});
  }
  else
  {
    auto surfaceModelStr = _elem->Get<std::string>("surface_model");
    if ((surfaceModelStr != "EARTH_WGS84") &&
        (surfaceModelStr != "MOON_SCS") &&
        (surfaceModelStr != "CUSTOM_SURFACE"))
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "The supplied <surface_model> [" + surfaceModelStr +
          "] is not supported."});
    }
    surfaceModel = gz::math::SphericalCoordinates::Convert(
        surfaceModelStr);
  }

  // Read ellipsoidal parameters for custom surfaces.
  double axisEquatorial = 0;
  double axisPolar = 0;

  if (surfaceModel == gz::math::SphericalCoordinates::CUSTOM_SURFACE)
  {
    if (!_elem->HasElement("surface_axis_equatorial"))
    {
      errors.push_back({ErrorCode::ELEMENT_MISSING,
          "Missing required element <surface_axis_equatorial>"});
    }
    else
    {
      axisEquatorial = _elem->Get<double>("surface_axis_equatorial");
    }

    if (!_elem->HasElement("surface_axis_polar"))
    {
      errors.push_back({ErrorCode::ELEMENT_MISSING,
          "Missing required element <surface_axis_polar>"});
    }
    else
    {
      axisPolar = _elem->Get<double>("surface_axis_polar");
    }
  }

  std::string worldFrameOrientation{"ENU"};
  if (_elem->HasElement("world_frame_orientation"))
  {
    worldFrameOrientation = _elem->Get<std::string>(
        "world_frame_orientation");
    if (worldFrameOrientation != "ENU")
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "The supplied <world_frame_orientation> [" + worldFrameOrientation +
          "] is not supported."});
    }
  }

  gz::math::Angle latitude{0.0};
  if (!_elem->HasElement("latitude_deg"))
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Missing required element <latitude_deg>"});
  }
  else
  {
    latitude.SetDegree(_elem->Get<double>("latitude_deg"));
  }

  gz::math::Angle longitude{0.0};
  if (!_elem->HasElement("longitude_deg"))
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Missing required element <longitude_deg>"});
  }
  else
  {
    longitude.SetDegree(_elem->Get<double>("longitude_deg"));
  }

  double elevation{0.0};
  if (!_elem->HasElement("elevation"))
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Missing required element <elevation>"});
  }
  else
  {
    elevation = _elem->Get<double>("elevation");
  }

  gz::math::Angle heading{0.0};
  if (!_elem->HasElement("heading_deg"))
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Missing required element <heading_deg>"});
  }
  else
  {
    heading.SetDegree(_elem->Get<double>("heading_deg"));
  }

  // Create coordinates
  this->sphericalCoordinates.emplace();
  if (surfaceModel != gz::math::SphericalCoordinates::CUSTOM_SURFACE)
  {
    this->sphericalCoordinates =
        gz::math::SphericalCoordinates(surfaceModel, latitude, longitude,
        elevation, heading);
  }
  else
  {
    this->sphericalCoordinates =
      gz::math::SphericalCoordinates(surfaceModel,
          axisEquatorial, axisPolar);

    this->sphericalCoordinates->SetLatitudeReference(latitude);
    this->sphericalCoordinates->SetLongitudeReference(longitude);
    this->sphericalCoordinates->SetElevationReference(elevation);
    this->sphericalCoordinates->SetHeadingOffset(heading);
  }

  return errors;
}

/////////////////////////////////////////////////
sdf::ElementPtr World::ToElement(const OutputConfig &_config) const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("world.sdf", elem);

  elem->GetAttribute("name")->Set(this->Name());
  elem->GetElement("gravity")->Set(this->Gravity());
  elem->GetElement("magnetic_field")->Set(this->MagneticField());

  sdf::ElementPtr windElem = elem->GetElement("wind");
  windElem->GetElement("linear_velocity")->Set(this->WindLinearVelocity());

  // Physics
  for (const sdf::Physics &physics : this->dataPtr->physics)
    elem->InsertElement(physics.ToElement(), true);

  // Models
  for (const sdf::Model &model : this->dataPtr->models)
    elem->InsertElement(model.ToElement(_config), true);

  // Actors
  for (const sdf::Actor &actor : this->dataPtr->actors)
    elem->InsertElement(actor.ToElement(), true);

  // Joints
  for (const sdf::Joint &joint : this->dataPtr->joints)
    elem->InsertElement(joint.ToElement(), true);

  // Lights
  for (const sdf::Light &light : this->dataPtr->lights)
    elem->InsertElement(light.ToElement(), true);

  // Frames
  for (const sdf::Frame &frame : this->dataPtr->frames)
    elem->InsertElement(frame.ToElement(), true);

  // Spherical coordinates.
  if (this->dataPtr->sphericalCoordinates)
  {
    sdf::ElementPtr sphericalElem = elem->GetElement("spherical_coordinates");
    sphericalElem->GetElement("surface_model")->Set(
        gz::math::SphericalCoordinates::Convert(
          this->dataPtr->sphericalCoordinates->Surface()));
    sphericalElem->GetElement("world_frame_orientation")->Set("ENU");
    sphericalElem->GetElement("latitude_deg")->Set(
        this->dataPtr->sphericalCoordinates->LatitudeReference().Degree());
    sphericalElem->GetElement("longitude_deg")->Set(
        this->dataPtr->sphericalCoordinates->LongitudeReference().Degree());
    sphericalElem->GetElement("elevation")->Set(
        this->dataPtr->sphericalCoordinates->ElevationReference());
    sphericalElem->GetElement("heading_deg")->Set(
        this->dataPtr->sphericalCoordinates->HeadingOffset().Degree());
    sphericalElem->GetElement("surface_axis_equatorial")->Set(
        this->dataPtr->sphericalCoordinates->SurfaceAxisEquatorial());
    sphericalElem->GetElement("surface_axis_polar")->Set(
        this->dataPtr->sphericalCoordinates->SurfaceAxisPolar());
  }

  // Atmosphere
  elem->InsertElement(this->dataPtr->atmosphere.ToElement(), true);

  // Gui
  if (this->dataPtr->gui)
    elem->InsertElement(this->dataPtr->gui->ToElement(), true);

  // Scene
  elem->InsertElement(this->dataPtr->scene.ToElement(), true);

  // Audio
  if (this->dataPtr->audioDevice != "default")
    elem->GetElement("audio")->GetElement("device")->Set(this->AudioDevice());

  // Add in the plugins
  for (const Plugin &plugin : this->dataPtr->plugins)
    elem->InsertElement(plugin.ToElement(), true);

  return elem;
}

/////////////////////////////////////////////////
void World::ClearModels()
{
  this->dataPtr->models.clear();
}

/////////////////////////////////////////////////
void World::ClearActors()
{
  this->dataPtr->actors.clear();
}

/////////////////////////////////////////////////
void World::ClearJoints()
{
  this->dataPtr->joints.clear();
}

/////////////////////////////////////////////////
void World::ClearLights()
{
  this->dataPtr->lights.clear();
}

/////////////////////////////////////////////////
void World::ClearPhysics()
{
  this->dataPtr->physics.clear();
}

/////////////////////////////////////////////////
void World::ClearFrames()
{
  this->dataPtr->frames.clear();
}

/////////////////////////////////////////////////
bool World::NameExistsInFrameAttachedToGraph(const std::string &_name) const
{
  if (!this->dataPtr->frameAttachedToGraph)
    return false;

  return this->dataPtr->frameAttachedToGraph.VertexIdByName(_name)
      !=  gz::math::graph::kNullId;
}

/////////////////////////////////////////////////
bool World::AddModel(const Model &_model)
{
  if (this->ModelNameExists(_model.Name()))
    return false;
  this->dataPtr->models.push_back(_model);
  return true;
}

/////////////////////////////////////////////////
bool World::AddActor(const Actor &_actor)
{
  if (this->ActorNameExists(_actor.Name()))
    return false;
  this->dataPtr->actors.push_back(_actor);

  return true;
}

/////////////////////////////////////////////////
bool World::AddJoint(const Joint &_joint)
{
  if (this->JointNameExists(_joint.Name()))
    return false;
  this->dataPtr->joints.push_back(_joint);

  return true;
}

/////////////////////////////////////////////////
bool World::AddLight(const Light &_light)
{
  if (this->LightNameExists(_light.Name()))
    return false;
  this->dataPtr->lights.push_back(_light);

  return true;
}

/////////////////////////////////////////////////
bool World::AddPhysics(const Physics &_physics)
{
  if (this->PhysicsNameExists(_physics.Name()))
    return false;
  this->dataPtr->physics.push_back(_physics);

  return true;
}

/////////////////////////////////////////////////
bool World::AddFrame(const Frame &_frame)
{
  if (this->FrameNameExists(_frame.Name()))
    return false;
  this->dataPtr->frames.push_back(_frame);

  return true;
}

/////////////////////////////////////////////////
const sdf::Plugins &World::Plugins() const
{
  return this->dataPtr->plugins;
}

/////////////////////////////////////////////////
sdf::Plugins &World::Plugins()
{
  return this->dataPtr->plugins;
}

/////////////////////////////////////////////////
void World::ClearPlugins()
{
  this->dataPtr->plugins.clear();
}

/////////////////////////////////////////////////
void World::AddPlugin(const Plugin &_plugin)
{
  this->dataPtr->plugins.push_back(_plugin);
}
