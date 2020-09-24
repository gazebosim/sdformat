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
#include <ignition/math/Vector3.hh>

#include "sdf/Actor.hh"
#include "sdf/Frame.hh"
#include "sdf/Light.hh"
#include "sdf/Model.hh"
#include "sdf/Physics.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"
#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::WorldPrivate
{
  /// \brief Default constructor
  public: WorldPrivate();

  /// \brief Copy constructor
  /// \param[in] _worldPrivate Joint axis to move.
  public: explicit WorldPrivate(const WorldPrivate &_worldPrivate);

  // Delete copy assignment so it is not accidentally used
  public: WorldPrivate &operator=(const WorldPrivate &) = delete;

  /// \brief Pointer to an atmosphere model.
  public: std::unique_ptr<Atmosphere> atmosphere;

  /// \brief Audio device name
  public: std::string audioDevice = "default";

  /// \brief Gravity vector.
  public: ignition::math::Vector3d gravity =
           ignition::math::Vector3d(0, 0, -9.80665);

  /// \brief Pointer to Gui parameters.
  public: std::unique_ptr<Gui> gui;

  /// \brief Pointer to Sene parameters.
  public: std::unique_ptr<Scene> scene;

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

  /// \brief Name of the world.
  public: std::string name = "";

  /// \brief The physics profiles specified in this world.
  public: std::vector<Physics> physics;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief Linear velocity of wind.
  public: ignition::math::Vector3d windLinearVelocity =
           ignition::math::Vector3d::Zero;

  /// \brief Frame Attached-To Graph constructed during Load.
  // public: std::shared_ptr<sdf::FrameAttachedToGraph> frameAttachedToGraph;
  public: std::shared_ptr<sdf::FrameAttachedToGraph> ownedFrameAttachedToGraph;
  public: sdf::ScopedGraph<sdf::FrameAttachedToGraph> frameAttachedToGraph;

  /// \brief Pose Relative-To Graph constructed during Load.
  public: std::shared_ptr<sdf::PoseRelativeToGraph> ownedPoseRelativeToGraph;
  public: sdf::ScopedGraph<sdf::PoseRelativeToGraph> poseRelativeToGraph;
};

/////////////////////////////////////////////////
WorldPrivate::WorldPrivate()
    : ownedFrameAttachedToGraph(std::make_shared<sdf::FrameAttachedToGraph>())
    , frameAttachedToGraph(ownedFrameAttachedToGraph)
    , ownedPoseRelativeToGraph(std::make_shared<sdf::PoseRelativeToGraph>())
    , poseRelativeToGraph(ownedPoseRelativeToGraph)
{
}

/////////////////////////////////////////////////
WorldPrivate::WorldPrivate(const WorldPrivate &_worldPrivate)
    : audioDevice(_worldPrivate.audioDevice),
      gravity(_worldPrivate.gravity),
      frames(_worldPrivate.frames),
      lights(_worldPrivate.lights),
      actors(_worldPrivate.actors),
      magneticField(_worldPrivate.magneticField),
      models(_worldPrivate.models),
      name(_worldPrivate.name),
      physics(_worldPrivate.physics),
      sdf(_worldPrivate.sdf),
      windLinearVelocity(_worldPrivate.windLinearVelocity)
{
  if (_worldPrivate.atmosphere)
  {
    this->atmosphere =
        std::make_unique<Atmosphere>(*(_worldPrivate.atmosphere));
  }
  if (_worldPrivate.gui)
  {
    this->gui = std::make_unique<Gui>(*(_worldPrivate.gui));
  }
  if (_worldPrivate.ownedFrameAttachedToGraph)
  {
    this->ownedFrameAttachedToGraph =
        std::make_shared<sdf::FrameAttachedToGraph>(
            *(_worldPrivate.ownedFrameAttachedToGraph));
    this->frameAttachedToGraph = this->ownedFrameAttachedToGraph;
  }
  if (_worldPrivate.ownedPoseRelativeToGraph)
  {
    this->ownedPoseRelativeToGraph = std::make_shared<sdf::PoseRelativeToGraph>(
        *(_worldPrivate.ownedPoseRelativeToGraph));
    this->poseRelativeToGraph = this->ownedPoseRelativeToGraph;
  }
  if (_worldPrivate.scene)
  {
    this->scene = std::make_unique<Scene>(*(_worldPrivate.scene));
  }
}

/////////////////////////////////////////////////
World::World()
  : dataPtr(new WorldPrivate)
{
  this->dataPtr->physics.emplace_back(Physics());
}

/////////////////////////////////////////////////
World::~World()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
World::World(const World &_world)
  : dataPtr(new WorldPrivate(*_world.dataPtr))
{
  for (auto &frame : this->dataPtr->frames)
  {
    frame.SetFrameAttachedToGraph(this->dataPtr->frameAttachedToGraph);
    frame.SetPoseRelativeToGraph(this->dataPtr->poseRelativeToGraph);
  }
  for (auto &model : this->dataPtr->models)
  {
    model.SetPoseRelativeToGraph(this->dataPtr->poseRelativeToGraph);
  }
  for (auto &light : this->dataPtr->lights)
  {
    light.SetXmlParentName("world");
    light.SetPoseRelativeToGraph(this->dataPtr->poseRelativeToGraph);
  }
}

/////////////////////////////////////////////////
World::World(World &&_world) noexcept
  : dataPtr(std::exchange(_world.dataPtr, nullptr))
{
}

/////////////////////////////////////////////////
World &World::operator=(const World &_world)
{
  return *this = World(_world);
}

/////////////////////////////////////////////////
World &World::operator=(World &&_world)
{
  std::swap(this->dataPtr, _world.dataPtr);
  return *this;
}

/////////////////////////////////////////////////
Errors World::Load(sdf::ElementPtr _sdf)
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
    this->dataPtr->atmosphere.reset(new sdf::Atmosphere());
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

  std::function <void(Model &)> beforeLoad = [this](Model &_model)
  {
    _model.SetPoseRelativeToGraph(this->dataPtr->poseRelativeToGraph);
    _model.SetFrameAttachedToGraph(this->dataPtr->frameAttachedToGraph);
  };
  // Load all the models.
  Errors modelLoadErrors = loadUniqueRepeated<Model>(_sdf, "model",
      this->dataPtr->models, beforeLoad);
  errors.insert(errors.end(), modelLoadErrors.begin(), modelLoadErrors.end());

  // Models are loaded first, and loadUniqueRepeated ensures there are no
  // duplicate names, so these names can be added to frameNames without
  // checking uniqueness.
  for (const auto &model : this->dataPtr->models)
  {
    frameNames.insert(model.Name());
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
    this->dataPtr->gui.reset(new sdf::Gui());
    Errors guiLoadErrors = this->dataPtr->gui->Load(_sdf->GetElement("gui"));
    errors.insert(errors.end(), guiLoadErrors.begin(), guiLoadErrors.end());
  }

  // Load the Scene
  if (_sdf->HasElement("scene"))
  {
    this->dataPtr->scene.reset(new sdf::Scene());
    Errors sceneLoadErrors =
        this->dataPtr->scene->Load(_sdf->GetElement("scene"));
    errors.insert(errors.end(), sceneLoadErrors.begin(), sceneLoadErrors.end());
  }

  // Build the graphs.
  Errors frameAttachedToGraphErrors =
  buildFrameAttachedToGraph(this->dataPtr->frameAttachedToGraph, this);
  errors.insert(errors.end(), frameAttachedToGraphErrors.begin(),
                              frameAttachedToGraphErrors.end());
  Errors validateFrameAttachedGraphErrors =
    validateFrameAttachedToGraph(this->dataPtr->frameAttachedToGraph);
  errors.insert(errors.end(), validateFrameAttachedGraphErrors.begin(),
                              validateFrameAttachedGraphErrors.end());
  for (auto &frame : this->dataPtr->frames)
  {
    frame.SetFrameAttachedToGraph(this->dataPtr->frameAttachedToGraph);
  }

  Errors poseRelativeToGraphErrors =
  buildPoseRelativeToGraph(this->dataPtr->poseRelativeToGraph, this);
  errors.insert(errors.end(), poseRelativeToGraphErrors.begin(),
                              poseRelativeToGraphErrors.end());
  Errors validatePoseGraphErrors =
    validatePoseRelativeToGraph(this->dataPtr->poseRelativeToGraph);
  errors.insert(errors.end(), validatePoseGraphErrors.begin(),
                              validatePoseGraphErrors.end());
  for (auto &frame : this->dataPtr->frames)
  {
    frame.SetPoseRelativeToGraph(this->dataPtr->poseRelativeToGraph);
  }
  for (auto &light : this->dataPtr->lights)
  {
    light.SetXmlParentName("world");
    light.SetPoseRelativeToGraph(this->dataPtr->poseRelativeToGraph);
  }

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
const Model *World::ModelByName(const std::string &_name) const
{
  for (auto const &m : this->dataPtr->models)
  {
    if (m.Name() == _name)
    {
      return &m;
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
const sdf::Atmosphere *World::Atmosphere() const
{
  return this->dataPtr->atmosphere.get();
}

/////////////////////////////////////////////////
void World::SetAtmosphere(const sdf::Atmosphere &_atmosphere) const
{
  this->dataPtr->atmosphere.reset(new sdf::Atmosphere(_atmosphere));
}

/////////////////////////////////////////////////
sdf::Gui *World::Gui() const
{
  return this->dataPtr->gui.get();
}

/////////////////////////////////////////////////
void World::SetGui(const sdf::Gui &_gui)
{
  return this->dataPtr->gui.reset(new sdf::Gui(_gui));
}

/////////////////////////////////////////////////
const sdf::Scene *World::Scene() const
{
  return this->dataPtr->scene.get();
}

/////////////////////////////////////////////////
void World::SetScene(const sdf::Scene &_scene)
{
  return this->dataPtr->scene.reset(new sdf::Scene(_scene));
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
  for (auto const &f : this->dataPtr->frames)
  {
    if (f.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
const Frame *World::FrameByName(const std::string &_name) const
{
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
