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
#include <memory>
#include <string>
#include <vector>
#include <gz/math/Inertial.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include "sdf/Collision.hh"
#include "sdf/Error.hh"
#include "sdf/Light.hh"
#include "sdf/Link.hh"
#include "sdf/parser.hh"
#include "sdf/ParticleEmitter.hh"
#include "sdf/Projector.hh"
#include "sdf/Sensor.hh"
#include "sdf/Types.hh"
#include "sdf/Visual.hh"
#include "sdf/ParserConfig.hh"
#include "sdf/Element.hh"

#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::Link::Implementation
{
  /// \brief Name of the link.
  public: std::string name = "";

  /// \brief Pose of the link
  public: gz::math::Pose3d pose = gz::math::Pose3d::Zero;

  /// \brief Frame of the pose.
  public: std::string poseRelativeTo = "";

  /// \brief The visuals specified in this link.
  public: std::vector<Visual> visuals;

  /// \brief The lights specified in this link.
  public: std::vector<Light> lights;

  /// \brief The collisions specified in this link.
  public: std::vector<Collision> collisions;

  /// \brief The sensors specified in this link.
  public: std::vector<Sensor> sensors;

  /// \brief The particle emitters specified in this link.
  public: std::vector<ParticleEmitter> emitters;

  /// \brief The projectors specified in this link.
  public: std::vector<Projector> projectors;

  /// \brief Density of the inertial which will be used for auto-inertial
  /// calculations if the collision density has not been set.
  public: std::optional<double> density;

  /// \brief SDF element pointer to <auto_inertia_params> tag
  public: sdf::ElementPtr autoInertiaParams{nullptr};

  /// \brief The inertial information for this link.
  public: gz::math::Inertiald inertial {{1.0,
            gz::math::Vector3d::One, gz::math::Vector3d::Zero},
            gz::math::Pose3d::Zero};

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief True if this link should be subject to wind, false otherwise.
  public: bool enableWind = false;

  /// \brief True if this link is kinematic only
  public: bool kinematic = false;

  /// \brief True if automatic caluclation for the link inertial is enabled
  public: bool autoInertia = false;

  /// \brief This variable is used to track whether the inertia values for
  /// link was saved in CalculateInertial()
  public: bool autoInertiaSaved = false;

  /// \brief Scoped Pose Relative-To graph at the parent model scope.
  public: sdf::ScopedGraph<sdf::PoseRelativeToGraph> poseRelativeToGraph;
};

/////////////////////////////////////////////////
Link::Link()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Link::Load(ElementPtr _sdf)
{
  return this->Load(_sdf, ParserConfig::GlobalConfig());
}

/////////////////////////////////////////////////
Errors Link::Load(ElementPtr _sdf, const ParserConfig &_config)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <link>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "link")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Link, but the provided SDF element is not a "
        "<link>."});
    return errors;
  }

  // Read the links's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A link name is required, but the name is not set."});
  }

  // Check that the link's name is valid
  if (isReservedName(this->dataPtr->name))
  {
    errors.push_back({ErrorCode::RESERVED_NAME,
                     "The supplied link name [" + this->dataPtr->name +
                     "] is reserved."});
  }

  // Load the pose. Ignore the return value since the pose is optional.
  loadPose(_sdf, this->dataPtr->pose, this->dataPtr->poseRelativeTo);

  // Load all the visuals.
  Errors visLoadErrors = loadUniqueRepeated<Visual>(_sdf, "visual",
      this->dataPtr->visuals, _config);
  errors.insert(errors.end(), visLoadErrors.begin(), visLoadErrors.end());

  // Load all the collisions.
  Errors collLoadErrors = loadUniqueRepeated<Collision>(_sdf, "collision",
      this->dataPtr->collisions, _config);
  errors.insert(errors.end(), collLoadErrors.begin(), collLoadErrors.end());

  // Load all the lights.
  Errors lightLoadErrors = loadUniqueRepeated<Light>(_sdf, "light",
      this->dataPtr->lights);
  errors.insert(errors.end(), lightLoadErrors.begin(), lightLoadErrors.end());

  // Load all the sensors.
  Errors sensorLoadErrors = loadUniqueRepeated<Sensor>(_sdf, "sensor",
      this->dataPtr->sensors);
  errors.insert(errors.end(), sensorLoadErrors.begin(), sensorLoadErrors.end());

  // Load all the particle emitters.
  Errors emitterLoadErrors = loadUniqueRepeated<ParticleEmitter>(_sdf,
      "particle_emitter", this->dataPtr->emitters);
  errors.insert(errors.end(), emitterLoadErrors.begin(),
      emitterLoadErrors.end());

  // Load all the projectors
  Errors projectorLoadErrors = loadUniqueRepeated<Projector>(_sdf,
      "projector", this->dataPtr->projectors);
  errors.insert(errors.end(), projectorLoadErrors.begin(),
      projectorLoadErrors.end());

  gz::math::Vector3d xxyyzz = gz::math::Vector3d::One;
  gz::math::Vector3d xyxzyz = gz::math::Vector3d::Zero;
  gz::math::Pose3d inertiaPose;
  std::string inertiaFrame = "";
  double mass = 1.0;

  if (_sdf->HasElement("inertial"))
  {
    sdf::ElementPtr inertialElem = _sdf->GetElement("inertial");

    if (inertialElem->HasElement("density"))
    {
      this->dataPtr->density = inertialElem->Get<double>("density");
    }

    // Load the auto_inertia_params element
    if (inertialElem->HasElement("auto_inertia_params"))
    {
      this->dataPtr->autoInertiaParams =
        inertialElem->GetElement("auto_inertia_params");
    }

    if (inertialElem->Get<bool>("auto"))
    {
      this->dataPtr->autoInertia = true;
      // If auto is to true but user has still provided
      // inertial values
      if (inertialElem->HasElement("pose") ||
          inertialElem->HasElement("mass") ||
          inertialElem->HasElement("inertia"))
      {
        Error err(
          ErrorCode::WARNING,
          "Inertial was used with auto=true for the link named " +
          this->dataPtr->name + ", but user-defined inertial values were "
          "found, which will be overwritten by the computed inertial values."
        );
        enforceConfigurablePolicyCondition(
          _config.WarningsPolicy(), err, errors);
      }
    }
    // If auto is set to false or not specified
    else
    {
      if (inertialElem->HasElement("pose"))
        loadPose(inertialElem->GetElement("pose"), inertiaPose, inertiaFrame);

      // Get the mass.
      mass = inertialElem->Get<double>("mass", 1.0).first;

      if (inertialElem->HasElement("inertia"))
      {
        sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");

        xxyyzz.X(inertiaElem->Get<double>("ixx", 1.0).first);
        xxyyzz.Y(inertiaElem->Get<double>("iyy", 1.0).first);
        xxyyzz.Z(inertiaElem->Get<double>("izz", 1.0).first);

        xyxzyz.X(inertiaElem->Get<double>("ixy", 0.0).first);
        xyxzyz.Y(inertiaElem->Get<double>("ixz", 0.0).first);
        xyxzyz.Z(inertiaElem->Get<double>("iyz", 0.0).first);
      }
    }

    if (inertialElem->HasElement("fluid_added_mass"))
    {
      auto addedMassElem = inertialElem->GetElement("fluid_added_mass");

      gz::math::Matrix6d addedMass;
      addedMass(0, 0) = addedMassElem->Get<double>("xx", 0.0).first;
      addedMass(0, 1) = addedMassElem->Get<double>("xy", 0.0).first;
      addedMass(0, 2) = addedMassElem->Get<double>("xz", 0.0).first;
      addedMass(0, 3) = addedMassElem->Get<double>("xp", 0.0).first;
      addedMass(0, 4) = addedMassElem->Get<double>("xq", 0.0).first;
      addedMass(0, 5) = addedMassElem->Get<double>("xr", 0.0).first;

      addedMass(1, 0) = addedMass(0, 1);
      addedMass(1, 1) = addedMassElem->Get<double>("yy", 0.0).first;
      addedMass(1, 2) = addedMassElem->Get<double>("yz", 0.0).first;
      addedMass(1, 3) = addedMassElem->Get<double>("yp", 0.0).first;
      addedMass(1, 4) = addedMassElem->Get<double>("yq", 0.0).first;
      addedMass(1, 5) = addedMassElem->Get<double>("yr", 0.0).first;

      addedMass(2, 0) = addedMass(0, 2);
      addedMass(2, 1) = addedMass(1, 2);
      addedMass(2, 2) = addedMassElem->Get<double>("zz", 0.0).first;
      addedMass(2, 3) = addedMassElem->Get<double>("zp", 0.0).first;
      addedMass(2, 4) = addedMassElem->Get<double>("zq", 0.0).first;
      addedMass(2, 5) = addedMassElem->Get<double>("zr", 0.0).first;

      addedMass(3, 0) = addedMass(0, 3);
      addedMass(3, 1) = addedMass(1, 3);
      addedMass(3, 2) = addedMass(2, 3);
      addedMass(3, 3) = addedMassElem->Get<double>("pp", 0.0).first;
      addedMass(3, 4) = addedMassElem->Get<double>("pq", 0.0).first;
      addedMass(3, 5) = addedMassElem->Get<double>("pr", 0.0).first;

      addedMass(4, 0) = addedMass(0, 4);
      addedMass(4, 1) = addedMass(1, 4);
      addedMass(4, 2) = addedMass(2, 4);
      addedMass(4, 3) = addedMass(3, 4);
      addedMass(4, 4) = addedMassElem->Get<double>("qq", 0.0).first;
      addedMass(4, 5) = addedMassElem->Get<double>("qr", 0.0).first;

      addedMass(5, 0) = addedMass(0, 5);
      addedMass(5, 1) = addedMass(1, 5);
      addedMass(5, 2) = addedMass(2, 5);
      addedMass(5, 3) = addedMass(3, 5);
      addedMass(5, 4) = addedMass(4, 5);
      addedMass(5, 5) = addedMassElem->Get<double>("rr", 0.0).first;

      if (!this->dataPtr->inertial.SetFluidAddedMass(addedMass))
      {
        errors.push_back({ErrorCode::LINK_INERTIA_INVALID,
                         "A link named " +
                         this->Name() +
                         " has invalid fluid added mass."});
      }
    }
  }

  if (!this->dataPtr->inertial.SetMassMatrix(
      gz::math::MassMatrix3d(mass, xxyyzz, xyxzyz)))
  {
    errors.push_back({ErrorCode::LINK_INERTIA_INVALID,
                     "A link named " +
                     this->Name() +
                     " has invalid inertia."});
  }

  /// \todo: Handle inertia frame properly
  this->dataPtr->inertial.SetPose(inertiaPose);

  this->dataPtr->enableWind = _sdf->Get<bool>("enable_wind",
      this->dataPtr->enableWind).first;

  this->dataPtr->kinematic = _sdf->Get<bool>("kinematic",
      this->dataPtr->kinematic).first;

  return errors;
}

/////////////////////////////////////////////////
std::string Link::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Link::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
std::optional<double> Link::Density() const
{
  return this->dataPtr->density;
}

/////////////////////////////////////////////////
void Link::SetDensity(double _density)
{
  this->dataPtr->density = _density;
}

/////////////////////////////////////////////////
sdf::ElementPtr Link::AutoInertiaParams() const
{
  return this->dataPtr->autoInertiaParams;
}

/////////////////////////////////////////////////
void Link::SetAutoInertiaParams(const sdf::ElementPtr _autoInertiaParams)
{
  this->dataPtr->autoInertiaParams = _autoInertiaParams;
}

/////////////////////////////////////////////////
uint64_t Link::VisualCount() const
{
  return this->dataPtr->visuals.size();
}

/////////////////////////////////////////////////
const Visual *Link::VisualByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->visuals.size())
    return &this->dataPtr->visuals[_index];
  return nullptr;
}

/////////////////////////////////////////////////
Visual *Link::VisualByIndex(uint64_t _index)
{
  return const_cast<Visual*>(
      static_cast<const Link*>(this)->VisualByIndex(_index));
}

/////////////////////////////////////////////////
bool Link::VisualNameExists(const std::string &_name) const
{
  for (auto const &v : this->dataPtr->visuals)
  {
    if (v.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
uint64_t Link::CollisionCount() const
{
  return this->dataPtr->collisions.size();
}

/////////////////////////////////////////////////
const Collision *Link::CollisionByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->collisions.size())
    return &this->dataPtr->collisions[_index];
  return nullptr;
}

/////////////////////////////////////////////////
Collision *Link::CollisionByIndex(uint64_t _index)
{
  return const_cast<Collision*>(
      static_cast<const Link*>(this)->CollisionByIndex(_index));
}

/////////////////////////////////////////////////
bool Link::CollisionNameExists(const std::string &_name) const
{
  for (auto const &c : this->dataPtr->collisions)
  {
    if (c.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
uint64_t Link::LightCount() const
{
  return this->dataPtr->lights.size();
}

/////////////////////////////////////////////////
const Light *Link::LightByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->lights.size())
    return &this->dataPtr->lights[_index];
  return nullptr;
}

/////////////////////////////////////////////////
Light *Link::LightByIndex(uint64_t _index)
{
  return const_cast<Light*>(
      static_cast<const Link*>(this)->LightByIndex(_index));
}

/////////////////////////////////////////////////
bool Link::LightNameExists(const std::string &_name) const
{
  return this->LightByName(_name) != nullptr;
}

/////////////////////////////////////////////////
uint64_t Link::SensorCount() const
{
  return this->dataPtr->sensors.size();
}

/////////////////////////////////////////////////
const Sensor *Link::SensorByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->sensors.size())
    return &this->dataPtr->sensors[_index];
  return nullptr;
}

/////////////////////////////////////////////////
Sensor *Link::SensorByIndex(uint64_t _index)
{
  return const_cast<Sensor*>(
      static_cast<const Link*>(this)->SensorByIndex(_index));
}

/////////////////////////////////////////////////
bool Link::SensorNameExists(const std::string &_name) const
{
  for (auto const &s : this->dataPtr->sensors)
  {
    if (s.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
const Sensor *Link::SensorByName(const std::string &_name) const
{
  for (auto const &s : this->dataPtr->sensors)
  {
    if (s.Name() == _name)
    {
      return &s;
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
Sensor *Link::SensorByName(const std::string &_name)
{
  return const_cast<Sensor*>(
      static_cast<const Link*>(this)->SensorByName(_name));
}

/////////////////////////////////////////////////
uint64_t Link::ParticleEmitterCount() const
{
  return this->dataPtr->emitters.size();
}

/////////////////////////////////////////////////
const ParticleEmitter *Link::ParticleEmitterByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->emitters.size())
    return &this->dataPtr->emitters[_index];
  return nullptr;
}

/////////////////////////////////////////////////
ParticleEmitter *Link::ParticleEmitterByIndex(uint64_t _index)
{
  return const_cast<ParticleEmitter*>(
      static_cast<const Link*>(this)->ParticleEmitterByIndex(_index));
}

/////////////////////////////////////////////////
bool Link::ParticleEmitterNameExists(const std::string &_name) const
{
  for (auto const &e : this->dataPtr->emitters)
  {
    if (e.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
const ParticleEmitter *Link::ParticleEmitterByName(
    const std::string &_name) const
{
  for (auto const &e : this->dataPtr->emitters)
  {
    if (e.Name() == _name)
    {
      return &e;
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
ParticleEmitter *Link::ParticleEmitterByName(const std::string &_name)
{
  return const_cast<ParticleEmitter *>(
      static_cast<const Link*>(this)->ParticleEmitterByName(_name));
}

/////////////////////////////////////////////////
uint64_t Link::ProjectorCount() const
{
  return this->dataPtr->projectors.size();
}

/////////////////////////////////////////////////
const Projector *Link::ProjectorByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->projectors.size())
    return &this->dataPtr->projectors[_index];
  return nullptr;
}

/////////////////////////////////////////////////
Projector *Link::ProjectorByIndex(uint64_t _index)
{
  return const_cast<Projector*>(
      static_cast<const Link*>(this)->ProjectorByIndex(_index));
}

/////////////////////////////////////////////////
bool Link::ProjectorNameExists(const std::string &_name) const
{
  for (auto const &e : this->dataPtr->projectors)
  {
    if (e.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
const Projector *Link::ProjectorByName(
    const std::string &_name) const
{
  for (auto const &e : this->dataPtr->projectors)
  {
    if (e.Name() == _name)
    {
      return &e;
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
Projector *Link::ProjectorByName(const std::string &_name)
{
  return const_cast<Projector *>(
      static_cast<const Link*>(this)->ProjectorByName(_name));
}

/////////////////////////////////////////////////
const gz::math::Inertiald &Link::Inertial() const
{
  return this->dataPtr->inertial;
}

/////////////////////////////////////////////////
bool Link::SetInertial(const gz::math::Inertiald &_inertial)
{
  this->dataPtr->inertial = _inertial;
  return _inertial.MassMatrix().IsValid();
}

/////////////////////////////////////////////////
Errors Link::ResolveInertial(
  gz::math::Inertiald &_inertial,
  const std::string &_resolveTo) const
{
  gz::math::Pose3d linkPose;
  auto errors = this->SemanticPose().Resolve(linkPose, _resolveTo);
  if (errors.empty())
  {
    _inertial = this->dataPtr->inertial;
    _inertial.SetPose(linkPose * _inertial.Pose());
  }
  return errors;
}

/////////////////////////////////////////////////
void Link::ResolveAutoInertials(sdf::Errors &_errors,
  const ParserConfig &_config)
{
  // If inertial calculations is set to automatic & the inertial values for the
  // link was not saved previously
  if (this->dataPtr->autoInertia && !this->dataPtr->autoInertiaSaved)
  {
    // Return an error if auto is set to true but there are no
    // collision elements in the link
    if (this->dataPtr->collisions.empty())
    {
      _errors.push_back({ErrorCode::ELEMENT_MISSING,
                        "Inertial is set to auto but there are no "
                        "<collision> elements for the link named " +
                        this->Name() + "."});
      return;
    }

    gz::math::Inertiald totalInertia;

    for (sdf::Collision &collision : this->dataPtr->collisions)
    {
      gz::math::Inertiald collisionInertia;
      collision.CalculateInertial(_errors, collisionInertia, _config,
                                  this->dataPtr->density,
                                  this->dataPtr->autoInertiaParams);
      totalInertia = totalInertia + collisionInertia;
    }

    this->dataPtr->inertial = totalInertia;

    // If CalculateInertial() was called with SAVE_CALCULATION
    // configuration then set autoInertiaSaved to true
    if (_config.CalculateInertialConfiguration() ==
      ConfigureResolveAutoInertials::SAVE_CALCULATION)
    {
      this->dataPtr->autoInertiaSaved = true;
    }
    else if (_config.CalculateInertialConfiguration() ==
      ConfigureResolveAutoInertials::SAVE_CALCULATION_IN_ELEMENT)
    {
      this->dataPtr->autoInertiaSaved = true;
      // Write calculated inertia values to //link/inertial element
      auto inertialElem = this->dataPtr->sdf->GetElement("inertial");
      inertialElem->GetElement("pose")->GetValue()->Set<gz::math::Pose3d>(
        totalInertia.Pose());
      inertialElem->GetElement("mass")->GetValue()->Set<double>(
        totalInertia.MassMatrix().Mass());
      auto momentOfInertiaElem = inertialElem->GetElement("inertia");
      momentOfInertiaElem->GetElement("ixx")->GetValue()->Set<double>(
        totalInertia.MassMatrix().Ixx());
      momentOfInertiaElem->GetElement("ixy")->GetValue()->Set<double>(
        totalInertia.MassMatrix().Ixy());
      momentOfInertiaElem->GetElement("ixz")->GetValue()->Set<double>(
        totalInertia.MassMatrix().Ixz());
      momentOfInertiaElem->GetElement("iyy")->GetValue()->Set<double>(
        totalInertia.MassMatrix().Iyy());
      momentOfInertiaElem->GetElement("iyz")->GetValue()->Set<double>(
        totalInertia.MassMatrix().Iyz());
      momentOfInertiaElem->GetElement("izz")->GetValue()->Set<double>(
        totalInertia.MassMatrix().Izz());
    }
  }
  // If auto is false, this means inertial values were set
  // from user given values in Link::Load(), therefore we can return
  else
  {
    return;
  }
}

/////////////////////////////////////////////////
const gz::math::Pose3d &Link::RawPose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
const std::string &Link::PoseRelativeTo() const
{
  return this->dataPtr->poseRelativeTo;
}

/////////////////////////////////////////////////
void Link::SetRawPose(const gz::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
void Link::SetPoseRelativeTo(const std::string &_frame)
{
  this->dataPtr->poseRelativeTo = _frame;
}

/////////////////////////////////////////////////
void Link::SetPoseRelativeToGraph(sdf::ScopedGraph<PoseRelativeToGraph> _graph)
{
  this->dataPtr->poseRelativeToGraph = _graph;

  // Pass graph to child elements.
  for (auto &collision : this->dataPtr->collisions)
  {
    collision.SetXmlParentName(this->dataPtr->name);
    collision.SetPoseRelativeToGraph(_graph);
  }
  for (auto &light : this->dataPtr->lights)
  {
    light.SetXmlParentName(this->dataPtr->name);
    light.SetPoseRelativeToGraph(_graph);
  }
  for (auto &sensor : this->dataPtr->sensors)
  {
    sensor.SetXmlParentName(this->dataPtr->name);
    sensor.SetPoseRelativeToGraph(_graph);
  }
  for (auto &visual : this->dataPtr->visuals)
  {
    visual.SetXmlParentName(this->dataPtr->name);
    visual.SetPoseRelativeToGraph(_graph);
  }

  for (auto &emitter : this->dataPtr->emitters)
  {
    emitter.SetXmlParentName(this->dataPtr->name);
    emitter.SetPoseRelativeToGraph(_graph);
  }

  for (auto &projector : this->dataPtr->projectors)
  {
    projector.SetXmlParentName(this->dataPtr->name);
    projector.SetPoseRelativeToGraph(_graph);
  }
}

/////////////////////////////////////////////////
sdf::SemanticPose Link::SemanticPose() const
{
  return sdf::SemanticPose(
      this->dataPtr->name,
      this->dataPtr->pose,
      this->dataPtr->poseRelativeTo,
      "__model__",
      this->dataPtr->poseRelativeToGraph);
}

/////////////////////////////////////////////////
const Visual *Link::VisualByName(const std::string &_name) const
{
  for (auto const &v : this->dataPtr->visuals)
  {
    if (v.Name() == _name)
    {
      return &v;
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
Visual *Link::VisualByName(const std::string &_name)
{
  return const_cast<Visual *>(
      static_cast<const Link*>(this)->VisualByName(_name));
}

/////////////////////////////////////////////////
const Collision *Link::CollisionByName(const std::string &_name) const
{
  for (auto &c : this->dataPtr->collisions)
  {
    if (c.Name() == _name)
    {
      return &c;
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
Collision *Link::CollisionByName(const std::string &_name)
{
  return const_cast<Collision *>(
      static_cast<const Link*>(this)->CollisionByName(_name));
}

/////////////////////////////////////////////////
const Light *Link::LightByName(const std::string &_name) const
{
  for (auto const &c : this->dataPtr->lights)
  {
    if (c.Name() == _name)
    {
      return &c;
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
Light *Link::LightByName(const std::string &_name)
{
  return const_cast<Light *>(
      static_cast<const Link*>(this)->LightByName(_name));

}

/////////////////////////////////////////////////
sdf::ElementPtr Link::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
bool Link::EnableWind() const
{
  return this->dataPtr->enableWind;
}

/////////////////////////////////////////////////
void Link::SetEnableWind(const bool _enableWind)
{
  this->dataPtr->enableWind = _enableWind;
}

/////////////////////////////////////////////////
bool Link::Kinematic() const
{
  return this->dataPtr->kinematic;
}

/////////////////////////////////////////////////
void Link::SetKinematic(bool _kinematic)
{
  this->dataPtr->kinematic = _kinematic;
}

/////////////////////////////////////////////////
bool Link::AutoInertia() const
{
  return this->dataPtr->autoInertia;
}

/////////////////////////////////////////////////
void Link::SetAutoInertia(bool _autoInertia)
{
  this->dataPtr->autoInertia = _autoInertia;
}

/////////////////////////////////////////////////
bool Link::AutoInertiaSaved() const
{
  return this->dataPtr->autoInertiaSaved;
}

/////////////////////////////////////////////////
void Link::SetAutoInertiaSaved(bool _autoInertiaSaved)
{
  this->dataPtr->autoInertiaSaved = _autoInertiaSaved;
}

//////////////////////////////////////////////////
bool Link::AddCollision(const Collision &_collision)
{
  if (this->CollisionNameExists(_collision.Name()))
    return false;
  this->dataPtr->collisions.push_back(_collision);
  return true;
}

//////////////////////////////////////////////////
bool Link::AddVisual(const Visual &_visual)
{
  if (this->VisualNameExists(_visual.Name()))
    return false;
  this->dataPtr->visuals.push_back(_visual);
  return true;
}

//////////////////////////////////////////////////
bool Link::AddLight(const Light &_light)
{
  if (this->LightNameExists(_light.Name()))
    return false;
  this->dataPtr->lights.push_back(_light);
  return true;
}

//////////////////////////////////////////////////
bool Link::AddSensor(const Sensor &_sensor)
{
  if (this->SensorNameExists(_sensor.Name()))
    return false;
  this->dataPtr->sensors.push_back(_sensor);
  return true;
}

//////////////////////////////////////////////////
bool Link::AddParticleEmitter(const ParticleEmitter &_emitter)
{
  if (this->ParticleEmitterNameExists(_emitter.Name()))
    return false;
  this->dataPtr->emitters.push_back(_emitter);
  return true;
}

//////////////////////////////////////////////////
bool Link::AddProjector(const Projector &_projector)
{
  if (this->ProjectorNameExists(_projector.Name()))
    return false;
  this->dataPtr->projectors.push_back(_projector);
  return true;
}

//////////////////////////////////////////////////
void Link::ClearCollisions()
{
  this->dataPtr->collisions.clear();
}

//////////////////////////////////////////////////
void Link::ClearVisuals()
{
  this->dataPtr->visuals.clear();
}

//////////////////////////////////////////////////
void Link::ClearLights()
{
  this->dataPtr->lights.clear();
}

//////////////////////////////////////////////////
void Link::ClearSensors()
{
  this->dataPtr->sensors.clear();
}

//////////////////////////////////////////////////
void Link::ClearParticleEmitters()
{
  this->dataPtr->emitters.clear();
}

//////////////////////////////////////////////////
void Link::ClearProjectors()
{
  this->dataPtr->projectors.clear();
}

/////////////////////////////////////////////////
sdf::ElementPtr Link::ToElement() const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("link.sdf", elem);

  elem->GetAttribute("name")->Set(this->Name());

  // Set pose
  sdf::ElementPtr poseElem = elem->GetElement("pose");
  if (!this->dataPtr->poseRelativeTo.empty())
  {
    poseElem->GetAttribute("relative_to")->Set<std::string>(
        this->dataPtr->poseRelativeTo);
  }
  poseElem->Set<gz::math::Pose3d>(this->RawPose());

  // inertial
  sdf::ElementPtr inertialElem = elem->GetElement("inertial");
  inertialElem->GetElement("pose")->Set(
      this->dataPtr->inertial.Pose());
  const gz::math::MassMatrix3d &massMatrix =
    this->dataPtr->inertial.MassMatrix();
  inertialElem->GetElement("mass")->Set<double>(massMatrix.Mass());
  sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");
  inertiaElem->GetElement("ixx")->Set(massMatrix.Ixx());
  inertiaElem->GetElement("ixy")->Set(massMatrix.Ixy());
  inertiaElem->GetElement("ixz")->Set(massMatrix.Ixz());
  inertiaElem->GetElement("iyy")->Set(massMatrix.Iyy());
  inertiaElem->GetElement("iyz")->Set(massMatrix.Iyz());
  inertiaElem->GetElement("izz")->Set(massMatrix.Izz());

  if (this->dataPtr->density.has_value())
  {
    inertialElem->GetElement("density")->Set(*this->dataPtr->density);
  }

  if (this->dataPtr->inertial.FluidAddedMass().has_value())
  {
    auto addedMass = this->dataPtr->inertial.FluidAddedMass().value();
    auto addedMassElem = inertialElem->GetElement("fluid_added_mass");
    addedMassElem->GetElement("xx")->Set(addedMass(0, 0));
    addedMassElem->GetElement("xy")->Set(addedMass(0, 1));
    addedMassElem->GetElement("xz")->Set(addedMass(0, 2));
    addedMassElem->GetElement("xp")->Set(addedMass(0, 3));
    addedMassElem->GetElement("xq")->Set(addedMass(0, 4));
    addedMassElem->GetElement("xr")->Set(addedMass(0, 5));
    addedMassElem->GetElement("yy")->Set(addedMass(1, 1));
    addedMassElem->GetElement("yz")->Set(addedMass(1, 2));
    addedMassElem->GetElement("yp")->Set(addedMass(1, 3));
    addedMassElem->GetElement("yq")->Set(addedMass(1, 4));
    addedMassElem->GetElement("yr")->Set(addedMass(1, 5));
    addedMassElem->GetElement("zz")->Set(addedMass(2, 2));
    addedMassElem->GetElement("zp")->Set(addedMass(2, 3));
    addedMassElem->GetElement("zq")->Set(addedMass(2, 4));
    addedMassElem->GetElement("zr")->Set(addedMass(2, 5));
    addedMassElem->GetElement("pp")->Set(addedMass(3, 3));
    addedMassElem->GetElement("pq")->Set(addedMass(3, 4));
    addedMassElem->GetElement("pr")->Set(addedMass(3, 5));
    addedMassElem->GetElement("qq")->Set(addedMass(4, 4));
    addedMassElem->GetElement("qr")->Set(addedMass(4, 5));
    addedMassElem->GetElement("rr")->Set(addedMass(5, 5));
  }

  // wind mode
  elem->GetElement("enable_wind")->Set(this->EnableWind());

  // kinematic
  elem->GetElement("kinematic")->Set(this->Kinematic());

  // Collisions
  for (const sdf::Collision &collision : this->dataPtr->collisions)
  {
    elem->InsertElement(collision.ToElement(), true);
  }

  // Light
  for (const sdf::Light &light : this->dataPtr->lights)
  {
    elem->InsertElement(light.ToElement(), true);
  }

  // Particle emitters
  for (const sdf::ParticleEmitter &emitter : this->dataPtr->emitters)
  {
    elem->InsertElement(emitter.ToElement(), true);
  }

  // Projectors
  for (const sdf::Projector &projector : this->dataPtr->projectors)
  {
    elem->InsertElement(projector.ToElement(), true);
  }

  // Sensors
  for (const sdf::Sensor &sensor : this->dataPtr->sensors)
  {
    elem->InsertElement(sensor.ToElement(), true);
  }

  // Visuals
  for (const sdf::Visual &visual : this->dataPtr->visuals)
  {
    elem->InsertElement(visual.ToElement(), true);
  }

  return elem;
}
