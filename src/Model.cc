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
#include "sdf/sdf_config.h"
#include "sdf/parser.hh"
#include "sdf/Model.hh"

/////////////////////////////////////////////////
bool sdf::loadName(sdf::ElementPtr _sdf, std::string &_name)
{
  // Read the name
  std::pair<std::string, bool> namePair = _sdf->Get<std::string>("name", "");

  _name = namePair.first;
  return namePair.second;
}

/////////////////////////////////////////////////
bool sdf::loadPose(sdf::ElementPtr _sdf, ignition::math::Pose3d &_pose,
              std::string &_frame)
{
  // Read the frame. An empty frame implies the parent frame.
  std::pair<std::string, bool> framePair = _sdf->Get<std::string>("frame", "");

  std::pair<ignition::math::Pose3d, bool> posePair =
    _sdf->Get<ignition::math::Pose3d>("", ignition::math::Pose3d::Zero);

  _pose = posePair.first;
  _frame = framePair.first;

  return true;
}

/////////////////////////////////////////////////
bool sdf::loadLights(sdf::ElementPtr _sdf,
                     std::map<std::string, Light> &_lights)
{
  bool result = true;

  // Read all the lights
  if (_sdf->HasElement("light"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("light");
    while (elem)
    {
      Light light;

      // Attempt to load the light
      if (light.Load(elem))
      {
        // Check that the light's name does not exist.
        if (_lights.find(light.Name()) != _lights.end())
        {
          std::cerr << "Light with name[" << light.Name() << "] already exists."
            << " Each light must have a unique name.\n";
          result = false;
        }
        _lights.insert(std::make_pair(light.Name(), std::move(light)));
      }
      elem = elem->GetNextElement("light");
    }
  }

  return result;
}

/////////////////////////////////////////////////
bool sdf::loadModels(sdf::ElementPtr _sdf,
                     std::map<std::string, Model> &_models)
{
  bool result = true;

  // Read all the models
  if (_sdf->HasElement("model"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("model");
    while (elem)
    {
      Model model;

      // Attempt to load the model
      if (model.Load(elem))
      {
        // Check that the model's name does not exist.
        if (_models.find(model.Name()) != _models.end())
        {
          std::cerr << "Model with name[" << model.Name() << "] already exists."
            << " Each model must have a unique name.\n";
          result = false;
        }
        _models.insert(std::make_pair(model.Name(), std::move(model)));
      }
      elem = elem->GetNextElement("model");
    }
  }

  return result;
}

/// \brief Private data for sdf::Root
class sdf::RootPrivate
{
  /// \brief Version string
  public: std::string version = "";

  /// \brief The worlds specified under the root SDF element
  public: std::map<std::string, World> worlds;

  /// \brief The models specified under the root SDF element
  public: std::map<std::string, Model> models;

  /// \brief The lights specified under the root SDF element
  public: std::map<std::string, Light> lights;
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
}

/////////////////////////////////////////////////
bool Root::Load(const std::string &_filename)
{
  // Read an SDF file, and store the result in sdfParsed.
  sdf::SDFPtr sdfParsed = sdf::readFile(_filename);

  if (sdfParsed)
    return this->Load(sdfParsed->Root());

  std::cerr << "Unable to read file[" << _filename << "]\n";
  return false;
}

/////////////////////////////////////////////////
bool Root::Load(const sdf::SDFPtr _sdf)
{
  return this->Load(_sdf->Root());
}

/////////////////////////////////////////////////
bool Root::Load(sdf::ElementPtr _sdf)
{
  bool result = true;

  // Get the SDF version
  std::pair<std::string, bool> versionPair =
    _sdf->Get<std::string>("version", SDF_VERSION);

  // Check that the version exists.
  if (!versionPair.second)
  {
    std::cerr << "SDF does not have a version.\n";
    result = false;
  }
  else
    this->dataPtr->version = versionPair.first;

  // Read all the worlds
  if (_sdf->HasElement("world"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("world");
    while (elem)
    {
      World world;

      // Attempt to load the world
      if (world.Load(elem))
      {
        // Check that the world's name does not exist.
        if (this->dataPtr->worlds.find(world.Name()) !=
            this->dataPtr->worlds.end())
        {
          std::cerr << "World with name[" << world.Name() << "] already exists."
            << " Each world must have a unique name. Skipping this world.\n";
          result = false;
        }

        this->dataPtr->worlds.insert(std::make_pair(world.Name(),
                                                    std::move(world)));
      }
      elem = elem->GetNextElement("world");
    }
  }

  // Read all the models
  result = result && loadModels(_sdf, this->dataPtr->models);

  // Read all the lights
  result = result && loadLights(_sdf, this->dataPtr->lights);

  return result;
}

/////////////////////////////////////////////////
std::string Root::Version() const
{
  return this->dataPtr->version;
}

/////////////////////////////////////////////////
void Root::Print(const std::string &_prefix) const
{
  std::cout << "SDF Version: " << this->Version() << "\n";

  for (auto const &world: this->dataPtr->worlds)
  {
    world.second.Print(_prefix + "  ");
  }

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
World::World()
  : dataPtr(new WorldPrivate)
{
}

/////////////////////////////////////////////////
World::~World()
{
  delete this->dataPtr;
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
std::string World::AudioDevice() const
{
  return this->dataPtr->audioDevice;
}

/////////////////////////////////////////////////
ignition::math::Vector3d World::WindLinearVelocity() const
{
  return this->dataPtr->windLinearVelocity;
}

/////////////////////////////////////////////////
bool Model::Load(sdf::ElementPtr _sdf)
{
  bool result = true;

  if (!loadName(_sdf, this->name))
  {
    std::cerr << "A model name is required, but is not set.\n";
    result = false;
  }

  if (!loadPose(_sdf, this->pose, this->frame))
  {
    std::cerr << "Unable to load the model[" << this->Name() << "]'s pose.\n";
    result = false;
  }

  this->isStatic = _sdf->Get<bool>("static", "false").first;
  this->selfCollide = _sdf->Get<bool>("self_collide", "false").first;
  this->autoDisable = _sdf->Get<bool>("allow_auto_disable", "true").first;
  this->enableWind = _sdf->Get<bool>("enable_wind", "false").first;

  // Read all the links
  if (_sdf->HasElement("link"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link");
    while (elem)
    {
      sdf::Link link;
      if (link.Load(elem))
        this->links.insert(std::make_pair(link.Name(), std::move(link)));
      else
         result = false;

      elem = elem->GetNextElement("link");
    }
  }

  // Read all the links
  if (_sdf->HasElement("joint"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("joint");
    while (elem)
    {
      sdf::Joint joint;
      if (joint.Load(elem))
        this->joints.insert(std::make_pair(joint.Name(), std::move(joint)));
      else
        result = false;
      elem = elem->GetNextElement("joint");
    }
  }

  // Read all the nested models
  result = result && loadModels(_sdf, this->models);

  return result;
}

/////////////////////////////////////////////////
void Model::Print(const std::string &_prefix) const
{
  std::cout << _prefix << " ## Model: " << this->name << "\n"
            << _prefix << "   * Pose:  " << this->pose << "\n"
            << _prefix << "   * Frame:  " << this->frame << "\n"
            << _prefix << "   * Static:  " << this->isStatic << "\n"
            << _prefix << "   * Enable wind:  " << this->enableWind << "\n"
            << _prefix << "   * Self collide:  " << this->selfCollide << "\n"
            << _prefix << "   * Auto disable:  " << this->autoDisable << "\n"
            << _prefix << "   * Link count:  " << this->links.size() << "\n"
            << _prefix << "   * Joint count: " << this->joints.size() << "\n"
            << _prefix << "   * Nested model count: "
            << this->models.size() << "\n";

  for (auto const &link: this->links)
  {
    link.second.Print(_prefix + "  ");
  }

  for (auto const &joint: this->joints)
  {
    joint.second.Print(_prefix + "  ");
  }

  for (auto const &model: this->models)
  {
    model.second.Print(_prefix + "  ");
  }
}

/////////////////////////////////////////////////
std::string Model::Name() const
{
  return this->name;
}

/////////////////////////////////////////////////
uint64_t Model::LinkCount() const
{
  return this->links.size();
}

/////////////////////////////////////////////////
const Link *Model::FindLink(const std::string &_name) const
{
  const auto iter = this->links.find(_name);
  if (iter != this->links.end())
    return &iter->second;
  return nullptr;
}

/////////////////////////////////////////////////
uint64_t Model::JointCount() const
{
  return this->joints.size();
}

/////////////////////////////////////////////////
const Joint *Model::FindJoint(const std::string &_name) const
{
  const auto iter = this->joints.find(_name);
  if (iter != this->joints.end())
    return &iter->second;
  return nullptr;
}

/////////////////////////////////////////////////
uint64_t Model::ModelCount() const
{
  return this->models.size();
}

/////////////////////////////////////////////////
const Model *Model::FindModel(const std::string &_name) const
{
  const auto iter = this->models.find(_name);
  if (iter != this->models.end())
    return &iter->second;
  return nullptr;
}

/////////////////////////////////////////////////
bool Joint::Load(sdf::ElementPtr _sdf)
{
  bool result = true;

  if (!loadName(_sdf, this->name))
  {
    std::cerr << "A link name is required, but is not set.\n";
    result = false;
  }

  return true;
}

/////////////////////////////////////////////////
void Joint::Print(const std::string &_prefix) const
{
  std::cout << _prefix << " Joint: " << this->name << std::endl;
}

/////////////////////////////////////////////////
std::string Joint::Name() const
{
  return this->name;
}

/////////////////////////////////////////////////
bool Link::Load(sdf::ElementPtr _sdf)
{
  bool result = true;

  if (!loadName(_sdf, this->name))
  {
    std::cerr << "A link name is required, but is not set.\n";
    result = false;
  }

  return true;
}

/////////////////////////////////////////////////
void Link::Print(const std::string &_prefix) const
{
  std::cout << _prefix << " Link: " << this->name << std::endl;
}

/////////////////////////////////////////////////
std::string Link::Name() const
{
  return this->name;
}

/////////////////////////////////////////////////
bool Light::Load(sdf::ElementPtr _sdf)
{
  bool result = true;

  if (!loadName(_sdf, this->name))
  {
    std::cerr << "A light name is required, but is not set.\n";
    result = false;
  }

  if (!loadPose(_sdf, this->pose, this->frame))
  {
    std::cerr << "Unable to load the light[" << this->Name() << "]'s pose.\n";
    result = false;
  }

  // Read the type
  std::pair<std::string, bool> typePair = _sdf->Get<std::string>("type", "");
  if (!typePair.second)
  {
    std::cerr << "Light[" << this->Name() << "] has no type\n";
    result = false;
  }
  else
  {
    this->type = typePair.first;
  }

  return result;
}

/////////////////////////////////////////////////
std::string Light::Name() const
{
  return this->name;
}

/////////////////////////////////////////////////
std::string Light::Type() const
{
  return this->type;
}

/////////////////////////////////////////////////
bool Light::CastShadows() const
{
  return this->castShadows;
}

/////////////////////////////////////////////////
ignition::math::Color Light::Diffuse() const
{
  return this->diffuse;
}

/////////////////////////////////////////////////
ignition::math::Color Light::Specular() const
{
  return this->specular;
}

/////////////////////////////////////////////////
void Light::Print(const std::string &_prefix) const
{
  std::cout << _prefix << "# Light: " << this->Name() << "\n"
            << _prefix << "  * Type: " << this->Type() << "\n"
            << _prefix << "  * Pose:  " << this->pose << "\n"
            << _prefix << "  * Frame:  " << this->frame << "\n"
            << _prefix << "  * Cast shadows: " << this->CastShadows() << "\n"
            << _prefix << "  * Diffuse: " << this->Diffuse() << "\n"
            << _prefix << "  * Specular: " << this->Specular() << "\n";
}

/////////////////////////////////////////////////
bool SphericalCoordinates::Load(ElementPtr _sdf)
{
  if (_sdf->GetName() != "spherical_coordinates")
  {
    std::cerr << "Provided element pointer is not a <spherical_coordinates> "
      << "element.\n";
    return false;
  }

  bool result = true;

  // Get the surface model
  std::pair<std::string, bool> surfaceModelPair =
    _sdf->Get<std::string>("surface_model", this->surfaceModel);
  if (!surfaceModelPair.second)
  {
    std::cerr << "Missing <surface_model> element, "
      << "child of <spherical_coordinates>. Using default value of "
      << this->surfaceModel << std::endl;
    result = false;
  }
  this->surfaceModel = surfaceModelPair.first;

  // Get the world frame orientation
  std::pair<std::string, bool> worldOrientPair =
    _sdf->Get<std::string>("world_frame_orientation",
        this->worldFrameOrientation);
  if (!worldOrientPair.second)
  {
    std::cerr << "Missing <world_frame_orientation> element, "
      << "child of <spherical_coordinates>. Using default value of "
      << this->worldFrameOrientation << std::endl;
    result = false;
  }
  this->worldFrameOrientation = worldOrientPair.first;

  // Get the latitude
  std::pair<double, bool> latPair = _sdf->Get<double>("latitude_deg",
        this->latitude);
  if (!latPair.second)
  {
    std::cerr << "Missing <latitude> element, "
      << "child of <spherical_coordinates>. Using default value of "
      << this->latitude << std::endl;
    result = false;
  }
  this->latitude = latPair.first;

  // Get the longitude
  std::pair<double, bool> lonPair = _sdf->Get<double>("longitude_deg",
        this->longitude);
  if (!lonPair.second)
  {
    std::cerr << "Missing <longitude> element, "
      << "child of <spherical_coordinates>. Using default value of "
      << this->longitude << std::endl;
    result = false;
  }
  this->longitude = lonPair.first;

  // Get the elevation
  std::pair<double, bool> elevationPair = _sdf->Get<double>("elevation",
        this->elevation);
  if (!elevationPair.second)
  {
    std::cerr << "Missing <elevation> element, "
      << "child of <spherical_coordinates>. Using default value of "
      << this->elevation << std::endl;
    result = false;
  }
  this->elevation = elevationPair.first;

  // Get the heading
  std::pair<double, bool> headingPair = _sdf->Get<double>("heading",
        this->heading);
  if (!headingPair.second)
  {
    std::cerr << "Missing <heading> element, "
      << "child of <spherical_coordinates>. Using default value of "
      << this->heading << std::endl;
    result = false;
  }
  this->heading = headingPair.first;

  return result;
}

/////////////////////////////////////////////////
std::string SphericalCoordinates::SurfaceModel()
{
  return this->surfaceModel;
}

/////////////////////////////////////////////////
std::string SphericalCoordinates::WorldFrameOrientation()
{
  return this->worldFrameOrientation;
}

/////////////////////////////////////////////////
double SphericalCoordinates::Latitude()
{
  return this->latitude;
}

/////////////////////////////////////////////////
double SphericalCoordinates::Longitude()
{
  return this->longitude;
}

/////////////////////////////////////////////////
double SphericalCoordinates::Elevation()
{
  return this->elevation;
}

/////////////////////////////////////////////////
double SphericalCoordinates::Heading()
{
  return this->heading;
}
