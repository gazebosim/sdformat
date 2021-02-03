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
#include <ignition/math/Pose3.hh>
#include "sdf/Error.hh"
#include "sdf/Types.hh"
#include "sdf/Visual.hh"
#include "sdf/Geometry.hh"
#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::Visual::Implementation
{
  /// \brief Name of the visual.
  public: std::string name = "";

  /// \brief Whether the visual casts shadows
  public: bool castShadows = true;

  /// \brief Transparency value between 0 and 1
  public: float transparency  = 0.0;

  /// \brief Pose of the visual object
  public: ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;

  /// \brief Frame of the pose.
  public: std::string poseRelativeTo = "";

  /// \brief The visual's a geometry.
  public: Geometry geom;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief The visual's material properties.
  public: std::optional<sdf::Material> material;

  /// \brief Name of xml parent object.
  public: std::string xmlParentName;

  /// \brief Scoped Pose Relative-To graph at the parent model scope.
  public: sdf::ScopedGraph<sdf::PoseRelativeToGraph> poseRelativeToGraph;

  /// \brief Visibility flags of a visual. Defaults to 0xFFFFFFFF
  public: uint32_t visibilityFlags = 4294967295u;
};

/////////////////////////////////////////////////
Visual::Visual()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Visual::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <visual>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "visual")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Visual, but the provided SDF element is not a "
        "<visual>."});
    return errors;
  }

  // Read the visuals's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A visual name is required, but the name is not set."});
  }

  // Check that the visual's name is valid
  if (isReservedName(this->dataPtr->name))
  {
    errors.push_back({ErrorCode::RESERVED_NAME,
                     "The supplied visual name [" + this->dataPtr->name +
                     "] is reserved."});
  }

  // load cast shadows
  if (_sdf->HasElement("cast_shadows"))
  {
    this->dataPtr->castShadows = _sdf->Get<bool>("cast_shadows",
        this->dataPtr->castShadows).first;
  }

  // load transparency
  if (_sdf->HasElement("transparency"))
  {
    this->dataPtr->transparency = _sdf->Get<float>("transparency");
  }

  if (_sdf->HasElement("material"))
  {
    this->dataPtr->material.emplace();
    Errors err = this->dataPtr->material->Load(_sdf->GetElement("material"));
    errors.insert(errors.end(), err.begin(), err.end());
  }

  // Load the pose. Ignore the return value since the pose is optional.
  loadPose(_sdf, this->dataPtr->pose, this->dataPtr->poseRelativeTo);


  // load visibility flags
  if (_sdf->HasElement("visibility_flags"))
  {
    this->dataPtr->visibilityFlags = _sdf->Get<uint32_t>("visibility_flags",
        this->dataPtr->visibilityFlags).first;
  }

  // Load the geometry
  Errors geomErr = this->dataPtr->geom.Load(_sdf->GetElement("geometry"));
  errors.insert(errors.end(), geomErr.begin(), geomErr.end());

  return errors;
}

/////////////////////////////////////////////////
std::string Visual::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Visual::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
bool Visual::CastShadows() const
{
  return this->dataPtr->castShadows;
}

/////////////////////////////////////////////////
void Visual::SetCastShadows(bool _castShadows)
{
  this->dataPtr->castShadows = _castShadows;
}

/////////////////////////////////////////////////
float Visual::Transparency() const
{
  return this->dataPtr->transparency;
}

/////////////////////////////////////////////////
void Visual::SetTransparency(float _transparency)
{
  this->dataPtr->transparency = _transparency;
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &Visual::RawPose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
const std::string &Visual::PoseRelativeTo() const
{
  return this->dataPtr->poseRelativeTo;
}

/////////////////////////////////////////////////
void Visual::SetRawPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
void Visual::SetPoseRelativeTo(const std::string &_frame)
{
  this->dataPtr->poseRelativeTo = _frame;
}

/////////////////////////////////////////////////
const Geometry *Visual::Geom() const
{
  return &this->dataPtr->geom;
}

/////////////////////////////////////////////////
void Visual::SetGeom(const Geometry &_geom)
{
  this->dataPtr->geom = _geom;
}

/////////////////////////////////////////////////
void Visual::SetXmlParentName(const std::string &_xmlParentName)
{
  this->dataPtr->xmlParentName = _xmlParentName;
}

/////////////////////////////////////////////////
void Visual::SetPoseRelativeToGraph(
    sdf::ScopedGraph<PoseRelativeToGraph> _graph)
{
  this->dataPtr->poseRelativeToGraph = _graph;
}

/////////////////////////////////////////////////
sdf::SemanticPose Visual::SemanticPose() const
{
  return sdf::SemanticPose(
      this->dataPtr->pose,
      this->dataPtr->poseRelativeTo,
      this->dataPtr->xmlParentName,
      this->dataPtr->poseRelativeToGraph);
}

/////////////////////////////////////////////////
sdf::ElementPtr Visual::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
const sdf::Material *Visual::Material() const
{
  return optionalToPointer(this->dataPtr->material);
}

/////////////////////////////////////////////////
void Visual::SetMaterial(const sdf::Material &_material)
{
  this->dataPtr->material = _material;
}

/////////////////////////////////////////////////
uint32_t Visual::VisibilityFlags() const
{
  return this->dataPtr->visibilityFlags;
}

/////////////////////////////////////////////////
void Visual::SetVisibilityFlags(uint32_t _flags)
{
  this->dataPtr->visibilityFlags = _flags;
}
