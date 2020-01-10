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
#include "Utils.hh"

using namespace sdf;

class sdf::VisualPrivate
{
  /// \brief Default constructor
  public: VisualPrivate() = default;

  /// \brief Copy constructor
  /// \param[in] _visualPrivate Joint axis to move.
  public: explicit VisualPrivate(const VisualPrivate &_visualPrivate);

  // Delete copy assignment so it is not accidentally used
  public: VisualPrivate &operator=(const VisualPrivate &) = delete;

  /// \brief Name of the visual.
  public: std::string name = "";

  /// \brief Whether the visual casts shadows
  public: bool castShadows = true;

  /// \brief Pose of the visual object
  public: ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;

  /// \brief Frame of the pose.
  public: std::string poseRelativeTo = "";

  /// \brief The visual's a geometry.
  public: Geometry geom;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief Pointer to the visual's material properties.
  public: std::unique_ptr<Material> material;

  /// \brief Name of xml parent object.
  public: std::string xmlParentName;

  /// \brief Weak pointer to model's Pose Relative-To Graph.
  public: std::weak_ptr<const sdf::PoseRelativeToGraph> poseRelativeToGraph;
};

/////////////////////////////////////////////////
VisualPrivate::VisualPrivate(const VisualPrivate &_visualPrivate)
    : name(_visualPrivate.name),
      castShadows(_visualPrivate.castShadows),
      pose(_visualPrivate.pose),
      poseRelativeTo(_visualPrivate.poseRelativeTo),
      geom(_visualPrivate.geom),
      sdf(_visualPrivate.sdf)
{
  if (_visualPrivate.material)
  {
    this->material = std::make_unique<Material>(*(_visualPrivate.material));
  }
}

/////////////////////////////////////////////////
Visual::Visual()
  : dataPtr(new VisualPrivate)
{
}

/////////////////////////////////////////////////
Visual::Visual(const Visual &_visual)
  : dataPtr(new VisualPrivate(*_visual.dataPtr))
{
}

/////////////////////////////////////////////////
Visual &Visual::operator=(const Visual &_visual)
{
  if (!this->dataPtr)
  {
    this->dataPtr = new VisualPrivate(*_visual.dataPtr);
  }
  else
  {
    this->dataPtr = new(this->dataPtr) VisualPrivate(*_visual.dataPtr);
  }
  return *this;
}

/////////////////////////////////////////////////
Visual::Visual(Visual &&_visual) noexcept
{
  this->dataPtr = _visual.dataPtr;
  _visual.dataPtr = nullptr;
}

/////////////////////////////////////////////////
Visual &Visual::operator=(Visual &&_visual)
{
  std::swap(this->dataPtr, _visual.dataPtr);
  return *this;
}

/////////////////////////////////////////////////
Visual::~Visual()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
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

  if (_sdf->HasElement("material"))
  {
    this->dataPtr->material.reset(new sdf::Material());
    Errors err = this->dataPtr->material->Load(_sdf->GetElement("material"));
    errors.insert(errors.end(), err.begin(), err.end());
  }

  // Load the pose. Ignore the return value since the pose is optional.
  loadPose(_sdf, this->dataPtr->pose, this->dataPtr->poseRelativeTo);

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
void Visual::SetName(const std::string &_name) const
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
const ignition::math::Pose3d &Visual::Pose() const
{
  return this->RawPose();
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &Visual::RawPose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
const std::string &Visual::PoseFrame() const
{
  return this->PoseRelativeTo();
}

/////////////////////////////////////////////////
const std::string &Visual::PoseRelativeTo() const
{
  return this->dataPtr->poseRelativeTo;
}

/////////////////////////////////////////////////
void Visual::SetPose(const ignition::math::Pose3d &_pose)
{
  this->SetRawPose(_pose);
}

/////////////////////////////////////////////////
void Visual::SetRawPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
void Visual::SetPoseFrame(const std::string &_frame)
{
  this->SetPoseRelativeTo(_frame);
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
    std::weak_ptr<const PoseRelativeToGraph> _graph)
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
sdf::Material *Visual::Material() const
{
  return this->dataPtr->material.get();
}

/////////////////////////////////////////////////
void Visual::SetMaterial(const sdf::Material &_material)
{
  this->dataPtr->material.reset(new sdf::Material(_material));
}
