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
#include "sdf/Collision.hh"
#include "sdf/Error.hh"
#include "sdf/Geometry.hh"
#include "sdf/Surface.hh"
#include "sdf/Types.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::CollisionPrivate
{
  /// \brief Name of the collision.
  public: std::string name = "";

  /// \brief Pose of the collision object
  public: ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;

  /// \brief Frame of the pose.
  public: std::string poseRelativeTo = "";

  /// \brief The collision's geometry.
  public: Geometry geom;

  /// \brief The collision's surface parameters.
  public: Surface surface;

  /// \brief The SDF element pointer used during load.
  public: ElementPtr sdf;

  /// \brief Name of xml parent object.
  public: std::string xmlParentName;

  /// \brief Weak pointer to model's Pose Relative-To Graph.
  public: std::weak_ptr<const PoseRelativeToGraph> poseRelativeToGraph;
};

/////////////////////////////////////////////////
Collision::Collision()
  : dataPtr(new CollisionPrivate)
{
}

/////////////////////////////////////////////////
Collision::~Collision()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
Collision::Collision(const Collision &_collision)
  : dataPtr(new CollisionPrivate(*_collision.dataPtr))
{
}

/////////////////////////////////////////////////
Collision::Collision(Collision &&_collision) noexcept
  : dataPtr(std::exchange(_collision.dataPtr, nullptr))
{
}

/////////////////////////////////////////////////
Collision &Collision::operator=(const Collision &_collision)
{
  return *this = Collision(_collision);
}

/////////////////////////////////////////////////
Collision &Collision::operator=(Collision &&_collision)
{
  std::swap(this->dataPtr, _collision.dataPtr);
  return *this;
}

/////////////////////////////////////////////////
Errors Collision::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <collision>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "collision")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Collision, but the provided SDF element is not a "
        "<collision>."});
    return errors;
  }

  // Read the collisions's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A collision name is required, but the name is not set."});
  }

  // Check that the collision's name is valid
  if (isReservedName(this->dataPtr->name))
  {
    errors.push_back({ErrorCode::RESERVED_NAME,
                     "The supplied collision name [" + this->dataPtr->name +
                     "] is reserved."});
  }

  // Load the pose. Ignore the return value since the pose is optional.
  loadPose(_sdf, this->dataPtr->pose, this->dataPtr->poseRelativeTo);

  // Load the geometry
  Errors geomErr = this->dataPtr->geom.Load(_sdf->GetElement("geometry"));
  errors.insert(errors.end(), geomErr.begin(), geomErr.end());

  // Load the surface parameters if they are given
  if (_sdf->HasElement("surface"))
  {
    this->dataPtr->surface.Load(_sdf->GetElement("surface"));
  }

  return errors;
}

/////////////////////////////////////////////////
std::string Collision::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Collision::SetName(const std::string &_name) const
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
const Geometry *Collision::Geom() const
{
  return &this->dataPtr->geom;
}

/////////////////////////////////////////////////
void Collision::SetGeom(const Geometry &_geom)
{
  this->dataPtr->geom = _geom;
}

/////////////////////////////////////////////////
Surface *Collision::Surface() const
{
  return &this->dataPtr->surface;
}

/////////////////////////////////////////////////
void Collision::SetSurface(const sdf::Surface &_surface)
{
  this->dataPtr->surface = _surface;
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &Collision::Pose() const
{
  return this->RawPose();
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &Collision::RawPose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
const std::string &Collision::PoseFrame() const
{
  return this->PoseRelativeTo();
}

/////////////////////////////////////////////////
const std::string &Collision::PoseRelativeTo() const
{
  return this->dataPtr->poseRelativeTo;
}

/////////////////////////////////////////////////
void Collision::SetPose(const ignition::math::Pose3d &_pose)
{
  this->SetRawPose(_pose);
}

/////////////////////////////////////////////////
void Collision::SetRawPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
void Collision::SetPoseFrame(const std::string &_frame)
{
  this->SetPoseRelativeTo(_frame);
}

/////////////////////////////////////////////////
void Collision::SetPoseRelativeTo(const std::string &_frame)
{
  this->dataPtr->poseRelativeTo = _frame;
}

/////////////////////////////////////////////////
void Collision::SetXmlParentName(const std::string &_xmlParentName)
{
  this->dataPtr->xmlParentName = _xmlParentName;
}

/////////////////////////////////////////////////
void Collision::SetPoseRelativeToGraph(
    std::weak_ptr<const PoseRelativeToGraph> _graph)
{
  this->dataPtr->poseRelativeToGraph = _graph;
}

/////////////////////////////////////////////////
SemanticPose Collision::SemanticPose() const
{
  return sdf::SemanticPose(
      this->dataPtr->pose,
      this->dataPtr->poseRelativeTo,
      this->dataPtr->xmlParentName,
      this->dataPtr->poseRelativeToGraph);
}

/////////////////////////////////////////////////
ElementPtr Collision::Element() const
{
  return this->dataPtr->sdf;
}
