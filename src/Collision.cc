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
#include <string>
#include <ignition/math/Matrix4.hh>
#include <ignition/math/Pose3.hh>
#include "sdf/Collision.hh"
#include "sdf/Geometry.hh"
#include "sdf/Error.hh"
#include "sdf/Types.hh"
#include "Utils.hh"

using namespace sdf;
using namespace ignition::math;

class sdf::CollisionPrivate
{
  /// \brief Frame of the pose.
  public: std::string poseFrame = "";

  /// \brief The collisions's a geometry.
  public: Geometry geom;

  /// \brief Pointer to the frame graph.
  public: std::shared_ptr<FrameGraph> frameGraph = nullptr;

  /// \brief Id of the frame for this object
  public: ignition::math::graph::VertexId frameVertexId;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Collision::Collision()
  : dataPtr(new CollisionPrivate)
{
  // Create the frame graph for the model, and add a node for the model.
  this->dataPtr->frameGraph.reset(new FrameGraph);
  this->dataPtr->frameVertexId = this->dataPtr->frameGraph->AddVertex(
      "", Matrix4d::Identity).Id();
}

/////////////////////////////////////////////////
Collision::Collision(Collision &&_collision)
{
  this->dataPtr = _collision.dataPtr;
  _collision.dataPtr = nullptr;
}

/////////////////////////////////////////////////
Collision::~Collision()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
Errors Collision::Load(ElementPtr _sdf, std::shared_ptr<FrameGraph> _frameGraph)
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
  std::string collisionName;
  if (!loadName(_sdf, collisionName))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A collision name is required, but the name is not set."});
  }

  // Load the pose. Ignore the return value since the pose is optional.
  Pose3d pose;
  loadPose(_sdf, pose, this->dataPtr->poseFrame);

  // Use the SDF parent as the pose frame if the poseFrame attribute is
  // empty.
  if (this->dataPtr->poseFrame.empty() && _sdf->GetParent())
    this->dataPtr->poseFrame = _sdf->GetParent()->Get<std::string>("name");

  if (_frameGraph)
  {
    this->dataPtr->frameVertexId =
      _frameGraph->AddVertex(collisionName, Matrix4d(pose)).Id();

    // Get the parent vertex based on this link's pose frame name.
    const ignition::math::graph::VertexRef_M<ignition::math::Matrix4d>
      parentVertices = _frameGraph->Vertices(this->dataPtr->poseFrame);

    // Connect the parent to the child
    _frameGraph->AddEdge({parentVertices.begin()->first,
        this->dataPtr->frameVertexId}, -1);

    // Connect the child to the parent
    _frameGraph->AddEdge({this->dataPtr->frameVertexId,
        parentVertices.begin()->first}, 1);

    this->dataPtr->frameGraph = _frameGraph;
  }

  // Load the geometry
  Errors geomErr = this->dataPtr->geom.Load(_sdf->GetElement("geometry"));
  errors.insert(errors.end(), geomErr.begin(), geomErr.end());

  return errors;
}

/////////////////////////////////////////////////
std::string Collision::Name() const
{
  return this->dataPtr->frameGraph->VertexFromId(
      this->dataPtr->frameVertexId).Name();
}

/////////////////////////////////////////////////
void Collision::SetName(const std::string &_name) const
{
  // Store the name in the frame graph
  this->dataPtr->frameGraph->VertexFromId(
      this->dataPtr->frameVertexId).SetName(_name);
}

/////////////////////////////////////////////////
const Geometry *Collision::Geom() const
{
  return &this->dataPtr->geom;
}

/////////////////////////////////////////////////
std::optional<Pose3d> Collision::Pose(const std::string &_frame) const
{
  return poseInFrame(
      this->Name(),
      _frame.empty() ? this->PoseFrame() : _frame,
      *this->dataPtr->frameGraph);
}

/////////////////////////////////////////////////
const std::string &Collision::PoseFrame() const
{
  return this->dataPtr->poseFrame;
}

/////////////////////////////////////////////////
void Collision::SetPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->frameGraph->VertexFromId(
      this->dataPtr->frameVertexId).Data() = ignition::math::Matrix4d(_pose);
}

/////////////////////////////////////////////////
bool Collision::SetPoseFrame(const std::string &_frame)
{
  if (_frame.empty())
    return false;

  this->dataPtr->poseFrame = _frame;
  return true;
}

/////////////////////////////////////////////////
sdf::ElementPtr Collision::Element() const
{
  return this->dataPtr->sdf;
}
