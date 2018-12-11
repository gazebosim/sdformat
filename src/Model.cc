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
#include <vector>
#include <ignition/math/graph/Graph.hh>
#include <ignition/math/Matrix4.hh>
#include <ignition/math/Pose3.hh>
#include "sdf/Error.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Types.hh"
#include "Utils.hh"

using namespace sdf;
using namespace ignition::math;

class sdf::ModelPrivate
{
  /// \brief True if this model is specified as static, false otherwise.
  public: bool isStatic = false;

  /// \brief True if this model should self-collide, false otherwise.
  public: bool selfCollide = false;

  /// \brief True if this model is allowed to conserve processing power by not
  /// updating when it's at rest.
  public: bool allowAutoDisable = true;

  /// \brief True if this model should be subject to wind, false otherwise.
  public: bool enableWind = false;

  /// \brief Frame of the pose.
  public: std::string poseFrame = "";

  /// \brief The links specified in this model.
  public: std::vector<Link> links;

  /// \brief The joints specified in this model.
  public: std::vector<Joint> joints;

  /// \brief Pointer to the frame graph.
  public: std::shared_ptr<FrameGraph> frameGraph = nullptr;

  /// \brief Id of the frame for this object
  public: ignition::math::graph::VertexId frameVertexId;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Model::Model()
  : dataPtr(new ModelPrivate)
{
  // Create the frame graph for the model, and add a node for the model.
  this->dataPtr->frameGraph.reset(new FrameGraph);
  this->dataPtr->frameVertexId = this->dataPtr->frameGraph->AddVertex(
      "", Matrix4d::Identity).Id();
}

/////////////////////////////////////////////////
Model::Model(Model &&_model)
{
  this->dataPtr = _model.dataPtr;
  _model.dataPtr = nullptr;
}

/////////////////////////////////////////////////
Model::~Model()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
Errors Model::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <model>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "model")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Model, but the provided SDF element is not a "
        "<model>."});
    return errors;
  }

  std::string modelName;

  // Read the models's name
  if (!loadName(_sdf, modelName))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A model name is required, but the name is not set."});
  }

  this->dataPtr->isStatic = _sdf->Get<bool>("static", false).first;

  this->dataPtr->selfCollide = _sdf->Get<bool>("self_collide", false).first;

  this->dataPtr->allowAutoDisable =
    _sdf->Get<bool>("allow_auto_disable", true).first;

  this->dataPtr->enableWind = _sdf->Get<bool>("enable_wind", false).first;

  // Load the pose, and add it to the frame graph.
  std::string frame;
  Pose3d pose;
  loadPose(_sdf, pose, frame);
  this->dataPtr->frameGraph.reset(new FrameGraph);
  this->dataPtr->frameVertexId = this->dataPtr->frameGraph->AddVertex(
      modelName, Matrix4d(pose)).Id();

  // Load any additional frames
  sdf::ElementPtr elem = _sdf->GetElement("frame");
  std::vector<std::pair<graph::Vertex<Matrix4d>, std::string>> edgesToAdd;
  while (elem)
  {
    // Get the pose data.
    std::string frameName, poseFrame;
    Pose3d poseValue;
    loadPose(elem, poseValue, poseFrame);

    // Get the name of the frame.
    frameName = elem->Get<std::string>("name", "").first;

    // This ugly if statement handles a bad aspect of this library. It will
    // create an default element if one doesn't exist. In this case, an
    // empty <frame> element will be created if it's not present in the SDF
    // file.
    if (frameName.empty() && poseFrame.empty() && poseValue == Pose3d::Zero)
    {
      elem = elem->GetNextElement("frame");
      continue;
    }

    // Make sure the frame name is not empty
    if (frameName.empty())
    {
      errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
          "A frame name is required, but the name is not set."});
    }
    else
    {
      // Use the model frame if the pose frame is empty, per the spec.
      if (!poseFrame.empty())
        poseFrame = this->Name();

      // Create the vertex.
      graph::Vertex<Matrix4d> &vert =
        this->dataPtr->frameGraph->AddVertex(frameName, Matrix4d(poseValue));

      // Store the edge to add. Create the edges later, because the poseFrame
      // may refer to a frame that has not been parsed yet.
      edgesToAdd.push_back(std::make_pair(vert, poseFrame));
    }

    // Get the next frame, if any
    elem = elem->GetNextElement("frame");
  }

  // Load all the links.
  Errors linkLoadErrors = loadUniqueRepeated<Link>(_sdf, "link",
    this->dataPtr->links, this->dataPtr->frameGraph);
  errors.insert(errors.end(), linkLoadErrors.begin(), linkLoadErrors.end());

  // Load all the joints.
  Errors jointLoadErrors = loadUniqueRepeated<Joint>(_sdf, "joint",
    this->dataPtr->joints, this->dataPtr->frameGraph);
  errors.insert(errors.end(), jointLoadErrors.begin(), jointLoadErrors.end());

  // Create edges in the frame graph.
  for (const std::pair<graph::Vertex<Matrix4d>, std::string> &edge : edgesToAdd)
  {
    const graph::VertexRef_M<Matrix4d> parentVertices =
      this->dataPtr->frameGraph->Vertices(edge.second);
    // Make sure a parent vertex was found.
    if (parentVertices.empty())
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "A frame named[" + edge.first.Name()
          + "] has an unknown pose frame of [" + edge.second + "]"});
      continue;
    }

    // Make sure only one parent vertex was found.
    if (parentVertices.size() > 1)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "A frame named[" + edge.first.Name()
          + "] has a pose frame of [" + edge.second
          + "] that resolves to multiple frames."});
      continue;
    }

    // Create the edges.
    this->dataPtr->frameGraph->AddEdge(
        {parentVertices.begin()->first, edge.first.Id()}, -1);
    this->dataPtr->frameGraph->AddEdge(
        {edge.first.Id(), parentVertices.begin()->first}, 1);
  }

  return errors;
}

/////////////////////////////////////////////////
std::string Model::Name() const
{
  return this->dataPtr->frameGraph->VertexFromId(
      this->dataPtr->frameVertexId).Name();
}

/////////////////////////////////////////////////
void Model::SetName(const std::string &_name)
{
  // Store the name in the frame graph
  this->dataPtr->frameGraph->VertexFromId(
      this->dataPtr->frameVertexId).SetName(_name);
}

/////////////////////////////////////////////////
bool Model::Static() const
{
  return this->dataPtr->isStatic;
}

/////////////////////////////////////////////////
void Model::SetStatic(const bool _static)
{
  this->dataPtr->isStatic = _static;
}

/////////////////////////////////////////////////
bool Model::SelfCollide() const
{
  return this->dataPtr->selfCollide;
}

/////////////////////////////////////////////////
void Model::SetSelfCollide(const bool _selfCollide)
{
  this->dataPtr->selfCollide = _selfCollide;
}

/////////////////////////////////////////////////
bool Model::AllowAutoDisable() const
{
  return this->dataPtr->allowAutoDisable;
}

/////////////////////////////////////////////////
void Model::SetAllowAutoDisable(const bool _allowAutoDisable)
{
  this->dataPtr->allowAutoDisable = _allowAutoDisable;
}

/////////////////////////////////////////////////
bool Model::EnableWind() const
{
  return this->dataPtr->enableWind;
}

/////////////////////////////////////////////////
void Model::SetEnableWind(const bool _enableWind)
{
  this->dataPtr->enableWind =_enableWind;
}

/////////////////////////////////////////////////
uint64_t Model::LinkCount() const
{
  return this->dataPtr->links.size();
}

/////////////////////////////////////////////////
const Link *Model::LinkByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->links.size())
    return &this->dataPtr->links[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Model::LinkNameExists(const std::string &_name) const
{
  for (auto const &l : this->dataPtr->links)
  {
    if (l.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
uint64_t Model::JointCount() const
{
  return this->dataPtr->joints.size();
}

/////////////////////////////////////////////////
const Joint *Model::JointByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->joints.size())
    return &this->dataPtr->joints[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Model::JointNameExists(const std::string &_name) const
{
  for (auto const &j : this->dataPtr->joints)
  {
    if (j.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
const Joint *Model::JointByName(const std::string &_name) const
{
  for (auto const &j : this->dataPtr->joints)
  {
    if (j.Name() == _name)
    {
      return &j;
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
Pose3d Model::Pose(const std::string &_frame) const
{
  return poseInFrame(
      this->Name(),
      _frame.empty() ? this->PoseFrame() : _frame,
      *this->dataPtr->frameGraph);
}

/////////////////////////////////////////////////
const std::string &Model::PoseFrame() const
{
  return this->dataPtr->poseFrame;
}

/////////////////////////////////////////////////
void Model::SetPose(const Pose3d &_pose)
{
  // Store the pose data in the frame graph
  this->dataPtr->frameGraph->VertexFromId(
      this->dataPtr->frameVertexId).Data() = ignition::math::Matrix4d(_pose);
}

/////////////////////////////////////////////////
bool Model::SetPoseFrame(const std::string &_frame)
{
  if (_frame.empty())
    return false;

  this->dataPtr->poseFrame = _frame;
  return true;
}

/////////////////////////////////////////////////
const Link *Model::LinkByName(const std::string &_name) const
{
  for (auto const &l : this->dataPtr->links)
  {
    if (l.Name() == _name)
    {
      return &l;
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
sdf::ElementPtr Model::Element() const
{
  return this->dataPtr->sdf;
}
