/*
 * Copyright 2021 Open Source Robotics Foundation
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

#include "sdf/InterfaceModel.hh"
#include "FrameSemantics.hh"
#include "ScopedGraph.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
class InterfaceModel::Implementation
{
  /// \brief Name of this interface model.
  public: std::string name;

  /// \brief Reposturing callback function.
  public: sdf::RepostureFunction repostureFunction;

  /// \brief Whether this model is static.
  public: bool isStatic;

  /// \brief Name of Canonical link. This is the resolved name of the canonical
  /// link, therefore, it cannot be an empty string.
  public: std::string canonicalLinkName;

  /// \brief Model frame pose relative to the parent frame.
  public: ignition::math::Pose3d poseInParentFrame;

  /// \brief Collection of child interface models
  public: std::vector<sdf::InterfaceModelConstPtr> nestedModels;

  /// \brief Collection of child interface frames
  public: std::vector<sdf::InterfaceFrame> frames;

  /// \brief Collection of child interface joints
  public: std::vector<sdf::InterfaceJoint> joints;

  /// \brief Collection of child interface links
  public: std::vector<sdf::InterfaceLink> links;
};

InterfaceModel::InterfaceModel(const std::string &_name,
    const sdf::RepostureFunction &_repostureFunction,
    bool _static,
    const std::string &_canonicalLinkName,
    const ignition::math::Pose3d &_poseInParentFrame)
    : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
  this->dataPtr->name = _name;
  this->dataPtr->repostureFunction = _repostureFunction;
  this->dataPtr->isStatic = _static;
  this->dataPtr->canonicalLinkName = _canonicalLinkName;
  this->dataPtr->poseInParentFrame = _poseInParentFrame;
}

/////////////////////////////////////////////////
const std::string &InterfaceModel::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
bool InterfaceModel::Static() const
{
  return this->dataPtr->isStatic;
}

/////////////////////////////////////////////////
const std::string &InterfaceModel::CanonicalLinkName() const
{
  return this->dataPtr->canonicalLinkName;
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &
InterfaceModel::ModelFramePoseInParentFrame() const
{
  return this->dataPtr->poseInParentFrame;
}

///////////////////////////////////////////////
void InterfaceModel::AddNestedModel(sdf::InterfaceModelConstPtr _nestedmodel)
{
  this->dataPtr->nestedModels.push_back(std::move(_nestedmodel));
}

/////////////////////////////////////////////////
const std::vector<sdf::InterfaceModelConstPtr> &
InterfaceModel::NestedModels() const
{
  return this->dataPtr->nestedModels;
}

/////////////////////////////////////////////////
void InterfaceModel::AddFrame(sdf::InterfaceFrame _frame)
{
  this->dataPtr->frames.push_back(std::move(_frame));
}

/////////////////////////////////////////////////
const std::vector<sdf::InterfaceFrame> &InterfaceModel::Frames() const
{
  return this->dataPtr->frames;
}

/////////////////////////////////////////////////
void InterfaceModel::AddJoint(sdf::InterfaceJoint _joint)
{
  this->dataPtr->joints.push_back(std::move(_joint));
}

/////////////////////////////////////////////////
const std::vector<sdf::InterfaceJoint> &InterfaceModel::Joints() const
{
  return this->dataPtr->joints;
}

/////////////////////////////////////////////////
void InterfaceModel::AddLink(sdf::InterfaceLink _link)
{
  this->dataPtr->links.push_back(std::move(_link));
}

/////////////////////////////////////////////////
const std::vector<sdf::InterfaceLink> &InterfaceModel::Links() const
{
  return this->dataPtr->links;
}

/////////////////////////////////////////////////
void InterfaceModel::InvokeRespostureFunction(
    sdf::ScopedGraph<PoseRelativeToGraph> _graph) const
{
  if (this->dataPtr->repostureFunction)
  {
    this->dataPtr->repostureFunction(
        sdf::InterfaceModelPoseGraph(this->dataPtr->name, _graph));
  }

  for (const auto &nestedIfaceModel : this->dataPtr->nestedModels)
  {
    nestedIfaceModel->InvokeRespostureFunction(
        _graph.ChildModelScope(this->Name()));
  }
}
}
}
