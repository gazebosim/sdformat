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
#include <ignition/math/Pose3.hh>
#include "sdf/Assert.hh"
#include "sdf/SemanticPose.hh"
#include "sdf/Error.hh"
#include "sdf/Types.hh"
#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "Utils.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {
/// \internal
/// \brief Private data for the SemanticPose class.
class SemanticPose::Implementation
{
  public: std::string name = "";

  /// \brief Raw pose of the SemanticPose object.
  public: ignition::math::Pose3d rawPose = ignition::math::Pose3d::Zero;

  /// \brief Name of the relative-to frame.
  public: std::string relativeTo = "";

  /// \brief Name of the default frame to resolve to.
  public: std::string defaultResolveTo = "";

  /// \brief Scoped Pose Relative-To graph at the parent model or world scope.
  /// TODO (addisu) Make this const
  public: sdf::ScopedGraph<sdf::PoseRelativeToGraph> poseRelativeToGraph;
};

/////////////////////////////////////////////////
SemanticPose::SemanticPose(
        const std::string &_name,
        const ignition::math::Pose3d &_pose,
        const std::string &_relativeTo,
        const std::string &_defaultResolveTo,
        const sdf::ScopedGraph<sdf::PoseRelativeToGraph> &_graph)
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
  this->dataPtr->name = _name;
  this->dataPtr->rawPose = _pose;
  this->dataPtr->relativeTo = _relativeTo;
  this->dataPtr->defaultResolveTo = _defaultResolveTo;
  this->dataPtr->poseRelativeToGraph = _graph;
}

/////////////////////////////////////////////////
SemanticPose::SemanticPose(
        const ignition::math::Pose3d &_pose,
        const std::string &_relativeTo,
        const std::string &_defaultResolveTo,
        const sdf::ScopedGraph<sdf::PoseRelativeToGraph> &_graph)
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
  this->dataPtr->rawPose = _pose;
  this->dataPtr->relativeTo = _relativeTo;
  this->dataPtr->defaultResolveTo = _defaultResolveTo;
  this->dataPtr->poseRelativeToGraph = _graph;
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &SemanticPose::RawPose() const
{
  return this->dataPtr->rawPose;
}

/////////////////////////////////////////////////
const std::string &SemanticPose::RelativeTo() const
{
  return this->dataPtr->relativeTo;
}

/////////////////////////////////////////////////
Errors SemanticPose::Resolve(
    ignition::math::Pose3d &_pose,
    const std::string &_resolveTo) const
{
  Errors errors;

  auto graph = this->dataPtr->poseRelativeToGraph;
  if (!graph)
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "SemanticPose has invalid pointer to PoseRelativeToGraph."});
    return errors;
  }

  std::string relativeTo = this->dataPtr->relativeTo;
  if (relativeTo.empty())
  {
    relativeTo = this->dataPtr->defaultResolveTo;
  }

  std::string resolveTo = _resolveTo;
  if (resolveTo.empty())
  {
    resolveTo = this->dataPtr->defaultResolveTo;
  }

  ignition::math::Pose3d pose;
  if (this->dataPtr->name.empty())
  {
    errors = resolvePose(pose, graph, relativeTo, resolveTo);
    pose *= this->RawPose();
  }
  else
  {
    errors = resolvePose(pose, graph, this->dataPtr->name, resolveTo);
  }

  if (errors.empty())
  {
    _pose = pose;
  }

  return errors;
}
}  // inline namespace
}  // namespace sdf
