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
#include <gz/math/Pose3.hh>
#include "sdf/Assert.hh"
#include "sdf/SemanticPose.hh"
#include "sdf/Error.hh"
#include "sdf/Types.hh"
#include "FrameSemantics.hh"
#include "Utils.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {
/// \internal
/// \brief Private data for the SemanticPose class.
class SemanticPosePrivate
{
  /// \brief Raw pose of the SemanticPose object.
  public: gz::math::Pose3d rawPose = gz::math::Pose3d::Zero;

  /// \brief Name of the relative-to frame.
  public: std::string relativeTo = "";

  /// \brief Name of the default frame to resolve to.
  public: std::string defaultResolveTo = "";

  /// \brief Weak pointer to model's Pose Relative-To Graph.
  public: std::weak_ptr<const sdf::PoseRelativeToGraph> poseRelativeToGraph;
};

/////////////////////////////////////////////////
SemanticPose::SemanticPose(
        const gz::math::Pose3d &_pose,
        const std::string &_relativeTo,
        const std::string &_defaultResolveTo,
        const std::weak_ptr<const sdf::PoseRelativeToGraph> _graph)
  : dataPtr(std::make_unique<SemanticPosePrivate>())
{
  this->dataPtr->rawPose = _pose;
  this->dataPtr->relativeTo = _relativeTo;
  this->dataPtr->defaultResolveTo = _defaultResolveTo;
  this->dataPtr->poseRelativeToGraph = _graph;
}

/////////////////////////////////////////////////
SemanticPose::~SemanticPose() = default;

/////////////////////////////////////////////////
SemanticPose::SemanticPose(const SemanticPose &_semanticPose)
  : dataPtr(std::make_unique<SemanticPosePrivate>(*_semanticPose.dataPtr))
{
}

/////////////////////////////////////////////////
SemanticPose::SemanticPose(SemanticPose &&_semanticPose) noexcept = default;

/////////////////////////////////////////////////
SemanticPose &SemanticPose::operator=(SemanticPose &&_semanticPose) = default;

/////////////////////////////////////////////////
SemanticPose &SemanticPose::operator=(const SemanticPose &_semanticPose)
{
  if (!this->dataPtr)
  {
    // This would happen if this object is moved from.
    this->dataPtr = std::make_unique<SemanticPosePrivate>();
  }
  *this->dataPtr =  *_semanticPose.dataPtr;
  return *this;
}

/////////////////////////////////////////////////
const gz::math::Pose3d &SemanticPose::RawPose() const
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
    gz::math::Pose3d &_pose,
    const std::string &_resolveTo) const
{
  Errors errors;

  auto graph = this->dataPtr->poseRelativeToGraph.lock();
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

  gz::math::Pose3d pose;
  errors = resolvePose(pose, *graph, relativeTo, resolveTo);
  pose *= this->RawPose();

  if (errors.empty())
  {
    _pose = pose;
  }

  return errors;
}
}  // inline namespace
}  // namespace sdf
