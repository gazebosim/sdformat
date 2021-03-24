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

#include "sdf/InterfaceModelPoseGraph.hh"

#include "FrameSemantics.hh"
#include "ScopedGraph.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
class InterfaceModelPoseGraph::Implementation
{
  /// \brief Pose relative-to graph anchored at the root node
  public: sdf::ScopedGraph<sdf::PoseRelativeToGraph> rootGraph;

  /// \brief Pose relative-to graph anchored at the scope of model
  public: sdf::ScopedGraph<sdf::PoseRelativeToGraph> modelGraph;

  /// \brief Vertex id of the interface model associated with this object.
  public: sdf::ScopedGraph<sdf::PoseRelativeToGraph>::VertexId modelVertexId;
};

InterfaceModelPoseGraph::InterfaceModelPoseGraph(
    const std::string &_name,
    const sdf::ScopedGraph<sdf::PoseRelativeToGraph> &_graph)
    : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
  this->dataPtr->modelGraph = _graph.ChildModelScope(_name);
  this->dataPtr->rootGraph = _graph.RootScope();
  this->dataPtr->modelVertexId = _graph.VertexIdByName(_name);
}

sdf::Errors InterfaceModelPoseGraph::ResolveNestedModelFramePoseInWorldFrame(
    ignition::math::Pose3d &_pose) const
{
  return sdf::resolvePose(_pose, this->dataPtr->rootGraph,
      this->dataPtr->modelVertexId, this->dataPtr->rootGraph.ScopeVertexId());
}

sdf::Errors InterfaceModelPoseGraph::ResolveNestedFramePose(
    ignition::math::Pose3d &_pose, const std::string &_frameName,
    const std::string &_relativeTo) const
{
  if (_relativeTo == "world")
  {
    const auto vertexId = this->dataPtr->modelGraph.VertexIdByName(_frameName);
    if (ignition::math::graph::kNullId == vertexId)
    {
      return {sdf::Error(sdf::ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
          "Frame name [" + _frameName + "] not found in pose graph.")};
    }
    return sdf::resolvePose(_pose, this->dataPtr->rootGraph, vertexId,
        this->dataPtr->rootGraph.ScopeVertexId());
  }

  return sdf::resolvePose(
      _pose, this->dataPtr->modelGraph, _frameName, _relativeTo);
}
}
}
