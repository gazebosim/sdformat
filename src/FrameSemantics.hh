/*
 * Copyright 2019 Open Source Robotics Foundation
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
#ifndef SDF_FRAMESEMANTICS_HH_
#define SDF_FRAMESEMANTICS_HH_

#include <map>
#include <memory>
#include <string>

#include <ignition/math/Pose3.hh>
#include <ignition/math/graph/Graph.hh>

#include "sdf/Error.hh"
#include "sdf/InterfaceModel.hh"
#include "sdf/Types.hh"

/// \ingroup sdf_frame_semantics
/// \brief namespace for Simulation Description Format Frame Semantics Utilities
///
/// The Frame Semantics Utilities construct and operate on graphs representing
/// the kinematics, frame attached_to, and pose relative_to relationships
/// defined within models and world.
///
/// Note that all graphs should only contain relative names (e.g. "my_link"),
/// not absolute names ("top_model::nested_model::my_link").
/// Graphs inside nested models (currently via directly nested models in
/// //model or //world elements) will be explicitly separate graphs.
namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  // Forward declaration.
  class Model;
  class World;
  template <typename T> class ScopedGraph;

  /// \enum FrameType
  /// \brief The set of frame types. INVALID indicates that frame type has
  /// not been set, or has not been set correctly.
  enum class FrameType
  {
    /// \brief An implicit world frame.
    WORLD = 0,

    /// \brief An implicit model frame.
    MODEL = 1,

    /// \brief An implicit link frame.
    LINK = 2,

    /// \brief An implicit joint frame.
    JOINT = 3,

    /// \brief An explicit frame.
    FRAME = 4,

    /// \brief An implicit static model frame.
    STATIC_MODEL = 5,
  };

  /// \brief Data structure for frame attached_to graphs for Model or World.
  struct FrameAttachedToGraph
  {
    /// \brief A DirectedGraph with a vertex for each frame and edges pointing
    /// to the frame to which another frame is attached. Each vertex stores
    /// its FrameType and each edge can store a boolean value.
    using GraphType = ignition::math::graph::DirectedGraph<FrameType, bool>;
    GraphType graph;

    /// \brief A Map from Vertex names to Vertex Ids.
    using MapType = std::map<std::string, ignition::math::graph::VertexId>;
    MapType map;

    /// \brief Name of scope vertex, either __model__ or world.
    std::string scopeName;
  };

  /// \brief Data structure for pose relative_to graphs for Model or World.
  struct PoseRelativeToGraph
  {
    /// \brief A DirectedGraph with a vertex for each explicit or implicit
    /// frame and edges pointing to a given frame from its relative-to frame.
    /// When well-formed, it should form a directed tree with a root vertex
    /// named __model__ or world. Each vertex stores its FrameType and each edge
    /// stores the Pose3 between those frames.
    using Pose3d = ignition::math::Pose3d;
    using GraphType = ignition::math::graph::DirectedGraph<FrameType, Pose3d>;
    GraphType graph;

    /// \brief A Map from Vertex names to Vertex Ids.
    using MapType = std::map<std::string, ignition::math::graph::VertexId>;
    MapType map;

    /// \brief Name of source vertex, either __model__ or world.
    std::string sourceName;
  };

  /// \brief Build a FrameAttachedToGraph for a model.
  /// \param[out] _out Graph object to write.
  /// \param[in] _model Model from which to build attached_to graph.
  /// \param[in] _isRoot True if the model is a standalone model, i.e,
  /// //sdf/model.
  /// \return Errors.
  Errors buildFrameAttachedToGraph(ScopedGraph<FrameAttachedToGraph> &_out,
              const Model *_model, bool _isRoot = true);

  /// \brief Build a FrameAttachedToGraph for an Interface Model.
  /// \param[out] _out Graph object to write.
  /// \param[in] _model Interface model from which to build attached_to graph.
  /// \return Errors.
  Errors buildFrameAttachedToGraph(ScopedGraph<FrameAttachedToGraph> &_out,
              InterfaceModelConstPtr _model);

  /// \brief Build a FrameAttachedToGraph for a world.
  /// \param[out] _out Graph object to write.
  /// \param[in] _world World from which to build attached_to graph.
  /// \return Errors.
  Errors buildFrameAttachedToGraph(
              ScopedGraph<FrameAttachedToGraph> &_out, const World *_world);

  /// \brief Build a PoseRelativeToGraph for a model.
  /// \param[out] _out Graph object to write.
  /// \param[in] _model Model from which to build relative_to graph.
  /// \param[in] _isRoot True if the model is a standalone model, i.e,
  /// //sdf/model.
  /// \return Errors.
  Errors buildPoseRelativeToGraph(ScopedGraph<PoseRelativeToGraph> &_out,
              const Model *_model, bool _isRoot = true);

  /// \brief Build a PoseRelativeToGraph for an Interface model.
  /// \param[out] _out Graph object to write.
  /// \param[in] _model Interface model from which to build relative_to graph.
  /// \return Errors.
  Errors buildPoseRelativeToGraph(ScopedGraph<PoseRelativeToGraph> &_out,
              InterfaceModelConstPtr _model);

  /// \brief Build a PoseRelativeToGraph for a world.
  /// \param[out] _out Graph object to write.
  /// \param[in] _world World from which to build attached_to graph.
  /// \return Errors.
  Errors buildPoseRelativeToGraph(
              ScopedGraph<PoseRelativeToGraph> &_out, const World *_world);

  /// \brief Confirm that FrameAttachedToGraph is valid by checking the number
  /// of outbound edges for each vertex and checking for graph cycles.
  /// \param[in] _in Graph object to validate.
  /// \return Errors.
  Errors validateFrameAttachedToGraph(
      const ScopedGraph<FrameAttachedToGraph> &_in);

  /// \brief Confirm that PoseRelativeToGraph is valid by checking the number
  /// of outbound edges for each vertex and checking for graph cycles.
  /// \param[in] _in Graph object to validate.
  /// \return Errors.
  Errors validatePoseRelativeToGraph(
      const ScopedGraph<PoseRelativeToGraph> &_in);

  /// \brief Resolve the attached-to body for a given frame. Following the
  /// edges of the frame attached-to graph from a given frame must lead
  /// to a link or world frame.
  /// \param[out] _attachedToBody Name of link to which this frame is
  /// attached or "world" if frame is attached to the world.
  /// \param[in] _in Graph to use for resolving the body.
  /// \param[in] _vertexName This resolves the attached-to body of the
  /// vertex with this name.
  /// \return Errors if the graph is invalid or the frame does not lead to
  /// a link or world frame.
  Errors resolveFrameAttachedToBody(
      std::string &_attachedToBody,
      const ScopedGraph<FrameAttachedToGraph> &_in,
      const std::string &_vertexName);

  /// \brief Resolve pose of a vertex relative to its outgoing ancestor
  /// (analog of the root of a tree).
  /// \param[out] _pose Pose object to write.
  /// \param[in] _graph PoseRelativeToGraph to read from.
  /// \param[in] _vertexName Name of vertex whose pose is to be computed.
  /// \return Errors.
  Errors resolvePoseRelativeToRoot(
      ignition::math::Pose3d &_pose,
      const ScopedGraph<PoseRelativeToGraph> &_graph,
      const std::string &_vertexName);

  /// \brief Resolve pose of a vertex relative to its outgoing ancestor
  /// (analog of the root of a tree). This overload takes a vertex ID.
  /// \param[out] _pose Pose object to write.
  /// \param[in] _graph PoseRelativeToGraph to read from.
  /// \param[in] _vertexId Id of vertex whose pose is to be computed.
  /// \return Errors.
  Errors resolvePoseRelativeToRoot(
      ignition::math::Pose3d &_pose,
      const ScopedGraph<PoseRelativeToGraph> &_graph,
      const ignition::math::graph::VertexId &_vertexId);

  /// \brief Resolve pose of a frame relative to named frame.
  /// \param[out] _pose Pose object to write.
  /// \param[in] _graph PoseRelativeToGraph to read from.
  /// \param[in] _frameName Name of frame whose pose is to be resolved.
  /// \param[in] _resolveTo Name of frame relative to which the pose is
  /// to be resolved.
  /// \return Errors.
  Errors resolvePose(
      ignition::math::Pose3d &_pose,
      const ScopedGraph<PoseRelativeToGraph> &_graph,
      const std::string &_frameName,
      const std::string &_resolveTo);

  /// \brief Resolve pose of a vertex relative to another vertex in the pose
  /// graph.
  /// \param[out] _pose Pose object to write.
  /// \param[in] _graph PoseRelativeToGraph to read from.
  /// \param[in] _frameVertedId Verted Id of frame whose pose is to be resolved.
  /// \param[in] _resolveToVertexId Verted ID of frame relative to which the
  /// pose is to be resolved.
  /// \return Errors.
  Errors resolvePose(
      ignition::math::Pose3d &_pose,
      const ScopedGraph<PoseRelativeToGraph> &_graph,
      const ignition::math::graph::VertexId &_frameVertexId,
      const ignition::math::graph::VertexId &_resolveToVertexId);
  }
}
#endif
