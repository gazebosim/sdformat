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
#include <string>

#include <ignition/math/graph/Graph.hh>

/// \ingroup sdf_frame_semantics
/// \brief namespace for Simulation Description Format Frame Semantics Utilities
///
/// The Frame Semantics Utilities construct and operate on graphs representing
/// the kinematics, frame attached_to, and pose relative_to relationships
/// defined within models and world.
namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  // Forward declaration.
  class Joint;
  class Link;
  class Model;
  class World;

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
  };

  /// \brief Data structure for kinematic graph for a Model.
  struct SDFORMAT_VISIBLE KinematicGraph
  {
    /// \brief A DirectedGraph with a vertex for each Link and an edge
    /// for each Joint.
    /// to the frame to which another frame is attached. Each vertex stores
    /// its FrameType and each edge can store a boolean value.
    using GraphType =
        ignition::math::graph::DirectedGraph<const Link*, const Joint*>;
    GraphType graph;

    /// \brief A Map from Vertex names to Vertex Ids.
    using MapType = std::map<std::string, ignition::math::graph::VertexId>;
    MapType map;
  };

  /// \brief Data structure for frame attached_to graphs for Model or World.
  struct SDFORMAT_VISIBLE FrameAttachedToGraph
  {
    /// \brief A DirectedGraph with a vertex for each frame and edges pointing
    /// to the frame to which another frame is attached. Each vertex stores
    /// its FrameType and each edge can store a boolean value.
    using GraphType = ignition::math::graph::DirectedGraph<FrameType, bool>;
    GraphType graph;

    /// \brief A Map from Vertex names to Vertex Ids.
    using MapType = std::map<std::string, ignition::math::graph::VertexId>;
    MapType map;
  };

  /// \brief Build a KinematicGraph for a model.
  /// \param[out] _out Graph object to write.
  /// \param[in] _model Model from which to build attached_to graph.
  /// \return Errors.
  SDFORMAT_VISIBLE
  Errors buildKinematicGraph(KinematicGraph &_out, const Model *_model);

  /// \brief Build a FrameAttachedToGraph for a model.
  /// \param[out] _out Graph object to write.
  /// \param[in] _model Model from which to build attached_to graph.
  /// \return Errors.
  SDFORMAT_VISIBLE
  Errors buildFrameAttachedToGraph(
              FrameAttachedToGraph &_out, const Model *_model);

  /// \brief Build a FrameAttachedToGraph for a world.
  /// \param[out] _out Graph object to write.
  /// \param[in] _world World from which to build attached_to graph.
  /// \return Errors.
  SDFORMAT_VISIBLE
  Errors buildFrameAttachedToGraph(
              FrameAttachedToGraph &_out, const World *_world);
  }
}
#endif
