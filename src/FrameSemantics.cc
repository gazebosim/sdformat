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
#include <string>
#include <utility>
#include <vector>

#include "sdf/Element.hh"
#include "sdf/Error.hh"
#include "sdf/Frame.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"

#include "FrameSemantics.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {

// The following two functions were originally submitted to ign-math,
// but were not accepted as they were not generic enough.
// For now, they will be kept here.
// https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/333

/// \brief Starting from a given vertex in a directed graph, traverse edges
/// in reverse direction to find a source vertex (has only outgoing edges).
/// This function returns a NullVertex if a graph cycle is detected or
/// if a vertex with multiple incoming edges is found.
/// Otherwise, this function returns the first source Vertex that is found.
/// It also returns the sequence of edges leading to the source vertex.
/// \param[in] _graph A directed graph.
/// \param[in] _id VertexId of the starting vertex.
/// \return A source vertex paired with a vector of the edges leading the
/// source to the starting vertex, or a NullVertex paired with an empty
/// vector if a cycle or vertex with multiple incoming edges are detected.
template<typename V, typename E>
std::pair<const ignition::math::graph::Vertex<V> &,
          std::vector< ignition::math::graph::DirectedEdge<E> > >
FindSourceVertex(
    const ignition::math::graph::DirectedGraph<V, E> &_graph,
    const ignition::math::graph::VertexId _id,
    Errors &_errors)
{
  using DirectedEdge = ignition::math::graph::DirectedEdge<E>;
  using Vertex = ignition::math::graph::Vertex<V>;
  using VertexId = ignition::math::graph::VertexId;
  using EdgesType = std::vector<DirectedEdge>;
  using PairType = std::pair<const Vertex &, EdgesType>;
  EdgesType edges;
  std::reference_wrapper<const Vertex> vertex(_graph.VertexFromId(_id));
  if (!vertex.get().Valid())
  {
    _errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
        "Unable to resolve pose, invalid vertex[" + std::to_string(_id) + "] "
        "in PoseRelativeToGraph."});
    return PairType(Vertex::NullVertex, EdgesType());
  }

  std::set<VertexId> visited;
  visited.insert(vertex.get().Id());

  auto incidentsTo = _graph.IncidentsTo(vertex);
  while (!incidentsTo.empty())
  {
    if (incidentsTo.size() != 1)
    {
      _errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
          "PoseRelativeToGraph error: multiple incoming edges to "
          "current vertex [" + vertex.get().Name() + "]."});
      return PairType(Vertex::NullVertex, EdgesType());
    }
    auto const &edge = incidentsTo.begin()->second;
    vertex = _graph.VertexFromId(edge.get().Vertices().first);
    edges.push_back(edge);
    if (visited.count(vertex.get().Id()))
    {
      _errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "PoseRelativeToGraph cycle detected, already visited vertex [" +
          vertex.get().Name() + "]."});
      return PairType(Vertex::NullVertex, EdgesType());
    }
    visited.insert(vertex.get().Id());
    incidentsTo = _graph.IncidentsTo(vertex);
  }

  return PairType(vertex, edges);
}

/// \brief Starting from a given vertex in a directed graph, follow edges
/// to find a sink vertex (has only incoming edges).
/// This function returns a NullVertex if a graph cycle is detected or
/// if a vertex with multiple outgoing edges is found.
/// Otherwise, this function returns the first sink Vertex that is found.
/// It also returns the sequence of edges leading to the sink vertex.
/// \param[in] _graph A directed graph.
/// \param[in] _id VertexId of the starting vertex.
/// \return A sink vertex paired with a vector of the edges leading the
/// sink to the starting vertex, or a NullVertex paired with an empty
/// vector if a cycle or vertex with multiple outgoing edges are detected.
template<typename V, typename E>
std::pair<const ignition::math::graph::Vertex<V> &,
          std::vector< ignition::math::graph::DirectedEdge<E> > >
FindSinkVertex(
    const ignition::math::graph::DirectedGraph<V, E> &_graph,
    const ignition::math::graph::VertexId _id,
    Errors &_errors)
{
  using DirectedEdge = ignition::math::graph::DirectedEdge<E>;
  using Vertex = ignition::math::graph::Vertex<V>;
  using VertexId = ignition::math::graph::VertexId;
  using EdgesType = std::vector<DirectedEdge>;
  using PairType = std::pair<const Vertex &, EdgesType>;
  EdgesType edges;
  std::reference_wrapper<const Vertex> vertex(_graph.VertexFromId(_id));
  if (!vertex.get().Valid())
  {
    _errors.push_back({ErrorCode::FRAME_ATTACHED_TO_INVALID,
        "Invalid vertex[" + std::to_string(_id) + "] "
        "in FrameAttachedToGraph."});
    return PairType(Vertex::NullVertex, EdgesType());
  }

  std::set<VertexId> visited;
  visited.insert(vertex.get().Id());

  auto incidentsFrom = _graph.IncidentsFrom(vertex);
  while (!incidentsFrom.empty())
  {
    if (incidentsFrom.size() != 1)
    {
      _errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
          "FrameAttachedToGraph error: multiple outgoing edges from "
          "current vertex [" + vertex.get().Name() + "]."});
      return PairType(Vertex::NullVertex, EdgesType());
    }
    auto const &edge = incidentsFrom.begin()->second;
    vertex = _graph.VertexFromId(edge.get().Vertices().second);
    edges.push_back(edge);
    if (visited.count(vertex.get().Id()))
    {
      _errors.push_back({ErrorCode::FRAME_ATTACHED_TO_CYCLE,
          "FrameAttachedToGraph cycle detected, already visited vertex [" +
          vertex.get().Name() + "]."});
      return PairType(Vertex::NullVertex, EdgesType());
    }
    visited.insert(vertex.get().Id());
    incidentsFrom = _graph.IncidentsFrom(vertex);
  }

  return PairType(vertex, edges);
}

/////////////////////////////////////////////////
std::pair<const Link *, std::string>
    modelCanonicalLinkAndRelativeName(const Model *_model)
{
  if (nullptr == _model)
  {
    return std::make_pair(nullptr, "");
  }
  return _model->CanonicalLinkAndRelativeName();
}

/////////////////////////////////////////////////
Errors buildFrameAttachedToGraph(
            FrameAttachedToGraph &_out, const Model *_model)
{
  Errors errors;

  // add implicit model frame vertex first
  const std::string scopeName = "__model__";
  _out.scopeName = scopeName;
  auto modelFrameId =
      _out.graph.AddVertex(scopeName, sdf::FrameType::MODEL).Id();
  _out.map[scopeName] = modelFrameId;


  if (!_model)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Invalid sdf::Model pointer."});
    return errors;
  }
  else if (!_model->Element())
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Invalid model element in sdf::Model."});
    return errors;
  }
  else if (_model->LinkCount() < 1 && _model->ModelCount() < 1)
  {
    errors.push_back({ErrorCode::MODEL_WITHOUT_LINK,
                     "A model must have at least one link."});
    return errors;
  }

  // identify canonical link, which may be nested
  auto canonicalLinkAndName = modelCanonicalLinkAndRelativeName(_model);
  const sdf::Link *canonicalLink = canonicalLinkAndName.first;
  const std::string canonicalLinkName = canonicalLinkAndName.second;
  if (nullptr == canonicalLink)
  {
    if (canonicalLinkName.empty())
    {
      errors.push_back({ErrorCode::MODEL_WITHOUT_LINK,
                       "A model must have at least one link."});
    }
    else
    {
      errors.push_back({ErrorCode::MODEL_CANONICAL_LINK_INVALID,
        "canonical_link with name[" + canonicalLinkName +
        "] not found in model with name[" + _model->Name() + "]."});
    }
    // return early
    return errors;
  }

  // Identify if the canonical link is in a nested model.
  if (_model->LinkByName(canonicalLink->Name()) != canonicalLink)
  {
    // The canonical link is nested, so its vertex should be added
    // here with an edge from __model__.
    // The nested canonical link name should be a nested name
    // relative to _model, delimited by "::".
    auto linkId =
        _out.graph.AddVertex(canonicalLinkName, sdf::FrameType::LINK).Id();
    _out.map[canonicalLinkName] = linkId;
    _out.graph.AddEdge({modelFrameId, linkId}, true);
  }

  // add link vertices
  for (uint64_t l = 0; l < _model->LinkCount(); ++l)
  {
    auto link = _model->LinkByIndex(l);
    if (_out.map.count(link->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Link with non-unique name [" + link->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    auto linkId =
        _out.graph.AddVertex(link->Name(), sdf::FrameType::LINK).Id();
    _out.map[link->Name()] = linkId;

    // add edge from implicit model frame vertex to canonical link
    if (link == canonicalLink)
    {
      _out.graph.AddEdge({modelFrameId, linkId}, true);
    }
  }

  // add joint vertices
  for (uint64_t j = 0; j < _model->JointCount(); ++j)
  {
    auto joint = _model->JointByIndex(j);
    if (_out.map.count(joint->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Joint with non-unique name [" + joint->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    auto jointId =
        _out.graph.AddVertex(joint->Name(), sdf::FrameType::JOINT).Id();
    _out.map[joint->Name()] = jointId;
  }

  // add frame vertices
  for (uint64_t f = 0; f < _model->FrameCount(); ++f)
  {
    auto frame = _model->FrameByIndex(f);
    if (_out.map.count(frame->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Frame with non-unique name [" + frame->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    auto frameId =
        _out.graph.AddVertex(frame->Name(), sdf::FrameType::FRAME).Id();
    _out.map[frame->Name()] = frameId;
  }

  // add nested model vertices
  for (uint64_t m = 0; m < _model->ModelCount(); ++m)
  {
    auto nestedModel = _model->ModelByIndex(m);
    if (_out.map.count(nestedModel->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Nested model with non-unique name [" + nestedModel->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    auto nestedModelId =
        _out.graph.AddVertex(nestedModel->Name(), sdf::FrameType::MODEL).Id();
    _out.map[nestedModel->Name()] = nestedModelId;
  }

  // add edges from joint to child frames
  for (uint64_t j = 0; j < _model->JointCount(); ++j)
  {
    auto joint = _model->JointByIndex(j);
    auto jointId = _out.map.at(joint->Name());
    auto childFrameName = joint->ChildLinkName();
    if (_out.map.count(childFrameName) != 1)
    {
      errors.push_back({ErrorCode::JOINT_CHILD_LINK_INVALID,
        "Child frame with name[" + childFrameName +
        "] specified by joint with name[" + joint->Name() +
        "] not found in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto childFrameId = _out.map.at(childFrameName);
    _out.graph.AddEdge({jointId, childFrameId}, true);
  }

  // add frame edges
  for (uint64_t f = 0; f < _model->FrameCount(); ++f)
  {
    auto frame = _model->FrameByIndex(f);
    auto frameId = _out.map.at(frame->Name());
    // look for vertex in graph that matches attached_to value
    std::string attachedTo = frame->AttachedTo();
    if (attachedTo.empty())
    {
      // if the attached-to name is empty, use the scope name
      attachedTo = scopeName;
    }
    if (_out.map.count(attachedTo) != 1)
    {
      errors.push_back({ErrorCode::FRAME_ATTACHED_TO_INVALID,
          "attached_to name[" + attachedTo +
          "] specified by frame with name[" + frame->Name() +
          "] does not match a nested model, link, joint, or frame name "
          "in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto attachedToId = _out.map[attachedTo];
    bool edgeData = true;
    if (frame->Name() == frame->AttachedTo())
    {
      // set edgeData to false if attaches to itself, since this is invalid
      edgeData = false;
      errors.push_back({ErrorCode::FRAME_ATTACHED_TO_CYCLE,
          "attached_to name[" + attachedTo +
          "] is identical to frame name[" + frame->Name() +
          "], causing a graph cycle "
          "in model with name[" + _model->Name() + "]."});
    }
    _out.graph.AddEdge({frameId, attachedToId}, edgeData);
  }

  return errors;
}

/////////////////////////////////////////////////
Errors buildFrameAttachedToGraph(
            FrameAttachedToGraph &_out, const World *_world)
{
  Errors errors;

  // add implicit world frame vertex first
  const std::string scopeName = "world";
  _out.scopeName = scopeName;
  auto worldFrameId =
      _out.graph.AddVertex(scopeName, sdf::FrameType::WORLD).Id();
  _out.map[scopeName] = worldFrameId;


  if (!_world)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Invalid sdf::World pointer."});
    return errors;
  }
  else if (!_world->Element())
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Invalid world element in sdf::World."});
    return errors;
  }

  // add model vertices
  for (uint64_t m = 0; m < _world->ModelCount(); ++m)
  {
    auto model = _world->ModelByIndex(m);
    if (_out.map.count(model->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Model with non-unique name [" + model->Name() +
          "] detected in world with name [" + _world->Name() +
          "]."});
      continue;
    }
    auto modelId =
        _out.graph.AddVertex(model->Name(), sdf::FrameType::MODEL).Id();
    _out.map[model->Name()] = modelId;
  }

  // add frame vertices
  for (uint64_t f = 0; f < _world->FrameCount(); ++f)
  {
    auto frame = _world->FrameByIndex(f);
    if (_out.map.count(frame->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Frame with non-unique name [" + frame->Name() +
          "] detected in world with name [" + _world->Name() +
          "]."});
      continue;
    }
    auto frameId =
        _out.graph.AddVertex(frame->Name(), sdf::FrameType::FRAME).Id();
    _out.map[frame->Name()] = frameId;
  }

  // add frame edges
  for (uint64_t f = 0; f < _world->FrameCount(); ++f)
  {
    auto frame = _world->FrameByIndex(f);
    auto frameId = _out.map.at(frame->Name());
    // look for vertex in graph that matches attached_to value
    std::string attachedTo = frame->AttachedTo();
    if (attachedTo.empty())
    {
      // if the attached-to name is empty, use the scope name
      attachedTo = scopeName;
      if (_out.map.count(scopeName) != 1)
      {
        errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                         "FrameAttachedToGraph error: scope frame[" +
                         scopeName + "] not found in map."});
        continue;
      }
    }
    if (_out.map.count(attachedTo) != 1)
    {
      errors.push_back({ErrorCode::FRAME_ATTACHED_TO_INVALID,
          "attached_to name[" + attachedTo +
          "] specified by frame with name[" + frame->Name() +
          "] does not match a model or frame name "
          "in world with name[" + _world->Name() + "]."});
      continue;
    }
    auto attachedToId = _out.map[attachedTo];
    bool edgeData = true;
    if (frame->Name() == frame->AttachedTo())
    {
      // set edgeData to false if attaches to itself, since this is invalid
      edgeData = false;
      errors.push_back({ErrorCode::FRAME_ATTACHED_TO_CYCLE,
          "attached_to name[" + attachedTo +
          "] is identical to frame name[" + frame->Name() +
          "], causing a graph cycle "
          "in world with name[" + _world->Name() + "]."});
    }
    _out.graph.AddEdge({frameId, attachedToId}, edgeData);
  }

  return errors;
}

/////////////////////////////////////////////////
Errors buildPoseRelativeToGraph(
            PoseRelativeToGraph &_out, const Model *_model)
{
  Errors errors;

  if (!_model)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Invalid sdf::Model pointer."});
    return errors;
  }
  else if (!_model->Element())
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Invalid model element in sdf::Model."});
    return errors;
  }

  // add implicit model frame vertex first
  const std::string sourceName = "__model__";
  _out.sourceName = sourceName;
  auto modelFrameId =
      _out.graph.AddVertex(sourceName, sdf::FrameType::MODEL).Id();
  _out.map[sourceName] = modelFrameId;

  // add link vertices and default edge if relative_to is empty
  for (uint64_t l = 0; l < _model->LinkCount(); ++l)
  {
    auto link = _model->LinkByIndex(l);
    if (_out.map.count(link->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Link with non-unique name [" + link->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    auto linkId =
        _out.graph.AddVertex(link->Name(), sdf::FrameType::LINK).Id();
    _out.map[link->Name()] = linkId;

    if (link->PoseRelativeTo().empty())
    {
      // relative_to is empty, so add edge from implicit model frame to link
      _out.graph.AddEdge({modelFrameId, linkId}, link->RawPose());
    }
  }

  // add joint vertices
  for (uint64_t j = 0; j < _model->JointCount(); ++j)
  {
    auto joint = _model->JointByIndex(j);
    if (_out.map.count(joint->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Joint with non-unique name [" + joint->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    auto jointId =
        _out.graph.AddVertex(joint->Name(), sdf::FrameType::JOINT).Id();
    _out.map[joint->Name()] = jointId;
  }

  // add frame vertices and default edge if both
  // relative_to and attached_to are empty
  for (uint64_t f = 0; f < _model->FrameCount(); ++f)
  {
    auto frame = _model->FrameByIndex(f);
    if (_out.map.count(frame->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Frame with non-unique name [" + frame->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    auto frameId =
        _out.graph.AddVertex(frame->Name(), sdf::FrameType::FRAME).Id();
    _out.map[frame->Name()] = frameId;

    if (frame->PoseRelativeTo().empty() && frame->AttachedTo().empty())
    {
      // add edge from implicit model frame to frame
      _out.graph.AddEdge({modelFrameId, frameId}, frame->RawPose());
    }
  }

  // add nested model vertices and default edge if relative_to is empty
  for (uint64_t m = 0; m < _model->ModelCount(); ++m)
  {
    auto nestedModel = _model->ModelByIndex(m);
    if (_out.map.count(nestedModel->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Nested model with non-unique name [" + nestedModel->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    auto nestedModelId =
        _out.graph.AddVertex(nestedModel->Name(), sdf::FrameType::MODEL).Id();
    _out.map[nestedModel->Name()] = nestedModelId;

    if (nestedModel->PoseRelativeTo().empty())
    {
      // relative_to is empty, so add edge from implicit model frame
      // to nestedModel
      _out.graph.AddEdge({modelFrameId, nestedModelId}, nestedModel->RawPose());
    }
  }

  // now that all vertices have been added to the graph,
  // add the edges that reference other vertices

  for (uint64_t l = 0; l < _model->LinkCount(); ++l)
  {
    auto link = _model->LinkByIndex(l);

    // check if we've already added a default edge
    const std::string relativeTo = link->PoseRelativeTo();
    if (relativeTo.empty())
    {
      continue;
    }

    auto linkId = _out.map.at(link->Name());

    // look for vertex in graph that matches relative_to value
    if (_out.map.count(relativeTo) != 1)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
          "relative_to name[" + relativeTo +
          "] specified by link with name[" + link->Name() +
          "] does not match a nested model, link, joint, or frame name "
          "in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto relativeToId = _out.map[relativeTo];
    if (link->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "relative_to name[" + relativeTo +
          "] is identical to link name[" + link->Name() +
          "], causing a graph cycle "
          "in model with name[" + _model->Name() + "]."});
    }
    _out.graph.AddEdge({relativeToId, linkId}, link->RawPose());
  }

  for (uint64_t j = 0; j < _model->JointCount(); ++j)
  {
    auto joint = _model->JointByIndex(j);

    std::string relativeTo = joint->PoseRelativeTo();
    if (relativeTo.empty())
    {
      // since nothing else was specified, use the joint's child frame
      relativeTo = joint->ChildLinkName();
    }

    auto jointId = _out.map.at(joint->Name());

    // look for vertex in graph that matches relative_to value
    if (_out.map.count(relativeTo) != 1)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
          "relative_to name[" + relativeTo +
          "] specified by joint with name[" + joint->Name() +
          "] does not match a nested model, link, joint, or frame name "
          "in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto relativeToId = _out.map[relativeTo];
    if (joint->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "relative_to name[" + relativeTo +
          "] is identical to joint name[" + joint->Name() +
          "], causing a graph cycle "
          "in model with name[" + _model->Name() + "]."});
    }
    _out.graph.AddEdge({relativeToId, jointId}, joint->RawPose());
  }

  for (uint64_t f = 0; f < _model->FrameCount(); ++f)
  {
    auto frame = _model->FrameByIndex(f);

    // check if we've already added a default edge
    if (frame->PoseRelativeTo().empty() && frame->AttachedTo().empty())
    {
      continue;
    }

    auto frameId = _out.map.at(frame->Name());
    std::string relativeTo;
    std::string typeForErrorMsg;
    ErrorCode errorCode;
    if (!frame->PoseRelativeTo().empty())
    {
      relativeTo = frame->PoseRelativeTo();
      typeForErrorMsg = "relative_to";
      errorCode = ErrorCode::POSE_RELATIVE_TO_INVALID;
    }
    else
    {
      relativeTo = frame->AttachedTo();
      typeForErrorMsg = "attached_to";
      errorCode = ErrorCode::FRAME_ATTACHED_TO_INVALID;
    }

    // look for vertex in graph that matches relative_to value
    if (_out.map.count(relativeTo) != 1)
    {
      errors.push_back({errorCode,
          typeForErrorMsg + " name[" + relativeTo +
          "] specified by frame with name[" + frame->Name() +
          "] does not match a nested model, link, joint, or frame name "
          "in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto relativeToId = _out.map[relativeTo];
    if (frame->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "relative_to name[" + relativeTo +
          "] is identical to frame name[" + frame->Name() +
          "], causing a graph cycle "
          "in model with name[" + _model->Name() + "]."});
    }
    _out.graph.AddEdge({relativeToId, frameId}, frame->RawPose());
  }

  for (uint64_t m = 0; m < _model->ModelCount(); ++m)
  {
    auto nestedModel = _model->ModelByIndex(m);

    // check if we've already added a default edge
    const std::string relativeTo = nestedModel->PoseRelativeTo();
    if (relativeTo.empty())
    {
      continue;
    }

    auto nestedModelId = _out.map.at(nestedModel->Name());

    // look for vertex in graph that matches relative_to value
    if (_out.map.count(relativeTo) != 1)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
          "relative_to name[" + relativeTo +
          "] specified by nested model with name[" + nestedModel->Name() +
          "] does not match a nested model, link, joint, or frame name "
          "in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto relativeToId = _out.map[relativeTo];
    if (nestedModel->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "relative_to name[" + relativeTo +
          "] is identical to nested model name[" + nestedModel->Name() +
          "], causing a graph cycle "
          "in model with name[" + _model->Name() + "]."});
    }
    _out.graph.AddEdge({relativeToId, nestedModelId}, nestedModel->RawPose());
  }

  return errors;
}

/////////////////////////////////////////////////
Errors buildPoseRelativeToGraph(
            PoseRelativeToGraph &_out, const World *_world)
{
  Errors errors;

  if (!_world)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Invalid sdf::World pointer."});
    return errors;
  }
  else if (!_world->Element())
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Invalid world element in sdf::World."});
    return errors;
  }

  // add implicit world frame vertex first
  const std::string sourceName = "world";
  _out.sourceName = sourceName;
  auto worldFrameId =
      _out.graph.AddVertex(sourceName, sdf::FrameType::WORLD).Id();
  _out.map[sourceName] = worldFrameId;

  // add model vertices and default edge if relative_to is empty
  for (uint64_t m = 0; m < _world->ModelCount(); ++m)
  {
    auto model = _world->ModelByIndex(m);
    if (_out.map.count(model->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Model with non-unique name [" + model->Name() +
          "] detected in world with name [" + _world->Name() +
          "]."});
      continue;
    }
    auto modelId =
        _out.graph.AddVertex(model->Name(), sdf::FrameType::MODEL).Id();
    _out.map[model->Name()] = modelId;

    if (model->PoseRelativeTo().empty())
    {
      // relative_to is empty, so add edge from implicit world frame to model
      _out.graph.AddEdge({worldFrameId, modelId}, model->RawPose());
    }
  }

  // add frame vertices and default edge if both
  // relative_to and attached_to are empty
  for (uint64_t f = 0; f < _world->FrameCount(); ++f)
  {
    auto frame = _world->FrameByIndex(f);
    if (_out.map.count(frame->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Frame with non-unique name [" + frame->Name() +
          "] detected in world with name [" + _world->Name() +
          "]."});
      continue;
    }
    auto frameId =
        _out.graph.AddVertex(frame->Name(), sdf::FrameType::FRAME).Id();
    _out.map[frame->Name()] = frameId;

    if (frame->PoseRelativeTo().empty() && frame->AttachedTo().empty())
    {
      // add edge from implicit world frame to frame
      _out.graph.AddEdge({worldFrameId, frameId}, frame->RawPose());
    }
  }

  // now that all vertices have been added to the graph,
  // add the edges that reference other vertices

  for (uint64_t m = 0; m < _world->ModelCount(); ++m)
  {
    auto model = _world->ModelByIndex(m);

    // check if we've already added a default edge
    const std::string relativeTo = model->PoseRelativeTo();
    if (relativeTo.empty())
    {
      continue;
    }

    auto modelId = _out.map.at(model->Name());

    // look for vertex in graph that matches relative_to value
    if (_out.map.count(relativeTo) != 1)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
          "relative_to name[" + relativeTo +
          "] specified by model with name[" + model->Name() +
          "] does not match a model or frame name "
          "in world with name[" + _world->Name() + "]."});
      continue;
    }
    auto relativeToId = _out.map[relativeTo];
    if (model->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "relative_to name[" + relativeTo +
          "] is identical to model name[" + model->Name() +
          "], causing a graph cycle "
          "in world with name[" + _world->Name() + "]."});
    }
    _out.graph.AddEdge({relativeToId, modelId}, model->RawPose());
  }

  for (uint64_t f = 0; f < _world->FrameCount(); ++f)
  {
    auto frame = _world->FrameByIndex(f);

    // check if we've already added a default edge
    if (frame->PoseRelativeTo().empty() && frame->AttachedTo().empty())
    {
      continue;
    }

    auto frameId = _out.map.at(frame->Name());
    std::string relativeTo;
    std::string typeForErrorMsg;
    ErrorCode errorCode;
    if (!frame->PoseRelativeTo().empty())
    {
      relativeTo = frame->PoseRelativeTo();
      typeForErrorMsg = "relative_to";
      errorCode = ErrorCode::POSE_RELATIVE_TO_INVALID;
    }
    else
    {
      relativeTo = frame->AttachedTo();
      typeForErrorMsg = "attached_to";
      errorCode = ErrorCode::FRAME_ATTACHED_TO_INVALID;
    }

    // look for vertex in graph that matches relative_to value
    if (_out.map.count(relativeTo) != 1)
    {
      errors.push_back({errorCode,
          typeForErrorMsg + " name[" + relativeTo +
          "] specified by frame with name[" + frame->Name() +
          "] does not match a model or frame name "
          "in world with name[" + _world->Name() + "]."});
      continue;
    }
    auto relativeToId = _out.map[relativeTo];
    if (frame->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "relative_to name[" + relativeTo +
          "] is identical to frame name[" + frame->Name() +
          "], causing a graph cycle "
          "in world with name[" + _world->Name() + "]."});
    }
    _out.graph.AddEdge({relativeToId, frameId}, frame->RawPose());
  }

  return errors;
}

/////////////////////////////////////////////////
Errors validateFrameAttachedToGraph(const FrameAttachedToGraph &_in)
{
  Errors errors;

  // Expect scopeName to be either "__model__" or "world"
  if (_in.scopeName != "__model__" && _in.scopeName != "world")
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "FrameAttachedToGraph error: scope frame[" + _in.scopeName + "] "
        " does not match __model__ or world."});
    return errors;
  }

  // Expect one vertex with name "__model__" and FrameType MODEL
  // or with name "world" and FrameType WORLD
  auto scopeVertices = _in.graph.Vertices(_in.scopeName);
  if (scopeVertices.empty())
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                     "FrameAttachedToGraph error: scope frame[" +
                     _in.scopeName + "] not found in graph."});
    return errors;
  }
  else if (scopeVertices.size() > 1)
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "FrameAttachedToGraph error, "
        "more than one vertex with scope name " + _in.scopeName});
    return errors;
  }

  auto scopeVertex = scopeVertices.begin()->second.get();
  sdf::FrameType scopeFrameType = scopeVertex.Data();
  if (_in.scopeName == "__model__" && scopeFrameType != sdf::FrameType::MODEL)
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "FrameAttachedToGraph error, "
        "scope vertex with name __model__ should have FrameType MODEL."});
    return errors;
  }
  else if (_in.scopeName == "world" &&
           scopeFrameType != sdf::FrameType::WORLD)
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "FrameAttachedToGraph error, "
        "scope vertex with name world should have FrameType WORLD."});
    return errors;
  }

  // Check number of outgoing edges for each vertex
  auto vertices = _in.graph.Vertices();
  for (auto vertexPair : vertices)
  {
    // Vertex names should not be empty
    if (vertexPair.second.get().Name().empty())
    {
      errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
          "FrameAttachedToGraph error, "
          "vertex with empty name detected."});
    }

    auto outDegree = _in.graph.OutDegree(vertexPair.first);
    if (outDegree > 1)
    {
      errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
          "FrameAttachedToGraph error, "
          "too many outgoing edges at a vertex with name [" +
          vertexPair.second.get().Name() + "]."});
    }
    else if (sdf::FrameType::MODEL == scopeFrameType)
    {
      switch (vertexPair.second.get().Data())
      {
        case sdf::FrameType::WORLD:
          errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
              "FrameAttachedToGraph error, "
              "vertex with name [" + vertexPair.second.get().Name() + "]" +
              "should not have type WORLD in MODEL attached_to graph."});
          break;
        case sdf::FrameType::LINK:
          if (outDegree != 0)
          {
            errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                "FrameAttachedToGraph error, "
                "LINK vertex with name [" +
                vertexPair.second.get().Name() +
                "] should have no outgoing edges "
                "in MODEL attached_to graph."});
          }
          break;
        case sdf::FrameType::MODEL:
          if ("__model__" != vertexPair.second.get().Name())
          {
            if (outDegree != 0)
            {
              errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                  "FrameAttachedToGraph error, "
                  "nested MODEL vertex with name [" +
                  vertexPair.second.get().Name() +
                  "] should have no outgoing edges "
                  "in MODEL attached_to graph."});
            }
            break;
          }
          // fall through to default case for __model__
          [[fallthrough]];
        default:
          if (outDegree == 0)
          {
            errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                "FrameAttachedToGraph error, "
                "Non-LINK vertex with name [" +
                vertexPair.second.get().Name() +
                "] is disconnected; it should have 1 outgoing edge " +
                "in MODEL attached_to graph."});
          }
          else if (outDegree >= 2)
          {
            errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                "FrameAttachedToGraph error, "
                "Non-LINK vertex with name [" +
                vertexPair.second.get().Name() +
                "] has " + std::to_string(outDegree) +
                " outgoing edges; it should only have 1 "
                "outgoing edge in MODEL attached_to graph."});
          }
          break;
      }
    }
    else
    {
      // scopeFrameType must be sdf::FrameType::WORLD
      switch (vertexPair.second.get().Data())
      {
        case sdf::FrameType::JOINT:
        case sdf::FrameType::LINK:
          errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
              "FrameAttachedToGraph error, "
              "No JOINT or LINK vertex should be in WORLD attached_to graph."});
          break;
        case sdf::FrameType::MODEL:
        case sdf::FrameType::WORLD:
          if (outDegree != 0)
          {
            errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                "FrameAttachedToGraph error, "
                "MODEL and WORLD vertices should have no outgoing edges "
                "in WORLD attached_to graph."});
          }
          break;
        default:
          if (outDegree != 1)
          {
            errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                "FrameAttachedToGraph error, "
                "FRAME vertices in WORLD attached_to graph should have "
                "1 outgoing edge."});
          }
          break;
      }
    }
  }

  // check graph for cycles by finding sink from each vertex
  for (auto const &namePair : _in.map)
  {
    std::string resolvedBody;
    Errors e = resolveFrameAttachedToBody(resolvedBody, _in, namePair.first);
    errors.insert(errors.end(), e.begin(), e.end());
  }

  return errors;
}

/////////////////////////////////////////////////
Errors validatePoseRelativeToGraph(const PoseRelativeToGraph &_in)
{
  Errors errors;

  // Expect sourceName to be either "__model__" or "world"
  if (_in.sourceName != "__model__" && _in.sourceName != "world")
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph error: source vertex name " + _in.sourceName +
        " does not match __model__ or world."});
    return errors;
  }

  // Expect one vertex with name "__model__" and FrameType MODEL
  // or with name "world" and FrameType WORLD
  auto sourceVertices = _in.graph.Vertices(_in.sourceName);
  if (sourceVertices.empty())
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
                     "PoseRelativeToGraph error: source frame[" +
                     _in.sourceName + "] not found in graph."});
    return errors;
  }
  else if (sourceVertices.size() > 1)
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph error, "
        "more than one vertex with source name " + _in.sourceName});
    return errors;
  }

  auto sourceVertex = sourceVertices.begin()->second.get();
  sdf::FrameType sourceFrameType = sourceVertex.Data();
  if (_in.sourceName == "__model__" && sourceFrameType != sdf::FrameType::MODEL)
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph error, "
        "source vertex with name __model__ should have FrameType MODEL."});
    return errors;
  }
  else if (_in.sourceName == "world" &&
           sourceFrameType != sdf::FrameType::WORLD)
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph error, "
        "source vertex with name world should have FrameType WORLD."});
    return errors;
  }

  // Check number of incoming edges for each vertex
  auto vertices = _in.graph.Vertices();
  for (auto vertexPair : vertices)
  {
    // Vertex names should not be empty
    if (vertexPair.second.get().Name().empty())
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
          "PoseRelativeToGraph error, "
          "vertex with empty name detected."});
    }

    auto inDegree = _in.graph.InDegree(vertexPair.first);
    if (inDegree > 1)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
          "PoseRelativeToGraph error, "
          "too many incoming edges at a vertex with name [" +
          vertexPair.second.get().Name() + "]."});
    }
    else if (sdf::FrameType::MODEL == sourceFrameType)
    {
      switch (vertexPair.second.get().Data())
      {
        case sdf::FrameType::WORLD:
          errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
              "PoseRelativeToGraph error, "
              "vertex with name [" + vertexPair.second.get().Name() + "]" +
              "should not have type WORLD in MODEL relative_to graph."});
          break;
        case sdf::FrameType::MODEL:
          if ("__model__" == vertexPair.second.get().Name())
          {
            if (inDegree != 0)
            {
              errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
                  "PoseRelativeToGraph error, "
                  "MODEL vertex with name [__model__"
                  "] should have no incoming edges "
                  "in MODEL relative_to graph."});
            }
            break;
          }
          // fall through to default case for nested models
          [[fallthrough]];
        default:
          if (inDegree == 0)
          {
            errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
                "PoseRelativeToGraph error, "
                "Vertex with name [" +
                vertexPair.second.get().Name() +
                "] is disconnected; it should have 1 incoming edge " +
                "in MODEL relative_to graph."});
          }
          else if (inDegree >= 2)
          {
            errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
                "PoseRelativeToGraph error, "
                "Non-MODEL vertex with name [" +
                vertexPair.second.get().Name() +
                "] has " + std::to_string(inDegree) +
                " incoming edges; it should only have 1 "
                "incoming edge in MODEL relative_to graph."});
          }
          break;
      }
    }
    else
    {
      // sourceFrameType must be sdf::FrameType::WORLD
      switch (vertexPair.second.get().Data())
      {
        case sdf::FrameType::JOINT:
        case sdf::FrameType::LINK:
          errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
              "PoseRelativeToGraph error, "
              "No JOINT or LINK vertex should be in WORLD relative_to graph."});
          break;
        case sdf::FrameType::WORLD:
          if (inDegree != 0)
          {
            errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
                "PoseRelativeToGraph error, "
                "WORLD vertices should have no incoming edges "
                "in WORLD relative_to graph."});
          }
          break;
        default:
          if (inDegree == 0)
          {
            errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
                "PoseRelativeToGraph error, "
                "MODEL / FRAME vertex with name [" +
                vertexPair.second.get().Name() +
                "] is disconnected; it should have 1 incoming edge " +
                "in WORLD relative_to graph."});
          }
          else if (inDegree >= 2)
          {
            errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
                "PoseRelativeToGraph error, "
                "MODEL / FRAME vertex with name [" +
                vertexPair.second.get().Name() +
                "] has " + std::to_string(inDegree) +
                " incoming edges; it should only have 1 "
                "incoming edge in WORLD relative_to graph."});
          }
          break;
      }
    }
  }

  // check graph for cycles by resolving pose of each vertex relative to root
  for (auto const &namePair : _in.map)
  {
    ignition::math::Pose3d pose;
    Errors e = resolvePoseRelativeToRoot(pose, _in, namePair.first);
    errors.insert(errors.end(), e.begin(), e.end());
  }

  return errors;
}

/////////////////////////////////////////////////
Errors resolveFrameAttachedToBody(
    std::string &_attachedToBody,
    const FrameAttachedToGraph &_in,
    const std::string &_vertexName)
{
  Errors errors;

  if (_in.scopeName != "__model__" && _in.scopeName != "world")
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "FrameAttachedToGraph error: scope frame[" + _in.scopeName + "] "
        " does not match __model__ or world."});
    return errors;
  }

  if (_in.map.count(_vertexName) != 1)
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_INVALID,
        "FrameAttachedToGraph unable to find unique frame with name [" +
        _vertexName + "] in graph."});
    return errors;
  }
  auto vertexId = _in.map.at(_vertexName);

  auto sinkVertexEdges = FindSinkVertex(_in.graph, vertexId, errors);
  auto sinkVertex = sinkVertexEdges.first;

  if (!errors.empty())
  {
    return errors;
  }

  if (!sinkVertex.Valid())
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "FrameAttachedToGraph unable to find sink vertex when starting "
        "from vertex with name [" + _vertexName + "]."});
    return errors;
  }

  if (_in.scopeName == "world" &&
      !(sinkVertex.Data() == FrameType::WORLD ||
        sinkVertex.Data() == FrameType::MODEL))
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "Graph has world scope but sink vertex named [" +
        sinkVertex.Name() + "] does not have FrameType WORLD or MODEL "
        "when starting from vertex with name [" + _vertexName + "]."});
    return errors;
  }

  if (_in.scopeName == "__model__")
  {
    if (sinkVertex.Data() == FrameType::MODEL &&
        sinkVertex.Name() == "__model__")
    {
      errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
          "Graph with __model__ scope has sink vertex named [__model__] "
          "when starting from vertex with name [" + _vertexName + "], "
          "which is not permitted."});
      return errors;
    }
    else if (sinkVertex.Data() != FrameType::LINK &&
             sinkVertex.Data() != FrameType::MODEL)
    {
      errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
          "Graph has __model__ scope but sink vertex named [" +
          sinkVertex.Name() + "] does not have FrameType LINK OR MODEL "
          "when starting from vertex with name [" + _vertexName + "]."});
      return errors;
    }
  }

  _attachedToBody = sinkVertex.Name();

  return errors;
}

/////////////////////////////////////////////////
Errors resolvePoseRelativeToRoot(
      ignition::math::Pose3d &_pose,
      const PoseRelativeToGraph &_graph,
      const std::string &_vertexName)
{
  Errors errors;

  if (_graph.map.count(_vertexName) != 1)
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
        "PoseRelativeToGraph unable to find unique frame with name [" +
        _vertexName + "] in graph."});
    return errors;
  }
  auto vertexId = _graph.map.at(_vertexName);

  auto incomingVertexEdges = FindSourceVertex(_graph.graph, vertexId, errors);

  if (!errors.empty())
  {
    return errors;
  }
  else if (!incomingVertexEdges.first.Valid())
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph unable to find path to source vertex "
        "when starting from vertex with name [" + _vertexName + "]."});
    return errors;
  }
  else if (incomingVertexEdges.first.Name() != _graph.sourceName)
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph frame with name [" + _vertexName + "] "
        "is disconnected; its source vertex has name [" +
        incomingVertexEdges.first.Name() +
        "], but its source name should be " + _graph.sourceName + "."});
    return errors;
  }

  ignition::math::Pose3d pose;
  for (auto const &edge : incomingVertexEdges.second)
  {
    pose = edge.Data() * pose;
  }

  if (errors.empty())
  {
    _pose = pose;
  }

  return errors;
}

/////////////////////////////////////////////////
Errors resolvePose(
    ignition::math::Pose3d &_pose,
    const PoseRelativeToGraph &_graph,
    const std::string &_frameName,
    const std::string &_resolveTo)
{
  Errors errors = resolvePoseRelativeToRoot(_pose, _graph, _frameName);

  ignition::math::Pose3d poseR;
  Errors errorsR = resolvePoseRelativeToRoot(poseR, _graph, _resolveTo);
  errors.insert(errors.end(), errorsR.begin(), errorsR.end());

  if (errors.empty())
  {
    _pose = poseR.Inverse() * _pose;
  }

  return errors;
}

/////////////////////////////////////////////////
Errors updateGraphPose(
    PoseRelativeToGraph &_graph,
    const std::string &_frameName,
    const ignition::math::Pose3d &_pose)
{
  Errors errors;

  if (_graph.map.count(_frameName) != 1)
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
        "PoseRelativeToGraph unable to find unique frame with name [" +
        _frameName+ "] in graph."});
    return errors;
  }
  auto vertexId = _graph.map.at(_frameName);
  auto incidentsTo = _graph.graph.IncidentsTo(vertexId);
  if (incidentsTo.size() == 1)
  {
    // There's no API to update the data of an edge, so we remove the edge and
    // insert a new one with the new pose.
    auto &edge = incidentsTo.begin()->second;
    auto tailVertexId = edge.get().Tail();
    auto headVertexId = edge.get().Head();
    _graph.graph.RemoveEdge(edge.get().Id());
    _graph.graph.AddEdge({tailVertexId, headVertexId}, _pose);
  }
  else if (incidentsTo.empty())
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph error: no incoming edge to "
        "vertex [" + _frameName + "]."});
  }
  else
  {
    // This is an error because there should be only one way to define the pose
    // of a frame. This would be like a frame having two //pose elements with
    // different //pose/@relative_to attributes.
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph error: multiple incoming edges to "
        "vertex [" + _frameName + "]."});
  }

  return errors;
}
}
}
