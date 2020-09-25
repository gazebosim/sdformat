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
#include <algorithm>
#include <string>

#include "sdf/Element.hh"
#include "sdf/Error.hh"
#include "sdf/Frame.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"

#include "FrameSemantics.hh"
#include "ScopedGraph.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {

void printGraph(const ScopedGraph<PoseRelativeToGraph> &_graph)
{
  std::cout << _graph.Graph() << std::endl;
}
void printGraph(const ScopedGraph<FrameAttachedToGraph> &_graph)
{
  std::cout << _graph.Graph() << std::endl;
}

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
template <typename T>
std::pair<const typename ScopedGraph<T>::Vertex &,
    std::vector<typename ScopedGraph<T>::Edge>>
FindSourceVertex(const ScopedGraph<T> &_graph,
    const ignition::math::graph::VertexId _id, Errors &_errors)
{
  using DirectedEdge = typename ScopedGraph<T>::Edge;
  using Vertex = typename ScopedGraph<T>::Vertex;
  using VertexId = ignition::math::graph::VertexId;
  using EdgesType = std::vector<DirectedEdge>;
  using PairType = std::pair<const Vertex &, EdgesType>;
  EdgesType edges;
  std::reference_wrapper<const Vertex> vertex(_graph.Graph().VertexFromId(_id));
  if (!vertex.get().Valid())
  {
    _errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
        "Unable to resolve pose, invalid vertex[" + std::to_string(_id) + "] "
        "in PoseRelativeToGraph."});
    return PairType(Vertex::NullVertex, EdgesType());
  }

  std::set<VertexId> visited;
  visited.insert(vertex.get().Id());

  auto incidentsTo = _graph.Graph().IncidentsTo(vertex);
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
    vertex = _graph.Graph().VertexFromId(edge.get().Vertices().first);
    edges.push_back(edge);
    if (visited.count(vertex.get().Id()))
    {
      _errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "PoseRelativeToGraph cycle detected, already visited vertex [" +
          vertex.get().Name() + "]."});
      return PairType(Vertex::NullVertex, EdgesType());
    }
    if (vertex.get().Id() == _graph.ScopeVertexId())
    {
      // This is the source.
      break;
    }
    visited.insert(vertex.get().Id());
    incidentsTo = _graph.Graph().IncidentsTo(vertex);
  }
  if (vertex.get().Id() != _graph.ScopeVertexId())
  {
    // Error, the root vertex is not the same as the the source
    return PairType(Vertex::NullVertex, EdgesType());
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
template<typename T>
std::pair<const typename ScopedGraph<T>::Vertex &,
    std::vector<typename ScopedGraph<T>::Edge>>
FindSinkVertex(
    const ScopedGraph<T> &_graph,
    const ignition::math::graph::VertexId _id,
    Errors &_errors)
{

  using DirectedEdge = typename ScopedGraph<T>::Edge;
  using Vertex = typename ScopedGraph<T>::Vertex;
  using VertexId = ignition::math::graph::VertexId;
  using EdgesType = std::vector<DirectedEdge>;
  using PairType = std::pair<const Vertex &, EdgesType>;
  EdgesType edges;
  std::reference_wrapper<const Vertex> vertex(_graph.Graph().VertexFromId(_id));
  if (!vertex.get().Valid())
  {
    _errors.push_back({ErrorCode::FRAME_ATTACHED_TO_INVALID,
        "Invalid vertex[" + std::to_string(_id) + "] "
        "in FrameAttachedToGraph."});
    return PairType(Vertex::NullVertex, EdgesType());
  }

  std::set<VertexId> visited;
  visited.insert(vertex.get().Id());

  auto incidentsFrom = _graph.Graph().IncidentsFrom(vertex);
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
    vertex = _graph.Graph().VertexFromId(edge.get().Vertices().second);
    edges.push_back(edge);
    if (visited.count(vertex.get().Id()))
    {
      _errors.push_back({ErrorCode::FRAME_ATTACHED_TO_CYCLE,
          "FrameAttachedToGraph cycle detected, already visited vertex [" +
          vertex.get().Name() + "]."});
      return PairType(Vertex::NullVertex, EdgesType());
    }
    visited.insert(vertex.get().Id());
    incidentsFrom = _graph.Graph().IncidentsFrom(vertex);
  }

  return PairType(vertex, edges);
}

/////////////////////////////////////////////////
Errors buildFrameAttachedToGraph(
            ScopedGraph<FrameAttachedToGraph> &_out, const Model *_model)
{
  Errors errors;

  if (_model->Static())
    return errors;

  // add implicit model frame vertex first
  const std::string scopeName = "__model__";
  _out.SetScopeName(scopeName);
  auto modelFrameId =
      _out.AddScopeVertex(_model->Name(), scopeName, sdf::FrameType::MODEL)
          .Id();

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
  else if (_model->LinkCount() < 1)
  {
    errors.push_back({ErrorCode::MODEL_WITHOUT_LINK,
                     "A model must have at least one link."});
    return errors;
  }

  // TODO (addisu) Handle finding the canonical link in a nested model.
  // TODO (addisu) Handle models with no links
  // identify canonical link
  const sdf::Link *canonicalLink = nullptr;
  if (_model->CanonicalLinkName().empty())
  {
    canonicalLink = _model->LinkByIndex(0);
  }
  else
  {
    canonicalLink = _model->LinkByName(_model->CanonicalLinkName());
  }
  if (nullptr == canonicalLink)
  {
    // return early
    errors.push_back({ErrorCode::MODEL_CANONICAL_LINK_INVALID,
      "canonical_link with name[" + _model->CanonicalLinkName() +
      "] not found in model with name[" + _model->Name() + "]."});
    return errors;
  }

  // add link vertices
  for (uint64_t l = 0; l < _model->LinkCount(); ++l)
  {
    auto link = _model->LinkByIndex(l);
    if (_out.Count(link->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Link with non-unique name [" + link->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    auto linkId =
        _out.AddVertex(link->Name(), sdf::FrameType::LINK).Id();

    // add edge from implicit model frame vertex to canonical link
    if (link == canonicalLink)
    {
      _out.AddEdge({modelFrameId, linkId}, true);
    }
  }

  // add joint vertices
  for (uint64_t j = 0; j < _model->JointCount(); ++j)
  {
    auto joint = _model->JointByIndex(j);
    if (_out.Count(joint->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Joint with non-unique name [" + joint->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    _out.AddVertex(joint->Name(), sdf::FrameType::JOINT).Id();
  }

  // add frame vertices
  for (uint64_t f = 0; f < _model->FrameCount(); ++f)
  {
    auto frame = _model->FrameByIndex(f);
    if (_out.Count(frame->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Frame with non-unique name [" + frame->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    _out.AddVertex(frame->Name(), sdf::FrameType::FRAME).Id();
  }

  // add nested model vertices
  for (uint64_t m = 0; m < _model->ModelCount(); ++m)
  {
    auto nestedModel = _model->ModelByIndex(m);
    if (_out.Count(nestedModel->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Nested model with non-unique name [" + nestedModel->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    _out.AddVertex(nestedModel->Name(), sdf::FrameType::MODEL).Id();
    ScopedGraph<sdf::FrameAttachedToGraph>childGraph = _out;
    buildFrameAttachedToGraph(childGraph, nestedModel);
  }

  // add edges from joint to child frames
  for (uint64_t j = 0; j < _model->JointCount(); ++j)
  {
    auto joint = _model->JointByIndex(j);
    auto jointId = _out.VertexIdByName(joint->Name());
    auto childFrameName = joint->ChildLinkName();
    if (_out.Count(childFrameName) != 1)
    {
      errors.push_back({ErrorCode::JOINT_CHILD_LINK_INVALID,
        "Child frame with name[" + childFrameName +
        "] specified by joint with name[" + joint->Name() +
        "] not found in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto childFrameId = _out.VertexIdByName(childFrameName);
    _out.AddEdge({jointId, childFrameId}, true);
  }

  // add model edges to their corresponding vertices in the model scope
  for (uint64_t m = 0; m < _model->ModelCount(); ++m)
  {
    auto model = _model->ModelByIndex(m);
    auto modelId = _out.VertexIdByName(model->Name());
    auto modelIdChildScope = _out.VertexIdByName(model->Name() + "::__model__");
    if (modelIdChildScope != ignition::math::graph::kNullId)
    {
      _out.AddEdge({modelId, modelIdChildScope}, true);
    }
  }

  // add frame edges
  for (uint64_t f = 0; f < _model->FrameCount(); ++f)
  {
    auto frame = _model->FrameByIndex(f);
    auto frameId = _out.VertexIdByName(frame->Name());
    // look for vertex in graph that matches attached_to value
    std::string attachedTo = frame->AttachedTo();
    if (attachedTo.empty())
    {
      // if the attached-to name is empty, use the scope name
      attachedTo = scopeName;
    }
    if (_out.Count(attachedTo) != 1)
    {
      errors.push_back({ErrorCode::FRAME_ATTACHED_TO_INVALID,
          "attached_to name[" + attachedTo +
          "] specified by frame with name[" + frame->Name() +
          "] does not match a nested model, link, joint, or frame name "
          "in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto attachedToId = _out.VertexIdByName(attachedTo);
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
    _out.AddEdge({frameId, attachedToId}, edgeData);
  }

  // std::cout << "Model FrameAttachedToGraph\n" << _out.Graph() << std::endl;
  return errors;
}

/////////////////////////////////////////////////
Errors buildFrameAttachedToGraph(
            ScopedGraph<FrameAttachedToGraph> &_out, const World *_world)
{
  Errors errors;

  // add implicit world frame vertex first
  const std::string scopeName = "world";
  _out.SetScopeName(scopeName);
  _out.AddScopeVertex("", scopeName, sdf::FrameType::WORLD).Id();

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
    if (_out.Count(model->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Model with non-unique name [" + model->Name() +
          "] detected in world with name [" + _world->Name() +
          "]."});
      continue;
    }
    _out.AddVertex(model->Name(), sdf::FrameType::MODEL).Id();
    ScopedGraph<sdf::FrameAttachedToGraph>childGraph = _out;
    buildFrameAttachedToGraph(childGraph, model);
  }

  // add frame vertices
  for (uint64_t f = 0; f < _world->FrameCount(); ++f)
  {
    auto frame = _world->FrameByIndex(f);
    if (_out.Count(frame->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Frame with non-unique name [" + frame->Name() +
          "] detected in world with name [" + _world->Name() +
          "]."});
      continue;
    }
    _out.AddVertex(frame->Name(), sdf::FrameType::FRAME).Id();
  }

  // add model edges to their corresponding vertices in the model scope
  for (uint64_t m = 0; m < _world->ModelCount(); ++m)
  {
    auto model = _world->ModelByIndex(m);
    auto modelId = _out.VertexIdByName(model->Name());
    auto modelIdChildScope = _out.VertexIdByName(model->Name() + "::__model__");
    if (modelIdChildScope != ignition::math::graph::kNullId)
    {
      _out.AddEdge({modelId, modelIdChildScope}, true);
    }
  }

  // add frame edges
  for (uint64_t f = 0; f < _world->FrameCount(); ++f)
  {
    auto frame = _world->FrameByIndex(f);
    auto frameId = _out.VertexIdByName(frame->Name());
    // look for vertex in graph that matches attached_to value
    std::string attachedTo = frame->AttachedTo();
    if (attachedTo.empty())
    {
      // if the attached-to name is empty, use the scope name
      attachedTo = scopeName;
      if (_out.Count(scopeName) != 1)
      {
        errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                         "FrameAttachedToGraph error: scope frame[" +
                         scopeName + "] not found in map."});
        continue;
      }
    }
    if (_out.Count(attachedTo) != 1)
    {
      errors.push_back({ErrorCode::FRAME_ATTACHED_TO_INVALID,
          "attached_to name[" + attachedTo +
          "] specified by frame with name[" + frame->Name() +
          "] does not match a model or frame name "
          "in world with name[" + _world->Name() + "]."});
      continue;
    }
    auto attachedToId = _out.VertexIdByName(attachedTo);
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
    _out.AddEdge({frameId, attachedToId}, edgeData);
  }

  // std::cout << "World FrameAttachedToGraph\n" << _out.Graph() << std::endl;
  return errors;
}

/////////////////////////////////////////////////
Errors buildPoseRelativeToGraph(
            ScopedGraph<PoseRelativeToGraph> &_out, const Model *_model)
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

  // add the model frame vertex first
  const std::string scopeName = "__model__";
  _out.SetScopeName(scopeName);
  auto modelFrameId =
      _out.AddScopeVertex(_model->Name(), scopeName, sdf::FrameType::MODEL).Id();

  // add link vertices and default edge if relative_to is empty
  for (uint64_t l = 0; l < _model->LinkCount(); ++l)
  {
    auto link = _model->LinkByIndex(l);
    if (_out.Count(link->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Link with non-unique name [" + link->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    auto linkId =
        _out.AddVertex(link->Name(), sdf::FrameType::LINK).Id();

    if (link->PoseRelativeTo().empty())
    {
      // relative_to is empty, so add edge from implicit model frame to link
      _out.AddEdge({modelFrameId, linkId}, link->RawPose());
    }
  }

  // add joint vertices
  for (uint64_t j = 0; j < _model->JointCount(); ++j)
  {
    auto joint = _model->JointByIndex(j);
    if (_out.Count(joint->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Joint with non-unique name [" + joint->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    _out.AddVertex(joint->Name(), sdf::FrameType::JOINT).Id();
  }

  // add frame vertices and default edge if both
  // relative_to and attached_to are empty
  for (uint64_t f = 0; f < _model->FrameCount(); ++f)
  {
    auto frame = _model->FrameByIndex(f);
    if (_out.Count(frame->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Frame with non-unique name [" + frame->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    auto frameId =
        _out.AddVertex(frame->Name(), sdf::FrameType::FRAME).Id();

    if (frame->PoseRelativeTo().empty() && frame->AttachedTo().empty())
    {
      // add edge from implicit model frame to frame
      _out.AddEdge({modelFrameId, frameId}, frame->RawPose());
    }
  }

  // add nested model vertices and default edge if relative_to is empty
  for (uint64_t m = 0; m < _model->ModelCount(); ++m)
  {
    auto nestedModel = _model->ModelByIndex(m);
    if (_out.Count(nestedModel->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Nested model with non-unique name [" + nestedModel->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    auto nestedModelId =
        _out.AddVertex(nestedModel->Name(), sdf::FrameType::MODEL).Id();

    ScopedGraph<sdf::PoseRelativeToGraph> childGraph = _out;
    buildPoseRelativeToGraph(childGraph , nestedModel);

    if (nestedModel->PoseRelativeTo().empty())
    {
      // relative_to is empty, so add edge from implicit model frame
      // to nestedModel
      _out.AddEdge({modelFrameId, nestedModelId}, nestedModel->RawPose());
    }
    // Add an aliasing edge from the nestedModel vertex to the
    // corresponding vertex in the PoseRelativeTo graph of the nested model,
    // i.e, to the <model_name>::__model__ vertex
    auto scopedNestedModelId =
        _out.VertexIdByName(nestedModel->Name() + "::__model__");
    if (scopedNestedModelId == ignition::math::graph::kNullId)
    {
      // TODO (addisu) ERROR
    }
    else
    {
      _out.AddEdge({nestedModelId, scopedNestedModelId}, {{}, true});
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

    auto linkId = _out.VertexIdByName(link->Name());

    // look for vertex in graph that matches relative_to value
    if (_out.Count(relativeTo) != 1)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
          "relative_to name[" + relativeTo +
          "] specified by link with name[" + link->Name() +
          "] does not match a nested model, link, joint, or frame name "
          "in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto relativeToId = _out.VertexIdByName(relativeTo);
    if (link->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "relative_to name[" + relativeTo +
          "] is identical to link name[" + link->Name() +
          "], causing a graph cycle "
          "in model with name[" + _model->Name() + "]."});
    }
    _out.AddEdge({relativeToId, linkId}, link->RawPose());
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

    auto jointId = _out.VertexIdByName(joint->Name());

    // look for vertex in graph that matches relative_to value
    if (_out.Count(relativeTo) != 1)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
          "relative_to name[" + relativeTo +
          "] specified by joint with name[" + joint->Name() +
          "] does not match a nested model, link, joint, or frame name "
          "in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto relativeToId = _out.VertexIdByName(relativeTo);
    if (joint->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "relative_to name[" + relativeTo +
          "] is identical to joint name[" + joint->Name() +
          "], causing a graph cycle "
          "in model with name[" + _model->Name() + "]."});
    }
    _out.AddEdge({relativeToId, jointId}, joint->RawPose());
  }

  for (uint64_t f = 0; f < _model->FrameCount(); ++f)
  {
    auto frame = _model->FrameByIndex(f);

    // check if we've already added a default edge
    if (frame->PoseRelativeTo().empty() && frame->AttachedTo().empty())
    {
      continue;
    }

    auto frameId = _out.VertexIdByName(frame->Name());
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
    if (_out.Count(relativeTo) != 1)
    {
      errors.push_back({errorCode,
          typeForErrorMsg + " name[" + relativeTo +
          "] specified by frame with name[" + frame->Name() +
          "] does not match a nested model, link, joint, or frame name "
          "in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto relativeToId = _out.VertexIdByName(relativeTo);
    if (frame->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "relative_to name[" + relativeTo +
          "] is identical to frame name[" + frame->Name() +
          "], causing a graph cycle "
          "in model with name[" + _model->Name() + "]."});
    }
    _out.AddEdge({relativeToId, frameId}, frame->RawPose());
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

    auto nestedModelId = _out.VertexIdByName(nestedModel->Name());

    // look for vertex in graph that matches relative_to value
    if (_out.Count(relativeTo) != 1)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
          "relative_to name[" + relativeTo +
          "] specified by nested model with name[" + nestedModel->Name() +
          "] does not match a nested model, link, joint, or frame name "
          "in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto relativeToId = _out.VertexIdByName(relativeTo);
    if (nestedModel->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "relative_to name[" + relativeTo +
          "] is identical to nested model name[" + nestedModel->Name() +
          "], causing a graph cycle "
          "in model with name[" + _model->Name() + "]."});
    }
    _out.AddEdge({relativeToId, nestedModelId}, nestedModel->RawPose());
  }

  // std::cout << "Model PoseRelativeToGraph\n" << _out.Graph() << std::endl;
  return errors;
}

/////////////////////////////////////////////////
Errors buildPoseRelativeToGraph(
    ScopedGraph<PoseRelativeToGraph> &_out, const World *_world)
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
  _out.SetScopeName(sourceName);
  auto worldFrameId =
      _out.AddScopeVertex("", sourceName, sdf::FrameType::WORLD).Id();

  // add model vertices and default edge if relative_to is empty
  for (uint64_t m = 0; m < _world->ModelCount(); ++m)
  {
    auto model = _world->ModelByIndex(m);
    if (_out.Count(model->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Model with non-unique name [" + model->Name() +
          "] detected in world with name [" + _world->Name() +
          "]."});
      continue;
    }
    auto modelId =
        _out.AddVertex(model->Name(), sdf::FrameType::MODEL).Id();

    ScopedGraph<sdf::PoseRelativeToGraph> childGraph = _out;
    buildPoseRelativeToGraph(childGraph , model);

    if (model->PoseRelativeTo().empty())
    {
      // relative_to is empty, so add edge from implicit world frame to model
      _out.AddEdge({worldFrameId, modelId}, model->RawPose());
    }
    // Add a non-constraint edge from the model vertex to the
    // corresponding vertex in the PoseRelativeTo graph of the child model,
    // i.e, to the <model_name>::__model__ vertex
    auto scopedModelId =
        _out.VertexIdByName(model->Name() + "::__model__");
    if (scopedModelId == ignition::math::graph::kNullId)
    {
      // TODO (addisu) ERROR
    }
    else
    {
      _out.AddEdge({modelId, scopedModelId}, {{}, false});
    }
  }

  // add frame vertices and default edge if both
  // relative_to and attached_to are empty
  for (uint64_t f = 0; f < _world->FrameCount(); ++f)
  {
    auto frame = _world->FrameByIndex(f);
    if (_out.Count(frame->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Frame with non-unique name [" + frame->Name() +
          "] detected in world with name [" + _world->Name() +
          "]."});
      continue;
    }
    auto frameId =
        _out.AddVertex(frame->Name(), sdf::FrameType::FRAME).Id();

    if (frame->PoseRelativeTo().empty() && frame->AttachedTo().empty())
    {
      // add edge from implicit world frame to frame
      _out.AddEdge({worldFrameId, frameId}, frame->RawPose());
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

    auto modelId = _out.VertexIdByName(model->Name());

    // look for vertex in graph that matches relative_to value
    if (_out.Count(relativeTo) != 1)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
          "relative_to name[" + relativeTo +
          "] specified by model with name[" + model->Name() +
          "] does not match a model or frame name "
          "in world with name[" + _world->Name() + "]."});
      continue;
    }
    auto relativeToId = _out.VertexIdByName(relativeTo);
    if (model->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "relative_to name[" + relativeTo +
          "] is identical to model name[" + model->Name() +
          "], causing a graph cycle "
          "in world with name[" + _world->Name() + "]."});
    }
    _out.AddEdge({relativeToId, modelId}, model->RawPose());
  }

  for (uint64_t f = 0; f < _world->FrameCount(); ++f)
  {
    auto frame = _world->FrameByIndex(f);

    // check if we've already added a default edge
    if (frame->PoseRelativeTo().empty() && frame->AttachedTo().empty())
    {
      continue;
    }

    auto frameId = _out.VertexIdByName(frame->Name());
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
    if (_out.Count(relativeTo) != 1)
    {
      errors.push_back({errorCode,
          typeForErrorMsg + " name[" + relativeTo +
          "] specified by frame with name[" + frame->Name() +
          "] does not match a model or frame name "
          "in world with name[" + _world->Name() + "]."});
      continue;
    }
    auto relativeToId = _out.VertexIdByName(relativeTo);
    if (frame->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "relative_to name[" + relativeTo +
          "] is identical to frame name[" + frame->Name() +
          "], causing a graph cycle "
          "in world with name[" + _world->Name() + "]."});
    }
    _out.AddEdge({relativeToId, frameId}, frame->RawPose());
  }

  // std::cout << "World PoseRelativeToGraph\n" << _out.Graph() << std::endl;
  return errors;
}

/////////////////////////////////////////////////
Errors validateFrameAttachedToGraph(const ScopedGraph<FrameAttachedToGraph> &_in)
{
  Errors errors;

  // Expect scopeName to be either "__model__" or "world"
  if (_in.ScopeName() != "__model__" && _in.ScopeName() != "world")
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "FrameAttachedToGraph error: scope frame[" + _in.ScopeName() + "] "
        " does not match __model__ or world."});
    return errors;
  }

  // Expect one vertex with name "__model__" and FrameType MODEL
  // or with name "world" and FrameType WORLD
  auto scopeVertex = _in.ScopeVertex();
  if (!scopeVertex.Valid())
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                     "FrameAttachedToGraph error: scope frame[" +
                     _in.ScopeName() + "] not found in graph."});
    return errors;
  }

  sdf::FrameType scopeFrameType = scopeVertex.Data();
  if (_in.ScopeName() == "__model__" && scopeFrameType != sdf::FrameType::MODEL)
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "FrameAttachedToGraph error, "
        "scope vertex with name __model__ should have FrameType MODEL."});
    return errors;
  }
  else if (_in.ScopeName() == "world" &&
           scopeFrameType != sdf::FrameType::WORLD)
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "FrameAttachedToGraph error, "
        "scope vertex with name world should have FrameType WORLD."});
    return errors;
  }

  // Check number of outgoing edges for each vertex
  auto vertices = _in.Vertices();
  for (auto vertexPair : vertices)
  {
    const std::string vertexName = _in.VertexName(vertexPair.second.get());
    // Vertex names should not be empty
    if (vertexName.empty())
    {
      errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
          "FrameAttachedToGraph error, "
          "vertex with empty name detected."});
    }

    auto outDegree = _in.Graph().OutDegree(vertexPair.first);
    if (outDegree > 1)
    {
      errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
          "FrameAttachedToGraph error, "
          "too many outgoing edges at a vertex with name [" +
          vertexName + "]."});
    }
    else if (sdf::FrameType::MODEL == scopeFrameType)
    {
      switch (vertexPair.second.get().Data())
      {
        case sdf::FrameType::WORLD:
          errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
              "FrameAttachedToGraph error, "
              "vertex with name [" + vertexName + "]" +
              "should not have type WORLD in MODEL attached_to graph."});
          break;
        case sdf::FrameType::LINK:
          if (outDegree != 0)
          {
            errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                "FrameAttachedToGraph error, "
                "LINK vertex with name [" +
                vertexName +
                "] should have no outgoing edges "
                "in MODEL attached_to graph."});
          }
          break;
        default:
          if (outDegree == 0)
          {
            errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                "FrameAttachedToGraph error, "
                "Non-LINK vertex with name [" +
                vertexName +
                "] is disconnected; it should have 1 outgoing edge " +
                "in MODEL attached_to graph."});
          }
          else if (outDegree >= 2)
          {
            errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                "FrameAttachedToGraph error, "
                "Non-LINK vertex with name [" +
                vertexName +
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
        case sdf::FrameType::WORLD:
          if (outDegree != 0)
          {
            errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                "FrameAttachedToGraph error, "
                "WORLD vertices should have no outgoing edges "
                "in WORLD attached_to graph."});
          }
          break;
        case sdf::FrameType::LINK:
          if (outDegree != 0)
          {
            errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                "FrameAttachedToGraph error, "
                "LINK vertex with name [" +
                vertexName +
                "] should have no outgoing edges "
                "in WORLD attached_to graph."});
          }
          break;
        default:
          if (outDegree != 1)
          {
            errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                "FrameAttachedToGraph error, "
                "Non-LINK vertex with name [" +
                vertexName +
                "] has " + std::to_string(outDegree) +
                " outgoing edges; it should only have 1 "
                "outgoing edge in WORLD attached_to graph."});
          }
          break;
      }
    }
  }

  // check graph for cycles by finding sink from each vertex
  for (auto const &name : _in.VertexNames())
  {
    std::string resolvedBody;
    Errors e = resolveFrameAttachedToBody(resolvedBody, _in, name);
    errors.insert(errors.end(), e.begin(), e.end());
  }

  return errors;
}

/////////////////////////////////////////////////
Errors validatePoseRelativeToGraph(
    const ScopedGraph<PoseRelativeToGraph> &_in)
{
  Errors errors;

  // Expect scopeName to be either "__model__" or "world"
  if (_in.ScopeName() != "__model__" && _in.ScopeName() != "world")
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph error: source vertex name " + _in.ScopeName() +
        " does not match __model__ or world."});
    return errors;
  }

  // Expect one vertex with name "__model__" and FrameType MODEL
  // or with name "world" and FrameType WORLD
  auto sourceVertices = _in.Vertices(_in.ScopeName());
  if (sourceVertices.empty())
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
                     "PoseRelativeToGraph error: source frame[" +
                     _in.ScopeName() + "] not found in graph."});
    return errors;
  }
  else if (sourceVertices.size() > 1)
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph error, "
        "more than one vertex with source name " + _in.ScopeName()});
    return errors;
  }

  auto sourceVertex = sourceVertices.begin()->second.get();
  sdf::FrameType sourceFrameType = sourceVertex.Data();
  if (_in.ScopeName() == "__model__" && sourceFrameType != sdf::FrameType::MODEL)
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph error, "
        "source vertex with name __model__ should have FrameType MODEL."});
    return errors;
  }
  else if (_in.ScopeName() == "world" &&
           sourceFrameType != sdf::FrameType::WORLD)
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph error, "
        "source vertex with name world should have FrameType WORLD."});
    return errors;
  }

  // Check number of incoming edges for each vertex
  auto vertices = _in.Vertices();
  for (auto vertexPair : vertices)
  {
    const std::string vertexName = _in.VertexName(vertexPair.second.get());
    // Vertex names should not be empty
    if (vertexName.empty())
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
          "PoseRelativeToGraph error, "
          "vertex with empty name detected."});
    }

    auto incomingEdges = _in.Graph().IncidentsTo(vertexPair.first);
    std::size_t inDegree = std::count_if(incomingEdges.begin(),
        incomingEdges.end(),
        [](const auto &_edge) { return !_edge.second.get().Data().aliasing; });

    if (inDegree > 1)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
          "PoseRelativeToGraph error, "
          "too many incoming edges at a vertex with name [" +
          vertexName + "]."});
    }
    else if (sdf::FrameType::MODEL == sourceFrameType)
    {
      switch (vertexPair.second.get().Data())
      {
        case sdf::FrameType::WORLD:
          errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
              "PoseRelativeToGraph error, "
              "vertex with name [" + vertexName + "]" +
              "should not have type WORLD in MODEL relative_to graph."});
          break;
        case sdf::FrameType::MODEL:
          // TODO: What we have to check here is that if this is the scope
          // vertex any incoming edge is from outside this scope.
          if (sdf::endswith(vertexName, "__model__"))
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
                vertexName +
                "] is disconnected; it should have 1 incoming edge " +
                "in MODEL relative_to graph."});
          }
          else if (inDegree >= 2)
          {
            errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
                "PoseRelativeToGraph error, "
                "Non-MODEL vertex with name [" +
                vertexName +
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
                vertexName +
                "] is disconnected; it should have 1 incoming edge " +
                "in WORLD relative_to graph."});
          }
          else if (inDegree >= 2)
          {
            errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
                "PoseRelativeToGraph error, "
                "MODEL / FRAME vertex with name [" +
                vertexName +
                "] has " + std::to_string(inDegree) +
                " incoming edges; it should only have 1 "
                "incoming edge in WORLD relative_to graph."});
          }
          break;
      }
    }
  }

  // check graph for cycles by resolving pose of each vertex relative to root
  for (auto const &name : _in.VertexNames())
  {
    ignition::math::Pose3d pose;
    Errors e = resolvePoseRelativeToRoot(pose, _in, name);
    errors.insert(errors.end(), e.begin(), e.end());
  }

  return errors;
}

/////////////////////////////////////////////////
Errors resolveFrameAttachedToBody(
    std::string &_attachedToBody,
    const ScopedGraph<FrameAttachedToGraph> &_in,
    const std::string &_vertexName)
{
  Errors errors;

  if (_in.ScopeName() != "__model__" && _in.ScopeName() != "world")
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "FrameAttachedToGraph error: scope frame[" + _in.ScopeName() + "] "
        " does not match __model__ or world."});
    return errors;
  }

  if (_in.Count(_vertexName) != 1)
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_INVALID,
        "FrameAttachedToGraph unable to find unique frame with name [" +
        _vertexName + "] in graph."});
    return errors;
  }
  auto vertexId = _in.VertexIdByName(_vertexName);

  auto sinkVertexEdges = FindSinkVertex(_in, vertexId, errors);
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

  if (_in.ScopeName() == "world" &&
      !(sinkVertex.Data() == FrameType::WORLD ||
        sinkVertex.Data() == FrameType::LINK))
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "Graph has world scope but sink vertex named [" +
        sinkVertex.Name() + "] does not have FrameType WORLD or LINK "
        "when starting from vertex with name [" + _vertexName + "]."});
    return errors;
  }

  if (_in.ScopeName() == "__model__")
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

  _attachedToBody = _in.VertexName(sinkVertex);

  return errors;
}

/////////////////////////////////////////////////
Errors resolvePoseRelativeToRoot(
      ignition::math::Pose3d &_pose,
      const ScopedGraph<PoseRelativeToGraph> &_graph,
      const std::string &_vertexName)
{
  Errors errors;

  if (_graph.Count(_vertexName) != 1)
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
        "PoseRelativeToGraph unable to find unique frame with name [" +
        _vertexName + "] in graph."});
    return errors;
  }
  auto vertexId = _graph.VertexIdByName(_vertexName);

  auto incomingVertexEdges = FindSourceVertex(_graph, vertexId, errors);

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
  else if (incomingVertexEdges.first.Id() != _graph.ScopeVertex().Id())
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph frame with name [" + _vertexName + "] "
        "is disconnected; its source vertex has name [" +
        incomingVertexEdges.first.Name() +
        "], but its source name should be " + _graph.ScopeName() + "."});
    return errors;
  }

  ignition::math::Pose3d pose;
  for (auto const &edge : incomingVertexEdges.second)
  {
    pose = edge.Data().pose * pose;
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
    const ScopedGraph<PoseRelativeToGraph> &_graph,
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
    ScopedGraph<PoseRelativeToGraph> &_graph,
    const std::string &_frameName,
    const ignition::math::Pose3d &_pose)
{
  Errors errors;

  if (_graph.Count(_frameName) != 1)
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
        "PoseRelativeToGraph unable to find unique frame with name [" +
        _frameName+ "] in graph."});
    return errors;
  }
  auto vertexId = _graph.VertexIdByName(_frameName);
  auto incidentsTo = _graph.Graph().IncidentsTo(vertexId);
  std::vector<std::size_t> toRemove;
  for (const auto &[id, edge] : incidentsTo)
  {
    if (edge.get().Data().aliasing)
    {
      toRemove.push_back(id);
    }
  }
  for (const auto id : toRemove)
  {
    incidentsTo.erase(id);
  }

  if (incidentsTo.size() == 1)
  {
    _graph.UpdateEdge(incidentsTo.begin()->second, _pose);
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
