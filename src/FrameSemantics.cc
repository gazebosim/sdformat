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

#include "sdf/Element.hh"
#include "sdf/Error.hh"
#include "sdf/Frame.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"

#include "sdf/FrameSemantics.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {

// The following two functions were originally submitted to ign-math,
// but were not accepted as they were not generic enough.
// For now, they will be kept here.
// https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/333

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
  else if (!_model->Element()->HasUniqueChildNames())
  {
    errors.push_back({ErrorCode::DUPLICATE_NAME,
        "Non-unique names detected in XML children of model."});
    return errors;
  }
  else if (_model->LinkCount() < 1)
  {
    errors.push_back({ErrorCode::MODEL_WITHOUT_LINK,
                     "A model must have at least one link."});
    return errors;
  }

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
    auto linkId =
        _out.graph.AddVertex(link->Name(), sdf::FrameType::LINK).Id();
    _out.map[link->Name()] = linkId;

    // add edge from implicit model frame vertex to canonical link
    if (link == canonicalLink)
    {
      _out.graph.AddEdge({modelFrameId, linkId}, true);
    }
  }

  // add joint vertices and edges to child link
  for (uint64_t j = 0; j < _model->JointCount(); ++j)
  {
    auto joint = _model->JointByIndex(j);
    auto jointId =
        _out.graph.AddVertex(joint->Name(), sdf::FrameType::JOINT).Id();
    _out.map[joint->Name()] = jointId;

    auto childLink = _model->LinkByName(joint->ChildLinkName());
    if (nullptr == childLink)
    {
      errors.push_back({ErrorCode::JOINT_CHILD_LINK_INVALID,
        "Child link with name[" + joint->ChildLinkName() +
        "] specified by joint with name[" + joint->Name() +
        "] not found in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto childLinkId = _out.map.at(childLink->Name());
    _out.graph.AddEdge({jointId, childLinkId}, true);
  }

  // add frame vertices
  for (uint64_t f = 0; f < _model->FrameCount(); ++f)
  {
    auto frame = _model->FrameByIndex(f);
    auto frameId =
        _out.graph.AddVertex(frame->Name(), sdf::FrameType::FRAME).Id();
    _out.map[frame->Name()] = frameId;
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
          "] does not match a link, joint, or frame name "
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
  else if (!_world->Element()->HasUniqueChildNames())
  {
    errors.push_back({ErrorCode::DUPLICATE_NAME,
        "Non-unique names detected in XML children of world."});
    return errors;
  }

  // add model vertices
  for (uint64_t l = 0; l < _world->ModelCount(); ++l)
  {
    auto model = _world->ModelByIndex(l);
    auto modelId =
        _out.graph.AddVertex(model->Name(), sdf::FrameType::MODEL).Id();
    _out.map[model->Name()] = modelId;
  }

  // add frame vertices
  for (uint64_t f = 0; f < _world->FrameCount(); ++f)
  {
    auto frame = _world->FrameByIndex(f);
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

  if (_in.scopeName == "__model__" && sinkVertex.Data() != FrameType::LINK)
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "Graph has __model__ scope but sink vertex named [" +
        sinkVertex.Name() + "] does not have FrameType LINK "
        "when starting from vertex with name [" + _vertexName + "]."});
    return errors;
  }

  _attachedToBody = sinkVertex.Name();

  return errors;
}
}
}
