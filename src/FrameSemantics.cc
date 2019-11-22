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
    _errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Input vertex [" + std::to_string(_id) + "] is not valid."});
    return PairType(Vertex::NullVertex, EdgesType());
  }

  std::set<VertexId> visited;
  visited.insert(vertex.get().Id());

  auto incidentsTo = _graph.IncidentsTo(vertex);
  while (!incidentsTo.empty())
  {
    if (incidentsTo.size() != 1)
    {
      _errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Multiple vertices incident to current vertex [" +
          std::to_string(vertex.get().Id()) + "]."});
      return PairType(Vertex::NullVertex, EdgesType());
    }
    auto const &edge = incidentsTo.begin()->second;
    vertex = _graph.VertexFromId(edge.get().Vertices().first);
    edges.push_back(edge);
    if (visited.count(vertex.get().Id()))
    {
      _errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Graph cycle detected, already visited vertex [" +
          std::to_string(vertex.get().Id()) + "]."});
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
/// vector if a cycle or vertex with multiple incoming edges are detected.
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
    _errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Input vertex [" + std::to_string(_id) + "] is not valid."});
    return PairType(Vertex::NullVertex, EdgesType());
  }

  std::set<VertexId> visited;
  visited.insert(vertex.get().Id());

  auto incidentsFrom = _graph.IncidentsFrom(vertex);
  while (!incidentsFrom.empty())
  {
    if (incidentsFrom.size() != 1)
    {
      _errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Multiple vertices incident from current vertex [" +
          std::to_string(vertex.get().Id()) + "]."});
      return PairType(Vertex::NullVertex, EdgesType());
    }
    auto const &edge = incidentsFrom.begin()->second;
    vertex = _graph.VertexFromId(edge.get().Vertices().second);
    edges.push_back(edge);
    if (visited.count(vertex.get().Id()))
    {
      _errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Graph cycle detected, already visited vertex [" +
          std::to_string(vertex.get().Id()) + "]."});
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
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Non-unique names detected in XML children of model."});
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
    errors.push_back({ErrorCode::ELEMENT_INVALID,
                     "Model's canonical link is invalid."});
    return errors;
  }

  // add implicit model frame vertex first
  const std::string scopeName = "__model__";
  _out.scopeName = scopeName;
  auto modelFrameId =
      _out.graph.AddVertex(scopeName, sdf::FrameType::MODEL).Id();
  _out.map[scopeName] = modelFrameId;

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
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Joint's child link is invalid."});
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
        errors.push_back({ErrorCode::ELEMENT_INVALID,
                         scopeName + "not found in map."});
        continue;
      }
    }
    if (_out.map.count(attachedTo) != 1)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Invalid attached_to value in Frame."});
      continue;
    }
    auto attachedToId = _out.map[attachedTo];
    bool edgeData = true;
    if (frame->Name() == frame->AttachedTo())
    {
      // set edgeData to false if attaches to itself, since this is invalid
      edgeData = false;
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Frame attached_to itself causes a cycle."});
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
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Non-unique names detected in XML children of world."});
    return errors;
  }

  // add implicit world frame vertex first
  const std::string scopeName = "world";
  _out.scopeName = scopeName;
  auto worldFrameId =
      _out.graph.AddVertex(scopeName, sdf::FrameType::WORLD).Id();
  _out.map[scopeName] = worldFrameId;

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
        errors.push_back({ErrorCode::ELEMENT_INVALID,
                         scopeName + "not found in map."});
        continue;
      }
    }
    if (_out.map.count(attachedTo) != 1)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Invalid attached_to value in Frame."});
      continue;
    }
    auto attachedToId = _out.map[attachedTo];
    bool edgeData = true;
    if (frame->Name() == frame->AttachedTo())
    {
      // set edgeData to false if attaches to itself, since this is invalid
      edgeData = false;
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Frame attached_to itself causes a cycle."});
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
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Scope vertex name " + _in.scopeName +
        " does not match __model__ or world."});
    return errors;
  }

  // Expect one vertex with name "__model__" and FrameType MODEL
  // or with name "world" and FrameType WORLD
  auto scopeVertices = _in.graph.Vertices(_in.scopeName);
  if (scopeVertices.empty())
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Missing scope vertex with name " + _in.scopeName});
    return errors;
  }
  else if (scopeVertices.size() > 1)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "More than one vertex with scope name " + _in.scopeName});
    return errors;
  }

  auto scopeVertex = scopeVertices.begin()->second.get();
  sdf::FrameType scopeFrameType = scopeVertex.Data();
  if (_in.scopeName == "__model__" && scopeFrameType != sdf::FrameType::MODEL)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Scope vertex with name __model__ should have FrameType MODEL."});
    return errors;
  }
  else if (_in.scopeName == "world" &&
           scopeFrameType != sdf::FrameType::WORLD)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Scope vertex with name world should have FrameType WORLD."});
    return errors;
  }

  // Check number of outgoing edges for each vertex
  auto vertices = _in.graph.Vertices();
  for (auto vertexPair : vertices)
  {
    // Vertex names should not be empty
    if (vertexPair.second.get().Name().empty())
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Vertex with empty name detected."});
    }

    auto outDegree = _in.graph.OutDegree(vertexPair.first);
    if (outDegree > 1)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Too many outgoing edges at a vertex with name [" +
          vertexPair.second.get().Name() + "]."});
    }
    else if (sdf::FrameType::MODEL == scopeFrameType)
    {
      switch (vertexPair.second.get().Data())
      {
        case sdf::FrameType::WORLD:
          errors.push_back({ErrorCode::ELEMENT_INVALID,
              "Vertex with name [" + vertexPair.second.get().Name() + "]" +
              "should not have type WORLD in MODEL attached_to graph."});
          break;
        case sdf::FrameType::LINK:
          if (outDegree != 0)
          {
            errors.push_back({ErrorCode::ELEMENT_INVALID,
                "LINK vertex with name [" +
                vertexPair.second.get().Name() +
                "] should have no outgoing edges "
                "in MODEL attached_to graph."});
          }
          break;
        default:
          if (outDegree != 1)
          {
            errors.push_back({ErrorCode::ELEMENT_INVALID,
                "Non-LINK vertex with name [" +
                vertexPair.second.get().Name() +
                "] should have 1 outgoing edge " +
                "in MODEL attached_to graph."});
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
          errors.push_back({ErrorCode::ELEMENT_INVALID,
              "No JOINT or LINK vertex should be in WORLD attached_to graph."});
          break;
        case sdf::FrameType::MODEL:
        case sdf::FrameType::WORLD:
          if (outDegree != 0)
          {
            errors.push_back({ErrorCode::ELEMENT_INVALID,
                "MODEL and WORLD vertices should have no outgoing edges "
                "in WORLD attached_to graph."});
          }
          break;
        default:
          if (outDegree != 1)
          {
            errors.push_back({ErrorCode::ELEMENT_INVALID,
                "FRAME vertices in WORLD attached_to graph should have "
                "1 outgoing edge."});
          }
          break;
      }
    }
  }

  // TODO: check graph for cycles

  return errors;
}

/////////////////////////////////////////////////
Errors resolveFrameAttachedToBody(const FrameAttachedToGraph &_in,
    const std::string &_vertexName, std::string &_attachedToBody)
{
  Errors errors;

  if (_in.scopeName != "__model__" && _in.scopeName != "world")
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Graph has invalid scope name [" + _in.scopeName + "],"
        " which should be [__model__] or [world]."});
    return errors;
  }

  if (_in.map.count(_vertexName) != 1)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Unique vertex with name [" + _vertexName + "] not found in graph."});
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
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Sink vertex not found in graph when starting from vertex with "
        "name [" + _vertexName + "]."});
    return errors;
  }

  if (_in.scopeName == "world" &&
      !(sinkVertex.Data() == FrameType::WORLD ||
        sinkVertex.Data() == FrameType::MODEL))
  {
    // errors.push_back({ErrorCode::ELEMENT_INVALID,
    //     "Graph has world scope but sink vertex has FrameType [" +
    //     std::to_string(sinkVertex.Data()) + "], when it should be either "
    //     "WORLD [" + FrameType::WORLD + "] or MODEL [" FrameType::MODEL + "] "
    //     "when starting from vertex with name [" + _vertexName + "]."});
    return errors;
  }

  if (_in.scopeName == "__model__" && sinkVertex.Data() != FrameType::LINK)
  {
    // errors.push_back({ErrorCode::ELEMENT_INVALID,
    //     "Graph has __model__ scope but sink vertex has FrameType ["
    //     sinkVertex.Data() + "], when it should be LINK "
    //     "[" + FrameType::LINK + "] "
    //     "when starting from vertex with name [" + _vertexName + "]."});
    return errors;
  }

  _attachedToBody = sinkVertex.Name();

  return errors;
}
}
}
