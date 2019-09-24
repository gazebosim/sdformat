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
Errors buildKinematicGraph(
            KinematicGraph &_out, const Model *_model)
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

  // add link vertices
  for (uint64_t l = 0; l < _model->LinkCount(); ++l)
  {
    auto link = _model->LinkByIndex(l);
    auto linkId = _out.graph.AddVertex(link->Name(), link).Id();
    _out.map[link->Name()] = linkId;
  }

  // add edge for each joint from parent link to child link
  for (uint64_t j = 0; j < _model->JointCount(); ++j)
  {
    auto joint = _model->JointByIndex(j);

    const std::string& parentLinkName = joint->ParentLinkName();
    ignition::math::graph::VertexId parentLinkId;
    auto vertices = _out.graph.Vertices(parentLinkName);
    if (!vertices.empty())
    {
      parentLinkId = vertices.begin()->first;
    }
    else if (parentLinkName == "world")
    {
      parentLinkId = _out.graph.AddVertex("world", nullptr).Id();
    }
    else
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Joint's parent link is invalid."});
      continue;
    }

    auto childLink = _model->LinkByName(joint->ChildLinkName());
    if (nullptr == childLink)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Joint's child link is invalid."});
      continue;
    }
    auto childLinkId = _out.map.at(childLink->Name());

    _out.graph.AddEdge({parentLinkId, childLinkId}, joint);
  }

  return errors;
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
  const std::string sinkName = "__model__";
  _out.sinkName = sinkName;
  auto modelFrameId =
      _out.graph.AddVertex(sinkName, sdf::FrameType::MODEL).Id();
  _out.map[sinkName] = modelFrameId;

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
      // if the attached-to name is empty, use the sink name
      attachedTo = sinkName;
      if (_out.map.count(sinkName) != 1)
      {
        errors.push_back({ErrorCode::ELEMENT_INVALID,
                         sinkName + "not found in map."});
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
  const std::string sinkName = "__world__";
  _out.sinkName = sinkName;
  auto worldFrameId =
      _out.graph.AddVertex(sinkName, sdf::FrameType::WORLD).Id();
  _out.map[sinkName] = worldFrameId;

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
      // if the attached-to name is empty, use the sink name
      attachedTo = sinkName;
      if (_out.map.count(sinkName) != 1)
      {
        errors.push_back({ErrorCode::ELEMENT_INVALID,
                         sinkName + "not found in map."});
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
  else if (!_model->Element()->HasUniqueChildNames())
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Non-unique names detected in XML children of model."});
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
    auto linkId =
        _out.graph.AddVertex(link->Name(), sdf::FrameType::LINK).Id();
    _out.map[link->Name()] = linkId;

    if (link->PoseRelativeTo().empty())
    {
      // relative_to is empty, so add edge from link to implicit model frame
      _out.graph.AddEdge({modelFrameId, linkId}, link->Pose());
    }
  }

  // add joint vertices and default edge if relative_to is empty
  for (uint64_t j = 0; j < _model->JointCount(); ++j)
  {
    auto joint = _model->JointByIndex(j);
    auto jointId =
        _out.graph.AddVertex(joint->Name(), sdf::FrameType::JOINT).Id();
    _out.map[joint->Name()] = jointId;

    if (joint->PoseRelativeTo().empty())
    {
      // relative_to is empty, so add edge from joint to child link
      auto childLink = _model->LinkByName(joint->ChildLinkName());
      if (nullptr == childLink)
      {
        errors.push_back({ErrorCode::ELEMENT_INVALID,
                         "Joint's child link is invalid."});
        continue;
      }
      auto childLinkId = _out.map.at(childLink->Name());
      _out.graph.AddEdge({childLinkId, jointId}, joint->Pose());
    }
  }

  // add frame vertices and default edge if both
  // relative_to and attached_to are empty
  for (uint64_t f = 0; f < _model->FrameCount(); ++f)
  {
    auto frame = _model->FrameByIndex(f);
    auto frameId =
        _out.graph.AddVertex(frame->Name(), sdf::FrameType::FRAME).Id();
    _out.map[frame->Name()] = frameId;

    if (frame->PoseRelativeTo().empty() && frame->AttachedTo().empty())
    {
      // add edge from frame to implicit model frame
      _out.graph.AddEdge({modelFrameId, frameId}, frame->Pose());
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
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Invalid relative_to value in Link."});
      continue;
    }
    auto relativeToId = _out.map[relativeTo];
    if (link->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Link relative_to itself causes a cycle."});
    }
    _out.graph.AddEdge({relativeToId, linkId}, link->Pose());
  }

  for (uint64_t j = 0; j < _model->JointCount(); ++j)
  {
    auto joint = _model->JointByIndex(j);

    // check if we've already added a default edge
    const std::string relativeTo = joint->PoseRelativeTo();
    if (joint->PoseRelativeTo().empty())
    {
      continue;
    }

    auto jointId = _out.map.at(joint->Name());

    // look for vertex in graph that matches relative_to value
    if (_out.map.count(relativeTo) != 1)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Invalid relative_to value in Joint."});
      continue;
    }
    auto relativeToId = _out.map[relativeTo];
    if (joint->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Joint relative_to itself causes a cycle."});
    }
    _out.graph.AddEdge({relativeToId, jointId}, joint->Pose());
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
    if (!frame->PoseRelativeTo().empty())
    {
      relativeTo = frame->PoseRelativeTo();
      typeForErrorMsg = "relative_to";
    }
    else
    {
      relativeTo = frame->AttachedTo();
      typeForErrorMsg = "attached_to";
    }

    // look for vertex in graph that matches relative_to value
    if (_out.map.count(relativeTo) != 1)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Invalid " + typeForErrorMsg + " value in Frame."});
      continue;
    }
    auto relativeToId = _out.map[relativeTo];
    if (frame->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Frame relative_to itself causes a cycle."});
    }
    _out.graph.AddEdge({relativeToId, frameId}, frame->Pose());
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
  else if (!_world->Element()->HasUniqueChildNames())
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Non-unique names detected in XML children of world."});
    return errors;
  }

  // add implicit world frame vertex first
  const std::string sourceName = "__world__";
  _out.sourceName = sourceName;
  auto worldFrameId =
      _out.graph.AddVertex(sourceName, sdf::FrameType::WORLD).Id();
  _out.map[sourceName] = worldFrameId;

  // add model vertices and default edge if relative_to is empty
  for (uint64_t m = 0; m < _world->ModelCount(); ++m)
  {
    auto model = _world->ModelByIndex(m);
    auto modelId =
        _out.graph.AddVertex(model->Name(), sdf::FrameType::MODEL).Id();
    _out.map[model->Name()] = modelId;

    if (model->PoseRelativeTo().empty())
    {
      // relative_to is empty, so add edge from model to implicit world frame
      _out.graph.AddEdge({worldFrameId, modelId}, model->Pose());
    }
  }

  // add frame vertices and default edge if both
  // relative_to and attached_to are empty
  for (uint64_t f = 0; f < _world->FrameCount(); ++f)
  {
    auto frame = _world->FrameByIndex(f);
    auto frameId =
        _out.graph.AddVertex(frame->Name(), sdf::FrameType::FRAME).Id();
    _out.map[frame->Name()] = frameId;

    if (frame->PoseRelativeTo().empty() && frame->AttachedTo().empty())
    {
      // add edge from frame to implicit world frame
      _out.graph.AddEdge({worldFrameId, frameId}, frame->Pose());
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
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Invalid relative_to value in Model."});
      continue;
    }
    auto relativeToId = _out.map[relativeTo];
    if (model->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Model relative_to itself causes a cycle."});
    }
    _out.graph.AddEdge({relativeToId, modelId}, model->Pose());
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
    if (!frame->PoseRelativeTo().empty())
    {
      relativeTo = frame->PoseRelativeTo();
      typeForErrorMsg = "relative_to";
    }
    else
    {
      relativeTo = frame->AttachedTo();
      typeForErrorMsg = "attached_to";
    }

    // look for vertex in graph that matches relative_to value
    if (_out.map.count(relativeTo) != 1)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Invalid " + typeForErrorMsg + " value in Frame."});
      continue;
    }
    auto relativeToId = _out.map[relativeTo];
    if (frame->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Frame relative_to itself causes a cycle."});
    }
    _out.graph.AddEdge({relativeToId, frameId}, frame->Pose());
  }

  return errors;
}

/////////////////////////////////////////////////
Errors validateFrameAttachedToGraph(const FrameAttachedToGraph &_in)
{
  Errors errors;

  // Expect sinkName to be either "__model__" or "__world__"
  if (_in.sinkName != "__model__" && _in.sinkName != "__world__")
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Sink vertex name " + _in.sinkName +
        " does not match __model__ or __world__."});
    return errors;
  }

  // Expect one vertex with name "__model__" and FrameType MODEL
  // or with name "__world__ and FrameType WORLD
  auto sinkVertices = _in.graph.Vertices(_in.sinkName);
  if (sinkVertices.empty())
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Missing sink vertex with name " + _in.sinkName});
    return errors;
  }
  else if (sinkVertices.size() > 1)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "More than one vertex with sink name " + _in.sinkName});
    return errors;
  }

  auto sinkVertex = sinkVertices.begin()->second.get();
  sdf::FrameType sinkFrameType = sinkVertex.Data();
  if (_in.sinkName == "__model__" && sinkFrameType != sdf::FrameType::MODEL)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Sink vertex with name __model__ should have FrameType MODEL."});
    return errors;
  }
  else if (_in.sinkName == "__world__" &&
           sinkFrameType != sdf::FrameType::WORLD)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Sink vertex with name __world__ should have FrameType WORLD."});
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
    else if (sdf::FrameType::MODEL == sinkFrameType)
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
      // sinkFrameType must be sdf::FrameType::WORLD
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
Errors validatePoseRelativeToGraph(const PoseRelativeToGraph &_in)
{
  Errors errors;

  // Expect sourceName to be either "__model__" or "__world__"
  if (_in.sourceName != "__model__" && _in.sourceName != "__world__")
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Source vertex name " + _in.sourceName +
        " does not match __model__ or __world__."});
    return errors;
  }

  // Expect one vertex with name "__model__" and FrameType MODEL
  // or with name "__world__ and FrameType WORLD
  auto sourceVertices = _in.graph.Vertices(_in.sourceName);
  if (sourceVertices.empty())
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Missing source vertex with name " + _in.sourceName});
    return errors;
  }
  else if (sourceVertices.size() > 1)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "More than one vertex with name " + _in.sourceName});
    return errors;
  }

  auto sourceVertex = sourceVertices.begin()->second.get();
  sdf::FrameType sourceFrameType = sourceVertex.Data();
  if (_in.sourceName == "__model__" && sourceFrameType != sdf::FrameType::MODEL)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Source vertex with name __model__ should have FrameType MODEL."});
    return errors;
  }
  else if (_in.sourceName == "__world__" &&
           sourceFrameType != sdf::FrameType::WORLD)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Source vertex with name __world__ should have FrameType WORLD."});
    return errors;
  }

  // Check number of incoming edges for each vertex
  auto vertices = _in.graph.Vertices();
  for (auto vertexPair : vertices)
  {
    // Vertex names should not be empty
    if (vertexPair.second.get().Name().empty())
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Vertex with empty name detected."});
    }

    auto inDegree = _in.graph.InDegree(vertexPair.first);
    if (inDegree > 1)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Too many incoming edges at a vertex with name [" +
          vertexPair.second.get().Name() + "]."});
    }
    else if (sdf::FrameType::MODEL == sourceFrameType)
    {
      switch (vertexPair.second.get().Data())
      {
        case sdf::FrameType::WORLD:
          errors.push_back({ErrorCode::ELEMENT_INVALID,
              "Vertex with name [" + vertexPair.second.get().Name() + "]" +
              "should not have type WORLD in MODEL relative_to graph."});
          break;
        case sdf::FrameType::MODEL:
          if (inDegree != 0)
          {
            errors.push_back({ErrorCode::ELEMENT_INVALID,
                "MODEL vertex with name [" +
                vertexPair.second.get().Name() +
                "] should have no incoming edges "
                "in MODEL relative_to graph."});
          }
          break;
        default:
          if (inDegree != 1)
          {
            errors.push_back({ErrorCode::ELEMENT_INVALID,
                "Non-MODEL vertex with name [" +
                vertexPair.second.get().Name() +
                "] should have 1 incoming edge " +
                "in MODEL relative_to graph."});
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
          errors.push_back({ErrorCode::ELEMENT_INVALID,
              "No JOINT or LINK vertex should be in WORLD relative_to graph."});
          break;
        case sdf::FrameType::WORLD:
          if (inDegree != 0)
          {
            errors.push_back({ErrorCode::ELEMENT_INVALID,
                "WORLD vertices should have no incoming edges "
                "in WORLD relative_to graph."});
          }
          break;
        default:
          if (inDegree != 1)
          {
            errors.push_back({ErrorCode::ELEMENT_INVALID,
                "MODEL and FRAME vertices in WORLD relative_to graph "
                "should have 1 incoming edge."});
          }
          break;
      }
    }
  }

  // TODO: check graph for cycles
  return errors;
}

/////////////////////////////////////////////////
Errors resolvePoseRelativeToRoot(const PoseRelativeToGraph &_graph,
      const std::string &_vertexName, ignition::math::Pose3d &_pose)
{
  Errors errors;

  if (_graph.map.count(_vertexName) != 1)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Unique vertex with name [" + _vertexName + "] not found in graph."});
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
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Source vertex not found in graph when starting from vertex with "
        "name [" + _vertexName + "]."});
    return errors;
  }
  else if (incomingVertexEdges.first.Name() != _graph.sourceName)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Source vertex found with name [" +
        incomingVertexEdges.first.Name() +
        "], but its name should be " + _graph.sourceName + "."});
    return errors;
  }

  ignition::math::Pose3d pose;
  for (auto const &edge : incomingVertexEdges.second)
  {
    pose = edge.Data() * pose;
  }

  _pose = pose;

  return errors;
}

/////////////////////////////////////////////////
Errors resolvePose(
    const PoseRelativeToGraph &_graph,
    const std::string &_frameName,
    const std::string &_relativeTo,
    ignition::math::Pose3d &_pose)
{
  Errors errors = resolvePoseRelativeToRoot(_graph, _frameName, _pose);

  ignition::math::Pose3d poseR;
  Errors errorsR = resolvePoseRelativeToRoot(_graph, _relativeTo, poseR);
  errors.insert(errors.end(), errorsR.begin(), errorsR.end());

  _pose = poseR.Inverse() * _pose;
  return errors;
}
}
}
