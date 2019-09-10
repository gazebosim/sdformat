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
  auto modelFrameId = _out.graph.AddVertex("", sdf::FrameType::MODEL).Id();
  _out.map[""] = modelFrameId;

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
    // note that the default attached_to value of "" is the vertex name
    // for the implicit model frame
    if (_out.map.count(frame->AttachedTo()) != 1)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Invalid attached_to value in Frame."});
      continue;
    }
    auto attachedToId = _out.map[frame->AttachedTo()];
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
  auto worldFrameId = _out.graph.AddVertex("", sdf::FrameType::WORLD).Id();
  _out.map[""] = worldFrameId;

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
    // note that the default attached_to value of "" is the vertex name
    // for the implicit world frame
    if (_out.map.count(frame->AttachedTo()) != 1)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
                       "Invalid attached_to value in Frame."});
      continue;
    }
    auto attachedToId = _out.map[frame->AttachedTo()];
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
  auto modelFrameId = _out.graph.AddVertex("", sdf::FrameType::MODEL).Id();
  _out.map[""] = modelFrameId;

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
      _out.graph.AddEdge({linkId, modelFrameId}, link->Pose());
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
      _out.graph.AddEdge({jointId, childLinkId}, joint->Pose());
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
      _out.graph.AddEdge({frameId, modelFrameId}, frame->Pose());
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
    _out.graph.AddEdge({linkId, relativeToId}, link->Pose());
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
    _out.graph.AddEdge({jointId, relativeToId}, joint->Pose());
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
    _out.graph.AddEdge({frameId, relativeToId}, frame->Pose());
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
  auto worldFrameId = _out.graph.AddVertex("", sdf::FrameType::WORLD).Id();
  _out.map[""] = worldFrameId;

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
      _out.graph.AddEdge({modelId, worldFrameId}, model->Pose());
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
      _out.graph.AddEdge({frameId, worldFrameId}, frame->Pose());
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
    _out.graph.AddEdge({modelId, relativeToId}, model->Pose());
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
    _out.graph.AddEdge({frameId, relativeToId}, frame->Pose());
  }

  return errors;
}

/////////////////////////////////////////////////
Errors validateFrameAttachedToGraph(const FrameAttachedToGraph &_in)
{
  Errors errors;

  // Expect one vertex with name "" of FrameType MODEL or WORLD
  auto rootVertices = _in.graph.Vertices("");
  if (rootVertices.empty())
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Missing root vertex with empty name."});
    return errors;
  }
  else if (rootVertices.size() > 1)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "More than one vertex with empty name."});
    return errors;
  }

  auto rootVertex = rootVertices.begin()->second.get();
  sdf::FrameType rootFrameType = rootVertex.Data();
  if (rootFrameType != sdf::FrameType::MODEL &&
      rootFrameType != sdf::FrameType::WORLD)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Root vertex with empty name should be MODEL or WORLD."});
    return errors;
  }

  // Check number of outgoing edges for each vertex
  auto vertices = _in.graph.Vertices();
  for (auto vertexPair : vertices)
  {
    auto outDegree = _in.graph.OutDegree(vertexPair.first);
    if (outDegree > 1)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Too many outgoing edges at a vertex with name [" +
          vertexPair.second.get().Name() + "]."});
    }
    else if (sdf::FrameType::MODEL == rootFrameType)
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
      // rootFrameType must be sdf::FrameType::WORLD
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

  // Expect one vertex with name "" of FrameType MODEL or WORLD
  auto rootVertices = _in.graph.Vertices("");
  if (rootVertices.empty())
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Missing root vertex with empty name."});
    return errors;
  }
  else if (rootVertices.size() > 1)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "More than one vertex with empty name."});
    return errors;
  }

  auto rootVertex = rootVertices.begin()->second.get();
  sdf::FrameType rootFrameType = rootVertex.Data();
  if (rootFrameType != sdf::FrameType::MODEL &&
      rootFrameType != sdf::FrameType::WORLD)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Root vertex with empty name should be MODEL or WORLD."});
    return errors;
  }

  // Check number of outgoing edges for each vertex
  auto vertices = _in.graph.Vertices();
  for (auto vertexPair : vertices)
  {
    auto outDegree = _in.graph.OutDegree(vertexPair.first);
    if (outDegree > 1)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Too many outgoing edges at a vertex with name [" +
          vertexPair.second.get().Name() + "]."});
    }
    else if (sdf::FrameType::MODEL == rootFrameType)
    {
      switch (vertexPair.second.get().Data())
      {
        case sdf::FrameType::WORLD:
          errors.push_back({ErrorCode::ELEMENT_INVALID,
              "Vertex with name [" + vertexPair.second.get().Name() + "]" +
              "should not have type WORLD in MODEL relative_to graph."});
          break;
        case sdf::FrameType::MODEL:
          if (outDegree != 0)
          {
            errors.push_back({ErrorCode::ELEMENT_INVALID,
                "MODEL vertex with name [" +
                vertexPair.second.get().Name() +
                "] should have no outgoing edges "
                "in MODEL relative_to graph."});
          }
          break;
        default:
          if (outDegree != 1)
          {
            errors.push_back({ErrorCode::ELEMENT_INVALID,
                "Non-MODEL vertex with name [" +
                vertexPair.second.get().Name() +
                "] should have 1 outgoing edge " +
                "in MODEL relative_to graph."});
          }
          break;
      }
    }
    else
    {
      // rootFrameType must be sdf::FrameType::WORLD
      switch (vertexPair.second.get().Data())
      {
        case sdf::FrameType::JOINT:
        case sdf::FrameType::LINK:
          errors.push_back({ErrorCode::ELEMENT_INVALID,
              "No JOINT or LINK vertex should be in WORLD relative_to graph."});
          break;
        case sdf::FrameType::WORLD:
          if (outDegree != 0)
          {
            errors.push_back({ErrorCode::ELEMENT_INVALID,
                "WORLD vertices should have no outgoing edges "
                "in WORLD relative_to graph."});
          }
          break;
        default:
          if (outDegree != 1)
          {
            errors.push_back({ErrorCode::ELEMENT_INVALID,
                "MODEL and FRAME vertices in WORLD relative_to graph "
                "should have 1 outgoing edge."});
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

  auto outgoingVertexEdges =
      ignition::math::graph::FindSinkVertex(_graph.graph, vertexId);

  if (!outgoingVertexEdges.first.Valid())
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Sink vertex not found in graph when starting from vertex with name [" +
        _vertexName + "]."});
    return errors;
  }
  else if (!outgoingVertexEdges.first.Name().empty())
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Sink vertex found with name [" +
        outgoingVertexEdges.first.Name() +
        "], but its name should be empty."});
    return errors;
  }

  ignition::math::Pose3d pose;
  for (auto const &edge : outgoingVertexEdges.second)
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
