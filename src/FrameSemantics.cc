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
#include <utility>
#include <vector>

#include "sdf/Element.hh"
#include "sdf/Error.hh"
#include "sdf/Frame.hh"
#include "sdf/InterfaceElements.hh"
#include "sdf/InterfaceFrame.hh"
#include "sdf/InterfaceJoint.hh"
#include "sdf/InterfaceLink.hh"
#include "sdf/InterfaceModel.hh"
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

// Helpful functions when debugging in gdb
void printGraph(const ScopedGraph<PoseRelativeToGraph> &_graph)
{
  std::stringstream ss;
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

  if (_id == _graph.ScopeVertexId())
  {
    // This is the source.
    return PairType(vertex, EdgesType());
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
/// \brief Resolve the pose of a model taking into account the placement frame
/// attribute. This function is used to calculate the pose of the edge between a
/// model and its parent.
///
/// Since this function calls sdf::resolvePoseRelativeToRoot to initially
/// resolve the pose of the placement frame, the passed in graph must already
/// have an edge between the input model and its parent frame. This edge will
/// later be updated with the pose calculated by this function.
///
/// Notation used in this function:
/// Pf - The placement frame inside the model
/// R - The `relative_to` frame of the placement frame's //pose element.
/// M - The input model's implicit frame (__model__)
/// X_RPf - The pose of frame Pf relative to frame R
/// X_RM - The pose of the frame M relative to frame R
/// X_MPf - The pose of the frame Pf relative to frame M
///
/// Example:
/// <sdf version="1.8">
///   <world name="W">
///     <frame name="R" />
///     <model name="M" placement_frame="Pf">
///       <pose relative_to="R">{X_RPf}</pose>
///       <link name="Pf"/>
///     </model>
///   </world>
/// </sdf>
///
/// When the placement_frame is specified in an SDFormat file, //model/pose
/// becomes the pose of the placement frame (X_RPf) instead of the pose of the
/// model frame (X_RM). However, when the model is inserted into a pose graph of
/// the parent scope, only the pose (X_RM) of the __model__ frame can be used.
/// i.e, X_RPf cannot be represented in the PoseRelativeTo graph.
/// Thus, the X_RM has to be calculated such that X_RPf = X_RM * X_MPf.
//
/// \param[in] _rawPose Raw pose of the model whose pose to resolve relative to
/// its parent
/// \param[in] _placementFrame Placement frame of the model whose pose to
/// resolve relative to its parent
/// \param[in] _graph The PoseRelativeTo graph with the same scope as the
/// frame referenced by the placement frame attribute of the input model. i.e,
/// the input model's graph.
/// \param[out] _resolvedPose The resolved pose (X_RM). If an error was
/// encountered during `sdf::resolvePoseRelativeToRoot`, _resolvedPose will not
/// be modified.
static Errors resolveModelPoseWithPlacementFrame(
    const ignition::math::Pose3d &_rawPose,
    const std::string _placementFrame,
    const sdf::ScopedGraph<PoseRelativeToGraph> &_graph,
    ignition::math::Pose3d &_resolvedPose)
{
  sdf::Errors errors;

  // Alias for better pose notation
  ignition::math::Pose3d &X_RM = _resolvedPose;
  const ignition::math::Pose3d &X_RPf = _rawPose;

  // If the model has a placement frame, calculate the necessary pose to use
  if (_placementFrame != "")
  {
    ignition::math::Pose3d X_MPf;

    errors = sdf::resolvePoseRelativeToRoot(X_MPf, _graph, _placementFrame);
    if (errors.empty())
    {
      X_RM = X_RPf * X_MPf.Inverse();
    }
  }

  return errors;
}

/////////////////////////////////////////////////
Errors buildFrameAttachedToGraph(
    ScopedGraph<FrameAttachedToGraph> &_out, const Model *_model, bool _isRoot)
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
  else if (_model->LinkCount() == 0 && _model->ModelCount() == 0 &&
      _model->InterfaceModelCount() == 0 && !_model->Static())
  {
    errors.push_back({ErrorCode::MODEL_WITHOUT_LINK,
                     "A model must have at least one link."});
    return errors;
  }

  auto frameType =
      _model->Static() ? sdf::FrameType::STATIC_MODEL : sdf::FrameType::MODEL;

  const std::string scopeContextName = "__model__";

  auto rootId = ignition::math::graph::kNullId;
  if (_isRoot)
  {
    // The __root__ vertex identifies the scope that contains a root level
    // model. In the PoseRelativeTo graph, this vertex is connected to the model
    // with an edge that holds the value of //model/pose. However, in the
    // FrameAttachedTo graph, the vertex is disconnected because nothing is
    // attached to it. Since only links and static models are allowed to be
    // disconnected in this graph, the STATIC_MODEL was chosen as its frame
    // type. A different frame type could potentially be used, but that adds
    // more complexity to the validateFrameAttachedToGraph code.
    _out = _out.AddScopeVertex(
        "", "__root__", scopeContextName, sdf::FrameType::STATIC_MODEL);
    rootId = _out.ScopeVertexId();
  }

  const auto modelId = _out.AddVertex(_model->Name(), frameType).Id();

  auto outModel = _out.AddScopeVertex(
      _model->Name(), scopeContextName, scopeContextName, frameType);
  const auto modelFrameId = outModel.ScopeVertexId();

  // Add an aliasing edge from the model vertex to the
  // corresponding vertex in the FrameAttachedTo graph of the child model,
  auto &edge = outModel.AddEdge({modelId, modelFrameId}, true);
  // Set the edge weight to 0 to indicate that this is an aliasing edge.
  edge.SetWeight(0);

  // add link vertices
  for (uint64_t l = 0; l < _model->LinkCount(); ++l)
  {
    auto link = _model->LinkByIndex(l);
    if (outModel.Count(link->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Link with non-unique name [" + link->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    outModel.AddVertex(link->Name(), sdf::FrameType::LINK);
  }

  // add joint vertices
  for (uint64_t j = 0; j < _model->JointCount(); ++j)
  {
    auto joint = _model->JointByIndex(j);
    if (outModel.Count(joint->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Joint with non-unique name [" + joint->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    outModel.AddVertex(joint->Name(), sdf::FrameType::JOINT).Id();
  }

  // add frame vertices
  for (uint64_t f = 0; f < _model->FrameCount(); ++f)
  {
    auto frame = _model->FrameByIndex(f);
    if (outModel.Count(frame->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Frame with non-unique name [" + frame->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    outModel.AddVertex(frame->Name(), sdf::FrameType::FRAME).Id();
  }

  // add nested model vertices
  for (uint64_t m = 0; m < _model->ModelCount(); ++m)
  {
    auto nestedModel = _model->ModelByIndex(m);
    if (outModel.Count(nestedModel->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Nested model with non-unique name [" + nestedModel->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    auto nestedErrors = buildFrameAttachedToGraph(outModel, nestedModel, false);
    errors.insert(errors.end(), nestedErrors.begin(), nestedErrors.end());
  }

  // add nested interface model vertices
  for (uint64_t m = 0; m < _model->InterfaceModelCount(); ++m)
  {
    auto nestedIfaceModel = _model->InterfaceModelByIndex(m);
    if (outModel.Count(nestedIfaceModel->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Nested interface model with non-unique name [" +
              nestedIfaceModel->Name() + "] detected in model with name [" +
              _model->Name() + "]."});
      continue;
    }
    auto nestedErrors = buildFrameAttachedToGraph(outModel, nestedIfaceModel);
    errors.insert(errors.end(), nestedErrors.begin(), nestedErrors.end());
  }

  // add edges from joint to child frames
  for (uint64_t j = 0; j < _model->JointCount(); ++j)
  {
    auto joint = _model->JointByIndex(j);
    auto jointId = outModel.VertexIdByName(joint->Name());
    auto childFrameName = joint->ChildLinkName();
    if (outModel.Count(childFrameName) != 1)
    {
      errors.push_back({ErrorCode::JOINT_CHILD_LINK_INVALID,
        "Child frame with name[" + childFrameName +
        "] specified by joint with name[" + joint->Name() +
        "] not found in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto childFrameId = outModel.VertexIdByName(childFrameName);
    outModel.AddEdge({jointId, childFrameId}, true);
  }

  // add frame edges
  for (uint64_t f = 0; f < _model->FrameCount(); ++f)
  {
    auto frame = _model->FrameByIndex(f);
    auto frameId = outModel.VertexIdByName(frame->Name());
    // look for vertex in graph that matches attached_to value
    std::string attachedTo = frame->AttachedTo();
    if (attachedTo.empty())
    {
      // if the attached-to name is empty, use the scope context name
      attachedTo = scopeContextName;
    }
    if (outModel.Count(attachedTo) != 1)
    {
      errors.push_back({ErrorCode::FRAME_ATTACHED_TO_INVALID,
          "attached_to name[" + attachedTo +
          "] specified by frame with name[" + frame->Name() +
          "] does not match a nested model, link, joint, or frame name "
          "in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto attachedToId = outModel.VertexIdByName(attachedTo);
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
    outModel.AddEdge({frameId, attachedToId}, edgeData);
  }

  // identify canonical link, which may be nested
  const auto[canonicalLink, canonicalLinkName] =
      modelCanonicalLinkAndRelativeName(_model);
  if (!_model->Static())
  {
    if (nullptr == canonicalLink)
    {
      if (canonicalLinkName.empty())
      {
        if (_model->ModelCount() == 0 && _model->InterfaceModelCount() == 0)
        {
          errors.push_back({ErrorCode::MODEL_WITHOUT_LINK,
              "A model must have at least one link."});
        }
        else
        {
          // The canonical link was not found, but the model could have a
          // descendant that has a static model, so simply create an edge to the
          // first model and let the attached_to frame resolution take care of
          // finding the canonical link
          //
          std::string firstChildModelName = "";
          if (_model->ModelCount() > 0)
          {
            firstChildModelName = _model->ModelByIndex(0)->Name();
          }
          else
          {
            firstChildModelName = _model->InterfaceModelByIndex(0)->Name();
          }
          auto firstChildModelId = outModel.VertexIdByName(firstChildModelName);
          outModel.AddEdge({modelFrameId, firstChildModelId}, true);
        }
      }
      else
      {
        // Search for the vertex in case the canonical link is an InterfaceLink
        auto canonicalLinkId = outModel.VertexIdByName(canonicalLinkName);
        if (ignition::math::graph::kNullId != canonicalLinkId)
        {
          outModel.AddEdge({modelFrameId, canonicalLinkId}, true);
        }
        else
        {
          errors.push_back({ErrorCode::MODEL_CANONICAL_LINK_INVALID,
              "canonical_link with name[" + canonicalLinkName +
              "] not found in model with name[" + _model->Name() + "]."});
        }
      }
      // return early
      return errors;
    }
    else
    {
      // Add an edge from the implicit model frame to the canonical link found.
      auto linkId = outModel.VertexIdByName(canonicalLinkName);
      outModel.AddEdge({modelFrameId, linkId}, true);
    }
  }

  return errors;
}

/////////////////////////////////////////////////
Errors buildFrameAttachedToGraph(ScopedGraph<FrameAttachedToGraph> &_out,
    InterfaceModelConstPtr _model)
{
  Errors errors;

  if (!_model)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Invalid sdf::InterfaceModel pointer."});
    return errors;
  }
  else if (_model->Links().size() == 0u && _model->NestedModels().size() == 0 &&
      !_model->Static())
  {
    errors.push_back({ErrorCode::MODEL_WITHOUT_LINK,
        "Model with name[" + _model->Name() +
            "] must have at least one link."});
    return errors;
  }

  auto frameType =
      _model->Static() ? sdf::FrameType::STATIC_MODEL : sdf::FrameType::MODEL;

  const std::string scopeContextName = "__model__";

  const auto modelId = _out.AddVertex(_model->Name(), frameType).Id();

  auto outModel = _out.AddScopeVertex(
      _model->Name(), scopeContextName, scopeContextName, frameType);
  const auto modelFrameId = outModel.ScopeVertexId();

  // Add an aliasing edge from the model vertex to the
  // corresponding vertex in the FrameAttachedTo graph of the child model,
  auto &edge = outModel.AddEdge({modelId, modelFrameId}, true);
  // Set the edge weight to 0 to indicate that this is an aliasing edge.
  edge.SetWeight(0);

  // add link vertices
  for (const auto &link : _model->Links())
  {
    if (outModel.Count(link.Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Link with non-unique name [" + link.Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    outModel.AddVertex(link.Name(), sdf::FrameType::LINK);
  }

  // add joint vertices
  for (const auto &joint : _model->Joints())
  {
    if (outModel.Count(joint.Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Joint with non-unique name [" + joint.Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    outModel.AddVertex(joint.Name(), sdf::FrameType::JOINT);
  }

  // add frame vertices
  for (const auto &frame : _model->Frames())
  {
    if (outModel.Count(frame.Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Frame with non-unique name [" + frame.Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    outModel.AddVertex(frame.Name(), sdf::FrameType::FRAME);
  }

  // add nested model vertices
  for (const auto &nestedIfaceModel : _model->NestedModels())
  {
    if (outModel.Count(nestedIfaceModel->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Nested model with non-unique name [" + nestedIfaceModel->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    auto nestedErrors = buildFrameAttachedToGraph(outModel, nestedIfaceModel);
    errors.insert(errors.end(), nestedErrors.begin(), nestedErrors.end());
  }

  // add edges from joint to child frames
  for (const auto &joint : _model->Joints())
  {
    auto jointId = outModel.VertexIdByName(joint.Name());
    const auto &childFrameName = joint.ChildName();
    if (outModel.Count(childFrameName) != 1)
    {
      errors.push_back({ErrorCode::JOINT_CHILD_LINK_INVALID,
        "Child frame with name[" + childFrameName +
        "] specified by joint with name[" + joint.Name() +
        "] not found in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto childFrameId = outModel.VertexIdByName(childFrameName);
    outModel.AddEdge({jointId, childFrameId}, true);
  }

  // add frame edges
  for (const auto &frame : _model->Frames())
  {
    auto frameId = outModel.VertexIdByName(frame.Name());
    // look for vertex in graph that matches attached_to value
    std::string attachedTo = frame.AttachedTo();
    if (attachedTo.empty())
    {
      // if the attached-to name is empty, use the scope context name
      attachedTo = scopeContextName;
    }
    if (outModel.Count(attachedTo) != 1)
    {
      errors.push_back({ErrorCode::FRAME_ATTACHED_TO_INVALID,
          "attached_to name[" + attachedTo +
          "] specified by frame with name[" + frame.Name() +
          "] does not match a nested model, link, joint, or frame name "
          "in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto attachedToId = outModel.VertexIdByName(attachedTo);
    bool edgeData = true;
    if (frame.Name() == frame.AttachedTo())
    {
      // set edgeData to false if attaches to itself, since this is invalid
      edgeData = false;
      errors.push_back({ErrorCode::FRAME_ATTACHED_TO_CYCLE,
          "attached_to name[" + attachedTo +
          "] is identical to frame name[" + frame.Name() +
          "], causing a graph cycle "
          "in model with name[" + _model->Name() + "]."});
    }
    outModel.AddEdge({frameId, attachedToId}, edgeData);
  }

  // identify canonical link, which may be nested
  const std::string canonicalLinkName = _model->CanonicalLinkName();
  const auto canonicalLinkId = outModel.VertexIdByName(canonicalLinkName);
  if (!_model->Static())
  {
    if (ignition::math::graph::kNullId == canonicalLinkId)
    {
      if (canonicalLinkName.empty())
      {
        if (_model->NestedModels().size() == 0u)
        {
          errors.push_back({ErrorCode::MODEL_WITHOUT_LINK,
              "Interface model with name[" + _model->Name() +
                  "] must have at least one link."});
        }
        else
        {
          // The canonical link was not found, but the model could have a
          // descendant that has a static model, so simply create an edge to the
          // first model and let the attached_to frame resolution take care of
          // finding the canonical link
          auto firstChildModelId =
              outModel.VertexIdByName(_model->NestedModels().front()->Name());
          outModel.AddEdge({modelFrameId, firstChildModelId }, true);
        }
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
    else
    {
      // Add an edge from the implicit model frame to the canonical link found.
      outModel.AddEdge({modelFrameId, canonicalLinkId}, true);
    }
  }

  return errors;
}

/////////////////////////////////////////////////
Errors buildFrameAttachedToGraph(
            ScopedGraph<FrameAttachedToGraph> &_out, const World *_world)
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
  const std::string scopeContextName = "world";
  _out = _out.AddScopeVertex(
      "", scopeContextName, scopeContextName, sdf::FrameType::WORLD);

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
    auto modelErrors = buildFrameAttachedToGraph(_out, model, false);
    errors.insert(errors.end(), modelErrors.begin(), modelErrors.end());
  }

  // add interface model vertices
  for (uint64_t im = 0; im < _world->InterfaceModelCount(); ++im)
  {
    auto ifaceModel = _world->InterfaceModelByIndex(im);
    if (_out.Count(ifaceModel->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Interface Model with non-unique name [" + ifaceModel->Name() +
          "] detected in world with name [" + _world->Name() +
          "]."});
      continue;
    }
    auto modelErrors = buildFrameAttachedToGraph(_out, ifaceModel);
    errors.insert(errors.end(), modelErrors.begin(), modelErrors.end());
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

  // add frame edges
  for (uint64_t f = 0; f < _world->FrameCount(); ++f)
  {
    auto frame = _world->FrameByIndex(f);
    auto frameId = _out.VertexIdByName(frame->Name());
    // look for vertex in graph that matches attached_to value
    std::string attachedTo = frame->AttachedTo();
    if (attachedTo.empty())
    {
      // if the attached-to name is empty, use the scope context name
      attachedTo = scopeContextName;
      if (_out.Count(scopeContextName) != 1)
      {
        errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                         "FrameAttachedToGraph error: scope frame[" +
                         scopeContextName + "] not found in map."});
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

  return errors;
}

/////////////////////////////////////////////////
Errors buildPoseRelativeToGraph(
    ScopedGraph<PoseRelativeToGraph> &_out, const Model *_model, bool _isRoot)
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

  const std::string scopeContextName = "__model__";
  auto rootId = ignition::math::graph::kNullId;
  // add the model frame vertex first
  if (_isRoot)
  {
    _out = _out.AddScopeVertex(
        "", "__root__", scopeContextName, sdf::FrameType::MODEL);
    rootId = _out.ScopeVertexId();
  }
  auto modelId = _out.AddVertex(_model->Name(), sdf::FrameType::MODEL).Id();
  auto outModel = _out.AddScopeVertex(_model->Name(), scopeContextName,
      scopeContextName, sdf::FrameType::MODEL);
  auto modelFrameId = outModel.ScopeVertexId();

  // Add an aliasing edge from the model vertex to the
  // corresponding vertex in the PoseRelativeTo graph of the child model,
  // i.e, to the <model_name>::__model__ vertex.
  auto &edge = _out.AddEdge({modelId, modelFrameId}, {});
  // Set the edge weight to 0 to indicate that this is an aliasing edge.
  edge.SetWeight(0);

  // add link vertices and default edge if relative_to is empty
  for (uint64_t l = 0; l < _model->LinkCount(); ++l)
  {
    auto link = _model->LinkByIndex(l);
    if (outModel.Count(link->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Link with non-unique name [" + link->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    auto linkId =
        outModel.AddVertex(link->Name(), sdf::FrameType::LINK).Id();

    if (link->PoseRelativeTo().empty())
    {
      // relative_to is empty, so add edge from implicit model frame to link
      outModel.AddEdge({modelFrameId, linkId}, link->RawPose());
    }
  }

  // add joint vertices
  for (uint64_t j = 0; j < _model->JointCount(); ++j)
  {
    auto joint = _model->JointByIndex(j);
    if (outModel.Count(joint->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Joint with non-unique name [" + joint->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    outModel.AddVertex(joint->Name(), sdf::FrameType::JOINT).Id();
  }

  // add frame vertices and default edge if both
  // relative_to and attached_to are empty
  for (uint64_t f = 0; f < _model->FrameCount(); ++f)
  {
    auto frame = _model->FrameByIndex(f);
    if (outModel.Count(frame->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Frame with non-unique name [" + frame->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    auto frameId =
        outModel.AddVertex(frame->Name(), sdf::FrameType::FRAME).Id();

    if (frame->PoseRelativeTo().empty() && frame->AttachedTo().empty())
    {
      // add edge from implicit model frame to frame
      outModel.AddEdge({modelFrameId, frameId}, frame->RawPose());
    }
  }

  // add nested model vertices
  for (uint64_t m = 0; m < _model->ModelCount(); ++m)
  {
    auto nestedModel = _model->ModelByIndex(m);
    if (outModel.Count(nestedModel->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Nested model with non-unique name [" + nestedModel->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }

    auto nestedErrors = buildPoseRelativeToGraph(outModel, nestedModel, false);
    errors.insert(errors.end(), nestedErrors.begin(), nestedErrors.end());
  }

  // add nested interface model vertices
  for (uint64_t m = 0; m < _model->InterfaceModelCount(); ++m)
  {
    auto ifaceModel = _model->InterfaceModelByIndex(m);
    if (outModel.Count(ifaceModel->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Nested model with non-unique name [" + ifaceModel->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }

    auto nestedErrors = buildPoseRelativeToGraph(outModel, ifaceModel);
    errors.insert(errors.end(), nestedErrors.begin(), nestedErrors.end());
  }

  // now that all vertices have been added to the graph,
  // add the edges that reference other vertices

  for (uint64_t l = 0; l < _model->LinkCount(); ++l)
  {
    auto link = _model->LinkByIndex(l);

    // check if we've already added a default edge
    const std::string &relativeTo = link->PoseRelativeTo();
    if (relativeTo.empty())
    {
      continue;
    }

    auto linkId = outModel.VertexIdByName(link->Name());

    // look for vertex in graph that matches relative_to value
    if (outModel.Count(relativeTo) != 1)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
          "relative_to name[" + relativeTo +
          "] specified by link with name[" + link->Name() +
          "] does not match a nested model, link, joint, or frame name "
          "in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto relativeToId = outModel.VertexIdByName(relativeTo);
    if (link->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "relative_to name[" + relativeTo +
          "] is identical to link name[" + link->Name() +
          "], causing a graph cycle "
          "in model with name[" + _model->Name() + "]."});
    }
    outModel.AddEdge({relativeToId, linkId}, link->RawPose());
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

    auto jointId = outModel.VertexIdByName(joint->Name());

    // look for vertex in graph that matches relative_to value
    if (outModel.Count(relativeTo) != 1)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
          "relative_to name[" + relativeTo +
          "] specified by joint with name[" + joint->Name() +
          "] does not match a nested model, link, joint, or frame name "
          "in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto relativeToId = outModel.VertexIdByName(relativeTo);
    if (joint->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "relative_to name[" + relativeTo +
          "] is identical to joint name[" + joint->Name() +
          "], causing a graph cycle "
          "in model with name[" + _model->Name() + "]."});
    }
    outModel.AddEdge({relativeToId, jointId}, joint->RawPose());
  }

  for (uint64_t f = 0; f < _model->FrameCount(); ++f)
  {
    auto frame = _model->FrameByIndex(f);

    // check if we've already added a default edge
    if (frame->PoseRelativeTo().empty() && frame->AttachedTo().empty())
    {
      continue;
    }

    auto frameId = outModel.VertexIdByName(frame->Name());
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
    if (outModel.Count(relativeTo) != 1)
    {
      errors.push_back({errorCode,
          typeForErrorMsg + " name[" + relativeTo +
          "] specified by frame with name[" + frame->Name() +
          "] does not match a nested model, link, joint, or frame name "
          "in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto relativeToId = outModel.VertexIdByName(relativeTo);
    if (frame->Name() == relativeTo)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "relative_to name[" + relativeTo +
          "] is identical to frame name[" + frame->Name() +
          "], causing a graph cycle "
          "in model with name[" + _model->Name() + "]."});
    }
    outModel.AddEdge({relativeToId, frameId}, frame->RawPose());
  }

  for (uint64_t m = 0; m < _model->ModelCount(); ++m)
  {
    auto nestedModel = _model->ModelByIndex(m);

    auto nestedModelId = outModel.VertexIdByName(nestedModel->Name());
    // if relative_to is empty, add edge from implicit model frame to
    // nestedModel
    auto relativeToId = modelFrameId;

    const std::string &relativeTo = nestedModel->PoseRelativeTo();
    if (!relativeTo.empty())
    {
      // look for vertex in graph that matches relative_to value
      if (outModel.Count(relativeTo) != 1)
      {
        errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
            "relative_to name[" + relativeTo +
            "] specified by nested model with name[" + nestedModel->Name() +
            "] does not match a nested model, link, joint, or frame name "
            "in model with name[" + _model->Name() + "]."});
        continue;
      }

      relativeToId = outModel.VertexIdByName(relativeTo);
      if (nestedModel->Name() == relativeTo)
      {
        errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
            "relative_to name[" + relativeTo +
            "] is identical to nested model name[" + nestedModel->Name() +
            "], causing a graph cycle "
            "in model with name[" + _model->Name() + "]."});
      }
    }

    ignition::math::Pose3d resolvedModelPose = nestedModel->RawPose();
    sdf::Errors resolveErrors = resolveModelPoseWithPlacementFrame(
        nestedModel->RawPose(), nestedModel->PlacementFrameName(),
        outModel.ChildModelScope(nestedModel->Name()), resolvedModelPose);
    errors.insert(errors.end(), resolveErrors.begin(), resolveErrors.end());

    outModel.AddEdge({relativeToId, nestedModelId}, resolvedModelPose);
  }
  for (uint64_t m = 0; m < _model->InterfaceModelCount(); ++m)
  {
    auto ifaceModel = _model->InterfaceModelByIndex(m);
    const auto *nestedInclude = _model->InterfaceModelNestedIncludeByIndex(m);

    auto nestedModelId = outModel.VertexIdByName(ifaceModel->Name());
    // if relative_to is empty, add edge from implicit model frame to
    // nestedModel
    auto relativeToId = modelFrameId;

    const std::string &relativeTo =
        nestedInclude->includePoseRelativeTo.value_or("");
    if (!relativeTo.empty())
    {
      // look for vertex in graph that matches relative_to value
      if (outModel.Count(relativeTo) != 1)
      {
        errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
            "relative_to name[" + relativeTo +
            "] specified by nested model with name[" + ifaceModel->Name() +
            "] does not match a nested model, link, joint, or frame name "
            "in model with name[" + _model->Name() + "]."});
        continue;
      }

      relativeToId = outModel.VertexIdByName(relativeTo);
      if (ifaceModel->Name() == relativeTo)
      {
        errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
            "relative_to name[" + relativeTo +
            "] is identical to nested model name[" + ifaceModel->Name() +
            "], causing a graph cycle "
            "in model with name[" + _model->Name() + "]."});
      }
    }

    ignition::math::Pose3d resolvedModelPose =
        nestedInclude->includeRawPose.value_or(ignition::math::Pose3d());

    sdf::Errors resolveErrors = resolveModelPoseWithPlacementFrame(
        nestedInclude->includeRawPose.value_or(ignition::math::Pose3d()),
        nestedInclude->placementFrame.value_or(""),
        outModel.ChildModelScope(ifaceModel->Name()), resolvedModelPose);
    errors.insert(errors.end(), resolveErrors.begin(), resolveErrors.end());

    outModel.AddEdge({relativeToId, nestedModelId}, resolvedModelPose);
  }

  if (_isRoot)
  {
    // We have to add this edge now with an identity pose to be able to call
    // resolveModelPoseWithPlacementFrame, which in turn calls
    // sdf::resolvePoseRelativeToRoot. We will later update the edge after the
    // pose is calculated.
    auto rootToModel = outModel.AddEdge({rootId, modelId}, {});
    ignition::math::Pose3d resolvedModelPose = _model->RawPose();
    sdf::Errors resolveErrors =
        resolveModelPoseWithPlacementFrame(_model->RawPose(),
            _model->PlacementFrameName(), outModel, resolvedModelPose);
    errors.insert(errors.end(), resolveErrors.begin(), resolveErrors.end());

    outModel.UpdateEdge(rootToModel, resolvedModelPose);
  }
  return errors;
}

/////////////////////////////////////////////////
Errors buildPoseRelativeToGraph(ScopedGraph<PoseRelativeToGraph> &_out,
    InterfaceModelConstPtr _model)
{
  Errors errors;

  if (!_model)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Invalid sdf::Model pointer."});
    return errors;
  }

  const std::string scopeContextName = "__model__";
  // add the model frame vertex first
  auto modelId = _out.AddVertex(_model->Name(), sdf::FrameType::MODEL).Id();
  auto outModel = _out.AddScopeVertex(_model->Name(), scopeContextName,
      scopeContextName, sdf::FrameType::MODEL);
  auto modelFrameId = outModel.ScopeVertexId();

  // Add an aliasing edge from the model vertex to the
  // corresponding vertex in the PoseRelativeTo graph of the child model,
  // i.e, to the <model_name>::__model__ vertex.
  auto &edge = _out.AddEdge({modelId, modelFrameId}, {});
  // Set the edge weight to 0 to indicate that this is an aliasing edge.
  edge.SetWeight(0);

  // add link vertices and default edge if relative_to is empty
  for (const auto &link : _model->Links())
  {
    if (outModel.Count(link.Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Link with non-unique name [" + link.Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    auto linkId =
        outModel.AddVertex(link.Name(), sdf::FrameType::LINK).Id();

    // relative_to is empty, so add edge from implicit model frame to link
    outModel.AddEdge({modelFrameId, linkId}, link.PoseInModelFrame());
  }

  // add joint vertices
  for (const auto &joint : _model->Joints())
  {
    if (outModel.Count(joint.Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Joint with non-unique name [" + joint.Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    outModel.AddVertex(joint.Name(), sdf::FrameType::JOINT);
  }

  // add frame vertices and default edge if both
  // relative_to and attached_to are empty
  for (const auto &frame: _model->Frames())
  {
    if (outModel.Count(frame.Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Frame with non-unique name [" + frame.Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }
    auto frameId = outModel.AddVertex(frame.Name(), sdf::FrameType::FRAME).Id();
    if (frame.AttachedTo().empty())
    {
      // add edge from implicit model frame to frame
      outModel.AddEdge({modelFrameId, frameId}, frame.PoseInAttachedToFrame());
    }
  }

  // add nested model vertices and default edge if relative_to is empty
  for (const auto &nestedIfaceModel : _model->NestedModels())
  {
    if (outModel.Count(nestedIfaceModel->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Nested model with non-unique name [" + nestedIfaceModel->Name() +
          "] detected in model with name [" + _model->Name() +
          "]."});
      continue;
    }

    auto nestedErrors = buildPoseRelativeToGraph(outModel, nestedIfaceModel);
    errors.insert(errors.end(), nestedErrors.begin(), nestedErrors.end());
    auto nestedModelId = outModel.VertexIdByName(nestedIfaceModel->Name());
    outModel.AddEdge({modelFrameId, nestedModelId},
        nestedIfaceModel->ModelFramePoseInParentFrame());
  }

  for (const auto &joint : _model->Joints())
  {
    const std::string &relativeTo = joint.ChildName();
    auto jointId = outModel.VertexIdByName(joint.Name());

    // look for vertex in graph that matches relative_to value
    if (outModel.Count(relativeTo) != 1)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
          "relative_to name[" + relativeTo +
          "] specified by joint with name[" + joint.Name() +
          "] does not match a nested model, link, joint, or frame name "
          "in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto relativeToId = outModel.VertexIdByName(relativeTo);
    if (joint.Name() == relativeTo)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "relative_to name[" + relativeTo +
          "] is identical to joint name[" + joint.Name() +
          "], causing a graph cycle "
          "in model with name[" + _model->Name() + "]."});
    }
    outModel.AddEdge({relativeToId, jointId}, joint.PoseInChildFrame());
  }

  for (const auto &frame: _model->Frames())
  {
    if (frame.AttachedTo().empty())
    {
      continue;
    }
    auto frameId = outModel.VertexIdByName(frame.Name());
    std::string relativeTo;
    std::string typeForErrorMsg;
    ErrorCode errorCode;
    relativeTo = frame.AttachedTo();
    typeForErrorMsg = "attached_to";
    errorCode = ErrorCode::FRAME_ATTACHED_TO_INVALID;

    // look for vertex in graph that matches relative_to value
    if (outModel.Count(relativeTo) != 1)
    {
      errors.push_back({errorCode,
          "attached_to name[" + relativeTo +
          "] specified by frame with name[" + frame.Name() +
          "] does not match a nested model, link, joint, or frame name "
          "in model with name[" + _model->Name() + "]."});
      continue;
    }
    auto relativeToId = outModel.VertexIdByName(relativeTo);
    if (frame.Name() == relativeTo)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
          "relative_to name[" + relativeTo +
          "] is identical to frame name[" + frame.Name() +
          "], causing a graph cycle "
          "in model with name[" + _model->Name() + "]."});
    }
    outModel.AddEdge({relativeToId, frameId}, frame.PoseInAttachedToFrame());
  }

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
  const std::string scopeContextName = "world";

  _out = _out.AddScopeVertex(
      "", "__root__", scopeContextName, sdf::FrameType::WORLD);
  auto rootId = _out.ScopeVertexId();

  _out = _out.AddScopeVertex(
      "", scopeContextName, scopeContextName, sdf::FrameType::WORLD);
  auto worldFrameId = _out.ScopeVertexId();

  _out.AddEdge({rootId, worldFrameId}, {});
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

    auto modelErrors = buildPoseRelativeToGraph(_out , model, false);
    errors.insert(errors.end(), modelErrors.begin(), modelErrors.end());
  }

  for (uint64_t m = 0; m < _world->InterfaceModelCount(); ++m)
  {
    auto ifaceModel = _world->InterfaceModelByIndex(m);
    if (_out.Count(ifaceModel->Name()) > 0)
    {
      errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Model with non-unique name [" + ifaceModel->Name() +
          "] detected in world with name [" + _world->Name() +
          "]."});
      continue;
    }

    auto modelErrors =
        buildPoseRelativeToGraph(_out, ifaceModel);
    errors.insert(errors.end(), modelErrors.begin(), modelErrors.end());
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

    auto modelId = _out.VertexIdByName(model->Name());
    // if relative_to is empty, add edge from implicit model frame to
    // world
    auto relativeToId = worldFrameId;

    const std::string &relativeTo = model->PoseRelativeTo();
    if (!relativeTo.empty())
    {
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

      relativeToId = _out.VertexIdByName(relativeTo);
      if (model->Name() == relativeTo)
      {
        errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
            "relative_to name[" + relativeTo +
            "] is identical to model name[" + model->Name() +
            "], causing a graph cycle "
            "in world with name[" + _world->Name() + "]."});
      }
    }

    ignition::math::Pose3d resolvedModelPose = model->RawPose();
    sdf::Errors resolveErrors = resolveModelPoseWithPlacementFrame(
        model->RawPose(), model->PlacementFrameName(),
        _out.ChildModelScope(model->Name()), resolvedModelPose);
    errors.insert(errors.end(), resolveErrors.begin(), resolveErrors.end());

    _out.AddEdge({relativeToId, modelId}, resolvedModelPose);
  }

  for (uint64_t m = 0; m < _world->InterfaceModelCount(); ++m)
  {
    auto ifaceModel = _world->InterfaceModelByIndex(m);
    const auto *nestedInclude = _world->InterfaceModelNestedIncludeByIndex(m);

    auto modelId = _out.VertexIdByName(ifaceModel->Name());
    // if relative_to is empty, add edge from implicit model frame to
    // world
    auto relativeToId = worldFrameId;

    const std::string &relativeTo =
        nestedInclude->includePoseRelativeTo.value_or("");
    if (!relativeTo.empty())
    {
      // look for vertex in graph that matches relative_to value
      if (_out.Count(relativeTo) != 1)
      {
        errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
            "relative_to name[" + relativeTo +
            "] specified by interface model with name[" + ifaceModel->Name() +
            "] does not match a model or frame name "
            "in world with name[" + _world->Name() + "]."});
        continue;
      }

      relativeToId = _out.VertexIdByName(relativeTo);
      if (ifaceModel->Name() == relativeTo)
      {
        errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
            "relative_to name[" + relativeTo +
            "] is identical to interface model name[" + ifaceModel->Name() +
            "], causing a graph cycle "
            "in world with name[" + _world->Name() + "]."});
      }
    }

    ignition::math::Pose3d resolvedModelPose =
        nestedInclude->includeRawPose.value_or(ignition::math::Pose3d());

    sdf::Errors resolveErrors = resolveModelPoseWithPlacementFrame(
        nestedInclude->includeRawPose.value_or(ignition::math::Pose3d()),
        nestedInclude->placementFrame.value_or(""),
        _out.ChildModelScope(ifaceModel->Name()), resolvedModelPose);
    errors.insert(errors.end(), resolveErrors.begin(), resolveErrors.end());

    _out.AddEdge({relativeToId, modelId}, resolvedModelPose);
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

  return errors;
}

/////////////////////////////////////////////////
Errors validateFrameAttachedToGraph(
    const ScopedGraph<FrameAttachedToGraph> &_in)
{
  Errors errors;

  // Expect ScopeContextName to be either "__model__" or "world"
  if (_in.ScopeContextName() != "__model__" &&
      _in.ScopeContextName() != "world")
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "FrameAttachedToGraph error: scope context name[" +
            _in.ScopeContextName() + "] does not match __model__ or world."});
    return errors;
  }

  // Expect the scope vertex to be valid and check that the scope context
  // matches the frame type.
  auto scopeVertex = _in.ScopeVertex();
  if (!scopeVertex.Valid())
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                     "FrameAttachedToGraph error: scope frame[" +
                     _in.ScopeContextName() + "] not found in graph."});
    return errors;
  }

  sdf::FrameType scopeFrameType = scopeVertex.Data();
  if (_in.ScopeContextName() == "__model__" &&
      scopeFrameType != sdf::FrameType::MODEL &&
      scopeFrameType != sdf::FrameType::STATIC_MODEL)
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "FrameAttachedToGraph error, "
        "scope vertex with name __model__ should have FrameType MODEL or "
        "STATIC_MODEL."});
    return errors;
  }
  else if (_in.ScopeContextName() == "world" &&
           scopeFrameType != sdf::FrameType::WORLD)
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "FrameAttachedToGraph error, "
        "scope vertex with name world should have FrameType WORLD."});
    return errors;
  }

  // Check number of outgoing edges for each vertex
  auto vertices = _in.Graph().Vertices();
  for (auto vertexPair : vertices)
  {
    const std::string vertexName = _in.VertexLocalName(vertexPair.second.get());
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
    else if (vertexName == "__root__" )
    {
      if (sdf::FrameType::STATIC_MODEL != scopeFrameType &&
          sdf::FrameType::WORLD != scopeFrameType)
      {
        errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
            "FrameAttachedToGraph error,"
            " __root__ should have FrameType STATIC_MODEL or WORLD."});
      }
      else if (outDegree != 0)
      {
        std::string graphType =
            scopeFrameType == sdf::FrameType::WORLD ? "WORLD" : "MODEL";
        errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
            "FrameAttachedToGraph error,"
            " __root__ should have no outgoing edges in " +
                graphType + " attached_to graph."});
      }
    }
    else if (sdf::FrameType::MODEL == scopeFrameType ||
        sdf::FrameType::STATIC_MODEL == scopeFrameType)
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
        case sdf::FrameType::STATIC_MODEL:
          if (outDegree != 0)
          {
            auto edges = _in.Graph().IncidentsFrom(vertexPair.first);
            // Filter out alias edges (edge with a weight of 0)
            auto outDegreeNoAliases = std::count_if(
                edges.begin(), edges.end(), [](const auto &_edge)
                {
                  return _edge.second.get().Weight() >= 1.0;
                });
            if (outDegreeNoAliases)
            {
              errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                  "FrameAttachedToGraph error, "
                  "STATIC_MODEL vertex with name [" +
                  vertexName +
                  "] should have no outgoing edges "
                  "in MODEL attached_to graph."});
            }
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
        case sdf::FrameType::STATIC_MODEL:
          if (outDegree != 0)
          {
            auto edges = _in.Graph().IncidentsFrom(vertexPair.first);
            // Filter out alias edges (edge with a weight of 0)
            auto outDegreeNoAliases = std::count_if(
                edges.begin(), edges.end(), [](const auto &_edge)
                {
                  return _edge.second.get().Weight() >= 1.0;
                });
            if (outDegreeNoAliases)
            {
              errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
                  "FrameAttachedToGraph error, "
                  "STATIC_MODEL vertex with name [" +
                  vertexName +
                  "] should have no outgoing edges "
                  "in WORLD attached_to graph."});
            }
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

  // Expect scopeContextName to be either "__model__" or "world"
  if (_in.ScopeContextName() != "__model__" &&
      _in.ScopeContextName() != "world")
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph error: scope context name " +
            _in.ScopeContextName() + " does not match __model__ or world."});
    return errors;
  }

  // Expect the scope vertex (which is the source vertex for this graph) to be
  // valid and check that the scope context matches the frame type.
  auto sourceVertex = _in.ScopeVertex();
  if (!sourceVertex.Valid())
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
                     "PoseRelativeToGraph error: source frame[" +
                     _in.ScopeContextName() + "] not found in graph."});
    return errors;
  }

  sdf::FrameType sourceFrameType = sourceVertex.Data();
  if (_in.ScopeContextName() == "__model__" &&
      sourceFrameType != sdf::FrameType::MODEL)
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph error, "
        "source vertex with name __model__ should have FrameType MODEL."});
    return errors;
  }
  else if (_in.ScopeContextName() == "world" &&
           sourceFrameType != sdf::FrameType::WORLD)
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph error, "
        "source vertex with name world should have FrameType WORLD."});
    return errors;
  }

  // Check number of incoming edges for each vertex
  auto vertices = _in.Graph().Vertices();
  for (auto vertexPair : vertices)
  {
    const std::string vertexName = _in.VertexLocalName(vertexPair.second.get());
    // Vertex names should not be empty
    if (vertexName.empty())
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
          "PoseRelativeToGraph error, "
          "vertex with empty name detected."});
    }

    std::size_t inDegree = _in.Graph().InDegree(vertexPair.first);

    if (inDegree > 1)
    {
      errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
          "PoseRelativeToGraph error, "
          "too many incoming edges at a vertex with name [" +
          vertexName + "]."});
    }
    else if (vertexName == "__root__" )
    {
      if (sdf::FrameType::MODEL != sourceFrameType &&
          sdf::FrameType::STATIC_MODEL != sourceFrameType &&
          sdf::FrameType::WORLD != sourceFrameType)
      {
        errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
            "PoseRelativeToGraph error,"
            " __root__ should have FrameType MODEL, STATIC_MODEL or WORLD."});
      }
      else if (inDegree != 0)
      {
        std::string graphType =
            sourceFrameType == sdf::FrameType::WORLD ? "WORLD" : "MODEL";
        errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
            "PoseRelativeToGraph error,"
            " __root__ vertex should have no incoming edges in " +
                graphType + " relative_to graph."});
      }
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
          if (_in.ScopeVertexId() == vertexPair.first)
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
          if (inDegree != 1)
          {
            errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
                "PoseRelativeToGraph error, "
                "WORLD vertices should have 1 incoming edge "
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
    if (name == "__root__")
      continue;
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

  if (_in.ScopeContextName() != "__model__" &&
      _in.ScopeContextName() != "world")
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "FrameAttachedToGraph error: scope context name[" +
            _in.ScopeContextName() + "] does not match __model__ or world."});
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

  if (_in.ScopeContextName() == "world" &&
      !(sinkVertex.Data() == FrameType::WORLD ||
          sinkVertex.Data() == FrameType::STATIC_MODEL ||
          sinkVertex.Data() == FrameType::LINK))
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "Graph has world scope but sink vertex named [" + sinkVertex.Name() +
            "] does not have FrameType WORLD, LINK or STATIC_MODEL "
            "when starting from vertex with name [" +
            _vertexName + "]."});
    return errors;
  }

  if (_in.ScopeContextName() == "__model__")
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
             sinkVertex.Data() != FrameType::STATIC_MODEL)
    {
      errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
          "Graph has __model__ scope but sink vertex named [" +
          sinkVertex.Name() + "] does not have FrameType LINK OR STATIC_MODEL "
          "when starting from vertex with name [" + _vertexName + "]."});
      return errors;
    }
  }

  _attachedToBody = _in.VertexLocalName(sinkVertex);

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

  return resolvePoseRelativeToRoot(
      _pose, _graph, _graph.VertexIdByName(_vertexName));
}
/////////////////////////////////////////////////
Errors resolvePoseRelativeToRoot(
      ignition::math::Pose3d &_pose,
      const ScopedGraph<PoseRelativeToGraph> &_graph,
      const ignition::math::graph::VertexId &_vertexId)
{
  Errors errors;

  auto incomingVertexEdges = FindSourceVertex(_graph, _vertexId, errors);

  if (!errors.empty())
  {
    return errors;
  }
  else if (!incomingVertexEdges.first.Valid())
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph unable to find path to source vertex "
        "when starting from vertex with id [" +
            std::to_string(_vertexId) + "]."});
    return errors;
  }
  else if (incomingVertexEdges.first.Id() != _graph.ScopeVertex().Id())
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph frame with name [" + std::to_string(_vertexId) +
            "] is disconnected; its source vertex has name [" +
            incomingVertexEdges.first.Name() + "], but its source name should "
            "be " + _graph.ScopeContextName() + "."});
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
Errors resolvePose(ignition::math::Pose3d &_pose,
    const ScopedGraph<PoseRelativeToGraph> &_graph,
    const ignition::math::graph::VertexId &_frameVertexId,
    const ignition::math::graph::VertexId &_resolveToVertexId)
{
  Errors errors = resolvePoseRelativeToRoot(_pose, _graph, _frameVertexId);

  // If the resolveTo is empty, we're resolving to the Root, so we're done
  if (_resolveToVertexId != ignition::math::graph::kNullId)
  {
    ignition::math::Pose3d poseR;
    Errors errorsR =
        resolvePoseRelativeToRoot(poseR, _graph, _resolveToVertexId);
    errors.insert(errors.end(), errorsR.begin(), errorsR.end());

    if (errors.empty())
    {
      _pose = poseR.Inverse() * _pose;
    }
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
  Errors errors;
  if (_graph.Count(_frameName) != 1)
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
        "PoseRelativeToGraph unable to find unique frame with name [" +
        _frameName + "] in graph."});
    return errors;
  }
  if (_graph.Count(_resolveTo) != 1)
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_INVALID,
        "PoseRelativeToGraph unable to find unique frame with name [" +
        _resolveTo + "] in graph."});
    return errors;
  }

  return resolvePose(_pose, _graph, _graph.VertexIdByName(_frameName),
      _graph.VertexIdByName(_resolveTo));
}
}
}
