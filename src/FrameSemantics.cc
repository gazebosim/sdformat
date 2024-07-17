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
#include <set>
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
#include "Utils.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {

// Helpful functions when debugging in gdb
void printGraph(const ScopedGraph<PoseRelativeToGraph> &_graph)
{
  std::cout << _graph.Graph() << std::endl;
}
void printGraph(const ScopedGraph<FrameAttachedToGraph> &_graph)
{
  std::cout << _graph.Graph() << std::endl;
}

// The following two functions were originally submitted to gz-math,
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
    const gz::math::graph::VertexId _id, Errors &_errors)
{
  using DirectedEdge = typename ScopedGraph<T>::Edge;
  using Vertex = typename ScopedGraph<T>::Vertex;
  using VertexId = gz::math::graph::VertexId;
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
    const gz::math::graph::VertexId _id,
    Errors &_errors)
{
  using DirectedEdge = typename ScopedGraph<T>::Edge;
  using Vertex = typename ScopedGraph<T>::Vertex;
  using VertexId = gz::math::graph::VertexId;
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
    const gz::math::Pose3d &_rawPose,
    const std::string &_placementFrame,
    const sdf::ScopedGraph<PoseRelativeToGraph> &_graph,
    gz::math::Pose3d &_resolvedPose)
{
  sdf::Errors errors;

  // Alias for better pose notation
  gz::math::Pose3d &X_RM = _resolvedPose;
  const gz::math::Pose3d &X_RPf = _rawPose;

  // If the model has a placement frame, calculate the necessary pose to use
  if (_placementFrame != "")
  {
    gz::math::Pose3d X_MPf;

    errors = sdf::resolvePoseRelativeToRoot(X_MPf, _graph, _placementFrame);
    if (errors.empty())
    {
      X_RM = X_RPf * X_MPf.Inverse();
    }
  }

  return errors;
}

/// \brief Base struct the contains a few common members. These structs provide
/// a common API used by the build*Graph functions to access the various
/// attributes and retrieve children of regular DOM objects and Interface
/// Elements.
struct WrapperBase
{
  /// \brief Name of the entity
  const std::string name;
  /// \brief Element type, such as Model, Link, or Interface Frame.
  const std::string elementType;
  /// \brief Frame type used in the FrameAttachedTo or PoseRelativeTo graphs.
  const FrameType frameType;
};

/// \brief Wrapper for sdf::Link and sdf::InterfaceLink
struct LinkWrapper : public WrapperBase
{
  /// \brief Constructor that takes an sdf::Link
  explicit LinkWrapper(const sdf::Link &_link)
      : WrapperBase{_link.Name(), "Link", FrameType::LINK},
        rawPose(_link.RawPose()),
        rawRelativeTo(_link.PoseRelativeTo()),
        relativeTo(rawRelativeTo)
  {
  }

  /// \brief Constructor that takes an sdf::InterfaceLink
  explicit LinkWrapper(const sdf::InterfaceLink &_ifaceLink)
      : WrapperBase{_ifaceLink.Name(), "Interface Link", FrameType::LINK},
        rawPose(_ifaceLink.PoseInModelFrame()),
        rawRelativeTo("__model__"),
        relativeTo(rawRelativeTo)
  {
  }

  /// \brief Constructor from name and pose data (without sdf::Link or
  /// sdf::InterfaceLink)
  LinkWrapper(const std::string &_name, const gz::math::Pose3d &_rawPose,
              const std::string &_relativeTo)
      : WrapperBase{_name, "Link", FrameType::LINK},
        rawPose(_rawPose),
        rawRelativeTo(_relativeTo),
        relativeTo(rawRelativeTo)
  {
  }

  /// \brief Raw pose of the entity.
  const gz::math::Pose3d rawPose;
  /// \brief The //pose/@relative_to attribute.
  const std::string rawRelativeTo;
  /// \brief The final @relative_to attribute. This is the same as rawRelativeTo
  /// for links and interface links.
  const std::string relativeTo;
};

/// \brief Wrapper for sdf::Frame and sdf::InterfaceFrame
struct FrameWrapper : public WrapperBase
{
  /// \brief Constructor that takes an sdf::Frame
  explicit FrameWrapper(const sdf::Frame &_frame)
      : WrapperBase{_frame.Name(), "Frame", FrameType::FRAME},
        rawPose(_frame.RawPose()),
        rawRelativeTo(_frame.PoseRelativeTo()),
        attachedTo(_frame.AttachedTo()),
        relativeTo(rawRelativeTo.empty() ? attachedTo : rawRelativeTo)
  {
  }

  /// \brief Constructor that takes an sdf::InterfaceFrame
  explicit FrameWrapper(const sdf::InterfaceFrame &_ifaceFrame)
      : WrapperBase{_ifaceFrame.Name(), "Interface Frame", FrameType::FRAME},
        rawPose(_ifaceFrame.PoseInAttachedToFrame()),
        rawRelativeTo(_ifaceFrame.AttachedTo()),
        attachedTo(_ifaceFrame.AttachedTo()),
        relativeTo(attachedTo)

  {
  }

  /// \brief Constructor from name, pose, and attachement data (without
  /// sdf::Frame or sdf::InterfaceFrame)
  FrameWrapper(const std::string &_name, const gz::math::Pose3d &_rawPose,
               const std::string &_relativeTo, const std::string &_attachedTo)
      : WrapperBase{_name, "Frame", FrameType::FRAME},
        rawPose(_rawPose),
        rawRelativeTo(_relativeTo),
        attachedTo(_attachedTo),
        relativeTo(rawRelativeTo.empty() ? attachedTo : rawRelativeTo)
  {
  }

  /// \brief Raw pose of the entity.
  const gz::math::Pose3d rawPose;
  /// \brief The //pose/@relative_to attribute.
  const std::string rawRelativeTo;
  /// \brief The //frame/@attached_to attribute.
  const std::string attachedTo;
  /// \brief The final @relative_to attribute. For sdf::Frame, this is set to
  /// the attachedTo if the rawRelativeTo is empty. For sdf::InterfaceFrame,
  /// it's always set to attachedTo.
  const std::string relativeTo;
};

/// \brief Wrapper for sdf::Joint and sdf::InterfaceJoint
struct JointWrapper : public WrapperBase
{
  /// \brief Constructor that takes an sdf::Joint
  explicit JointWrapper(const sdf::Joint &_joint)
      : WrapperBase{_joint.Name(), "Joint", FrameType::JOINT},
        rawPose(_joint.RawPose()),
        rawRelativeTo(_joint.PoseRelativeTo()),
        childName(_joint.ChildName()),
        relativeTo(rawRelativeTo.empty() ? childName : rawRelativeTo)
  {
  }

  /// \brief Constructor that takes an sdf::InterfaceJoint
  explicit JointWrapper(const sdf::InterfaceJoint &_ifaceJoint)
      : WrapperBase{_ifaceJoint.Name(), "Interface Joint", FrameType::JOINT},
        rawPose(_ifaceJoint.PoseInChildFrame()),
        rawRelativeTo(_ifaceJoint.ChildName()),
        childName(_ifaceJoint.ChildName()),
        relativeTo(childName)
  {
  }

  /// \brief Raw pose of the entity.
  const gz::math::Pose3d rawPose;
  /// \brief The //pose/@relative_to attribute.
  const std::string rawRelativeTo;
  /// \brief The name of the child frame (i.e. content of //joint/child).
  const std::string childName;
  /// \brief The final @relative_to attribute. For sdf::Joint, this is set to
  /// the childName if the rawRelativeTo is empty. For sdf::InterfaceJoint, it's
  /// always set to childName.
  const std::string relativeTo;
};

/// \brief Placement frame information
struct PlacementFrameInfo
{
  /// \brief Computed name of the proxy model frame
  std::string proxyName;
  /// \brief The name of placement frame from //include/placement_frame.
  std::string placementFrameName;
};

/// \brief Wrapper for sdf::Model and sdf::InterfaceModel
struct ModelWrapper : public WrapperBase
{
  /// \brief Constructor that takes an sdf::Model
  explicit ModelWrapper(const sdf::Model &_model)
      : WrapperBase{_model.Name(), "Model",
                    _model.Static() ? FrameType::STATIC_MODEL
                                    : FrameType::MODEL},
        rawPose(_model.RawPose()),
        rawRelativeTo(_model.PoseRelativeTo()),
        relativeTo(rawRelativeTo),
        canonicalLinkName(_model.CanonicalLinkAndRelativeName().second),
        placementFrameName(_model.PlacementFrameName()),
        isStatic(_model.Static())
  {
    for (uint64_t i = 0; i < _model.LinkCount(); ++i)
    {
      this->links.emplace_back(*_model.LinkByIndex(i));
    }
    for (uint64_t i = 0; i < _model.FrameCount(); ++i)
    {
      this->frames.emplace_back(*_model.FrameByIndex(i));
    }
    for (uint64_t i = 0; i < _model.JointCount(); ++i)
    {
      this->joints.emplace_back(*_model.JointByIndex(i));
    }
    for (uint64_t i = 0; i < _model.ModelCount(); ++i)
    {
      this->models.emplace_back(*_model.ModelByIndex(i));
    }
    for (uint64_t i = 0; i < _model.InterfaceModelCount(); ++i)
    {
      this->models.emplace_back(*_model.InterfaceModelNestedIncludeByIndex(i),
                                *_model.InterfaceModelByIndex(i));
    }
    for (const auto &[nestedInclude, model] : _model.MergedInterfaceModels())
    {
      const std::string proxyModelFrameName = computeMergedModelProxyFrameName(
          nestedInclude->LocalModelName().value_or(model->Name()));

      std::string poseRelativeTo =
          nestedInclude->IncludePoseRelativeTo().value_or("");
      if (poseRelativeTo.empty())
      {
        poseRelativeTo = "__model__";
      }
      this->frames.emplace_back(proxyModelFrameName,
                                nestedInclude->IncludeRawPose().value_or(
                                    model->ModelFramePoseInParentFrame()),
                                poseRelativeTo, model->CanonicalLinkName());

      for (const auto &item : model->Links())
      {
        this->links.emplace_back(item.Name(), item.PoseInModelFrame(),
                                 proxyModelFrameName);
      }
      for (const auto &item : model->Frames())
      {
        std::string attachedTo = item.AttachedTo();
        if (item.AttachedTo() == "__model__")
        {
          attachedTo = proxyModelFrameName;
        }
        this->frames.emplace_back(item.Name(), item.PoseInAttachedToFrame(),
                                  attachedTo, attachedTo);
      }
      for (const auto &item : model->Joints())
      {
        std::string childName = item.ChildName();
        if (item.ChildName() == "__model__")
        {
          childName = proxyModelFrameName;
        }
        this->joints.emplace_back(sdf::InterfaceJoint(item.Name(), childName,
                                                      item.PoseInChildFrame()));
      }
      if (nestedInclude->PlacementFrame().has_value())
      {
        this->mergedModelPlacements.push_back(
            {proxyModelFrameName, *nestedInclude->PlacementFrame()});
      }

      // Skip adding nested interface models because they are already included
      // in the parent model's list of nested models.
    }
  }

  /// \brief Constructor that takes an sdf::NestedInclude and
  /// sdf::InterfaceModel.
  explicit ModelWrapper(const NestedInclude &_nestedInclude,
                        const sdf::InterfaceModel &_ifaceModel)
      : WrapperBase{_ifaceModel.Name(), "Interface Model",
                    _ifaceModel.Static() ? FrameType::STATIC_MODEL
                                         : FrameType::MODEL},
        rawPose(_nestedInclude.IncludeRawPose().value_or(
            _ifaceModel.ModelFramePoseInParentFrame())),
        rawRelativeTo(_nestedInclude.IncludePoseRelativeTo().value_or("")),
        relativeTo(rawRelativeTo),
        canonicalLinkName(_ifaceModel.CanonicalLinkName()),
        placementFrameName(_nestedInclude.PlacementFrame().value_or("")),
        isStatic(_ifaceModel.Static())
  {
    this->AddInterfaceChildren(_ifaceModel);
  }

  /// \brief Constructor that takes an sdf::InterfaceModel. These are
  /// InterfaceModels nested under other InterfaceModels.
  explicit ModelWrapper(const sdf::InterfaceModel &_ifaceModel)
      : WrapperBase{_ifaceModel.Name(), "Interface Model",
                    _ifaceModel.Static() ? FrameType::STATIC_MODEL
                                         : FrameType::MODEL},
        rawPose(_ifaceModel.ModelFramePoseInParentFrame()),
        rawRelativeTo(""),
        relativeTo(rawRelativeTo),
        canonicalLinkName(_ifaceModel.CanonicalLinkName()),
        placementFrameName(""),
        isStatic(_ifaceModel.Static())
  {
    this->AddInterfaceChildren(_ifaceModel);
  }

  /// \brief Raw pose of the entity.
  const gz::math::Pose3d rawPose;
  /// \brief The //pose/@relative_to attribute.
  const std::string rawRelativeTo;
  /// \brief The final @relative_to attribute. This is set to rawRelativeTo for
  /// sdf::Model and sdf::InterfaceModel.
  const std::string relativeTo;
  /// \brief The name of the resolved canonical link.
  const std::string canonicalLinkName;
  /// \brief The name of the placement frame.
  const std::string placementFrameName;
  /// \brief Whether the model/interface model is static.
  const bool isStatic;

  /// \brief Children links and interface links.
  std::vector<LinkWrapper> links;
  /// \brief Children frames and interface frames.
  std::vector<FrameWrapper> frames;
  /// \brief Children joints and interface joints.
  std::vector<JointWrapper> joints;
  /// \brief Children nested models and interface models.
  std::vector<ModelWrapper> models;
  /// \brief Placement frame information for each merged model.
  std::vector<PlacementFrameInfo> mergedModelPlacements;

  /// \brief Helper function to add children of interface models.
  private: void AddInterfaceChildren(const sdf::InterfaceModel &_ifaceModel)
  {
    for (const auto &item : _ifaceModel.Links())
    {
      this->links.emplace_back(item);
    }
    for (const auto &item : _ifaceModel.Frames())
    {
      this->frames.emplace_back(item);
    }
    for (const auto &item : _ifaceModel.Joints())
    {
      this->joints.emplace_back(item);
    }
    for (const auto &item : _ifaceModel.NestedModels())
    {
      this->models.emplace_back(*item);
    }
  }
};

/// \brief Wrapper for sdf::World
struct WorldWrapper : public WrapperBase
{
  /// \brief Constructor that takes an sdf::World
  explicit WorldWrapper(const sdf::World &_world)
      : WrapperBase{_world.Name(), "World", FrameType::WORLD}
  {
    for (uint64_t i = 0; i < _world.FrameCount(); ++i)
    {
      this->frames.emplace_back(*_world.FrameByIndex(i));
    }
    for (uint64_t i = 0; i < _world.JointCount(); ++i)
    {
      this->joints.emplace_back(*_world.JointByIndex(i));
    }
    for (uint64_t i = 0; i < _world.ModelCount(); ++i)
    {
      this->models.emplace_back(*_world.ModelByIndex(i));
    }
    for (uint64_t i = 0; i < _world.InterfaceModelCount(); ++i)
    {
      this->models.emplace_back(*_world.InterfaceModelNestedIncludeByIndex(i),
                                *_world.InterfaceModelByIndex(i));
    }
  }

  /// \brief Children frames.
  std::vector<FrameWrapper> frames;
  /// \brief Children joints.
  std::vector<JointWrapper> joints;
  /// \brief Children models and interface models.
  std::vector<ModelWrapper> models;
};

/////////////////////////////////////////////////
/// \brief Add vertices to either Frame attached to or Pose graph. If the child
/// element is a model, it calls the corresponding build*Graph function.
/// \tparam ElementT The type of Element. This must be a class that derives from
/// WrapperBase.
/// \tparam GraphT Type of Graph. Either PoseRelativeToGraph or
/// FrameAttachedToGraph.
/// \param[in,out] _out The graph to which vertices will be added.
/// \param[in] _items List of Elements such as links, joints, etc for which
/// vertices will be created in the graph.
/// \param[in] _parent Parent element of the items in `_items`.
/// \param[out] _errors Errors encountered while adding vertices.
template <typename ElementT, typename GraphT>
void addVerticesToGraph(ScopedGraph<GraphT> &_out,
                        const std::vector<ElementT> &_items,
                        const WrapperBase &_parent, Errors &_errors)
{
  for (const auto &item : _items)
  {
    if (_out.Count(item.name) > 0)
    {
      _errors.emplace_back(ErrorCode::DUPLICATE_NAME, item.elementType +
          " with non-unique name [" + item.name + "] detected in " +
          lowercase(_parent.elementType) + " with name [" +
          _parent.name + "].");
      continue;
    }
    if constexpr (std::is_same_v<ElementT, ModelWrapper>)
    {
      if constexpr (std::is_same_v<GraphT, sdf::FrameAttachedToGraph>)
      {
        auto nestedErrors = wrapperBuildFrameAttachedToGraph(_out, item, false);
        _errors.insert(_errors.end(), nestedErrors.begin(), nestedErrors.end());
      }
      else
      {
        auto nestedErrors = wrapperBuildPoseRelativeToGraph(_out, item, false);
        _errors.insert(_errors.end(), nestedErrors.begin(), nestedErrors.end());
      }
    }
    else
    {
      _out.AddVertex(item.name, item.frameType);
    }
  }
}

/////////////////////////////////////////////////
/// \brief Add edges to the PoseRelativeTo graph.
/// \tparam ElementT The type of Element. This must be a class that derives from
/// WrapperBase.
/// \param[in,out] _out The PoseRelativeTo graph to which edges will be added.
/// \param[in] _items List of Elements such as links, joints, etc for which
/// edges will be created in the graph.
/// \param[in] _parent Parent element of the items in `_items`.
/// \param[out] _errors Errors encountered while adding edges.
template <typename ElementT>
void addEdgesToGraph(ScopedGraph<PoseRelativeToGraph> &_out,
                     const std::vector<ElementT> &_items,
                     const WrapperBase &_parent, Errors &_errors)
{
  for (const auto &item : _items)
  {
    const std::string &relativeTo = item.relativeTo;
    const gz::math::Pose3d poseInRelativeTo = item.rawPose;

    auto itemId = _out.VertexIdByName(item.name);
    auto relativeToId = _out.ScopeVertexId();
    std::string typeForErrorMsg = "relative_to";
    ErrorCode errorCode = ErrorCode::POSE_RELATIVE_TO_INVALID;

    if constexpr (std::is_same_v<ElementT, FrameWrapper>)
    {
      if (item.rawRelativeTo.empty())
      {
        typeForErrorMsg = "attached_to";
        errorCode = ErrorCode::FRAME_ATTACHED_TO_INVALID;
      }
    }

    if (!relativeTo.empty())
    {
      // look for vertex in graph that matches relative_to value
      if (_out.Count(relativeTo) != 1)
      {
        std::stringstream errMsg;
        errMsg << typeForErrorMsg << " name[" << relativeTo << "] specified by "
               << lowercase(item.elementType) << " with name[" << item.name
               << "] does not match a";
        if (_parent.frameType == FrameType::WORLD)
        {
          errMsg << " model or frame name ";
        }
        else
        {
          errMsg << " nested model, link, joint, or frame name ";
        }
        errMsg << "in " + lowercase(_parent.elementType) + " with name[" +
                      _parent.name + "].";

        _errors.push_back({errorCode, errMsg.str()});
        continue;
      }

      relativeToId = _out.VertexIdByName(relativeTo);
      if (item.name == relativeTo)
      {
        _errors.push_back({ErrorCode::POSE_RELATIVE_TO_CYCLE,
            "relative_to name[" + relativeTo +
            "] is identical to " + lowercase(item.elementType) + " name[" +
            item.name + "], causing a graph cycle in " +
            lowercase(_parent.elementType) + " with name[" +
            _parent.name + "]."});
      }
    }

    if constexpr (std::is_same_v<ElementT, ModelWrapper>)
    {
      gz::math::Pose3d resolvedModelPose = item.rawPose;

      sdf::Errors resolveErrors = resolveModelPoseWithPlacementFrame(
          item.rawPose, item.placementFrameName,
          _out.ChildModelScope(item.name), resolvedModelPose);
      _errors.insert(_errors.end(), resolveErrors.begin(), resolveErrors.end());

      _out.AddEdge({relativeToId, itemId}, resolvedModelPose);
    }
    else
    {
      _out.AddEdge({relativeToId, itemId}, poseInRelativeTo);
    }
  }
}

/////////////////////////////////////////////////
/// \brief Add Joint and InterfaceJoint edges to the FrameAttachedTo graph
/// \param[in,out] _out The FrameAttachedTo graph to which edges will be added.
/// \param[in] _joints List of joints for which edges will be created in the
/// graph.
/// \param[in] _parent Parent of the joints in `_joints`.
/// \param[out] _errors Errors encountered while adding edges.
void addEdgesToGraph(ScopedGraph<FrameAttachedToGraph> &_out,
                     const std::vector<JointWrapper> &_joints,
                     const WrapperBase &_parent, Errors &_errors)
{
  for (const auto &joint : _joints)
  {
    auto jointId = _out.VertexIdByName(joint.name);

    if (_out.Count(joint.childName) != 1)
    {
      _errors.push_back(
          {ErrorCode::JOINT_CHILD_LINK_INVALID,
           "Child frame with name[" + joint.childName + "] specified by " +
               lowercase(joint.elementType) + " with name[" + joint.name +
               "] not found in " + lowercase(_parent.elementType) +
               " with name[" + _parent.name + "]."});
      continue;
    }
    auto childFrameId = _out.VertexIdByName(joint.childName);
    _out.AddEdge({jointId, childFrameId}, true);
  }
}

/////////////////////////////////////////////////
/// \brief Add Frame and InterfaceFrame edges to the FrameAttachedTo graph
/// \param[in,out] _out The FrameAttachedTo graph to which edges will be added.
/// \param[in] _frames List of frames for which edges will be created in the
/// graph.
/// \param[in] _parent Parent of the frames in `_frames`.
/// \param[out] _errors Errors encountered while adding edges.
void addEdgesToGraph(ScopedGraph<FrameAttachedToGraph> &_out,
                     const std::vector<FrameWrapper> &_frames,
                     const WrapperBase &_parent, Errors &_errors)
{
  for (const auto &frame : _frames)
  {
    auto frameId = _out.VertexIdByName(frame.name);
    // look for vertex in graph that matches attached_to value
    std::string attachedTo = frame.attachedTo;
    if (attachedTo.empty())
    {
      // if the attached-to name is empty, use the default attachedTo
      attachedTo = _out.ScopeContextName();
    }

    if (_out.Count(attachedTo) != 1)
    {
      std::stringstream errMsg;
      errMsg << "attached_to name[" << attachedTo << "] specified by "
             << lowercase(frame.elementType) << " with name[" << frame.name
             << "] does not match a";
      if (_parent.frameType == FrameType::WORLD)
      {
        errMsg << " model or frame name ";
      }
      else
      {
        errMsg << " nested model, link, joint, or frame name ";
      }
      errMsg << "in " + lowercase(_parent.elementType) + " with name[" +
                    _parent.name + "].";

      _errors.push_back({ErrorCode::FRAME_ATTACHED_TO_INVALID, errMsg.str()});
      continue;
    }

    auto attachedToId = _out.VertexIdByName(attachedTo);
    bool edgeData = true;
    if (frame.name == frame.attachedTo)
    {
      // set edgeData to false if attaches to itself, since this is invalid
      edgeData = false;
      _errors.push_back({ErrorCode::FRAME_ATTACHED_TO_CYCLE,
          "attached_to name[" + attachedTo +
          "] is identical to frame name[" + frame.attachedTo +
          "], causing a graph cycle in " + lowercase(_parent.elementType) +
          " with name[" + _parent.name + "]."});
    }
    _out.AddEdge({frameId, attachedToId}, edgeData);
  }
}

/////////////////////////////////////////////////
/// \brief Helper function that actually build a FrameAttachedToGraph given a
/// wrapped Model or Interface Model.
/// \param[out] _out Graph object to write.
/// \param[in] _model Wrapped Model or Interface model from which to build
/// attached_to graph.
/// \param[in] _isRoot True if the model is a standalone model, i.e,
/// //sdf/model. This is not relevant if the _model is a wrapped Interface
/// Model.
/// \return Errors.
Errors wrapperBuildFrameAttachedToGraph(ScopedGraph<FrameAttachedToGraph> &_out,
                                        const ModelWrapper &_model,
                                        bool _isRoot)
{
  Errors errors;

  if (_model.links.size() == 0u && _model.models.size() == 0 &&
      !_model.isStatic)
  {
    errors.push_back({ErrorCode::MODEL_WITHOUT_LINK,
                      "A model must have at least one link."});
    return errors;
  }

  auto frameType = _model.frameType;

  const std::string scopeContextName = "__model__";

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
  }

  const auto modelId = _out.AddVertex(_model.name, frameType).Id();

  auto outModel = _out.AddScopeVertex(
      _model.name, scopeContextName, scopeContextName, frameType);
  const auto modelFrameId = outModel.ScopeVertexId();

  // Add an aliasing edge from the model vertex to the
  // corresponding vertex in the FrameAttachedTo graph of the child model,
  auto &edge = outModel.AddEdge({modelId, modelFrameId}, true);
  // Set the edge weight to 0 to indicate that this is an aliasing edge.
  edge.SetWeight(0);

  // add link vertices
  addVerticesToGraph(outModel, _model.links, _model, errors);

  // add joint vertices
  addVerticesToGraph(outModel, _model.joints, _model, errors);

  // add frame vertices
  addVerticesToGraph(outModel, _model.frames, _model, errors);

  // add nested model vertices
  addVerticesToGraph(outModel, _model.models, _model, errors);

  // add edges from joint to child frames
  addEdgesToGraph(outModel, _model.joints, _model, errors);

  // add frame edges
  addEdgesToGraph(outModel, _model.frames, _model, errors);

  // identify canonical link, which may be nested
  const std::string canonicalLinkName = _model.canonicalLinkName;
  const auto canonicalLinkId = outModel.VertexIdByName(canonicalLinkName);
  if (!_model.isStatic)
  {
    if (gz::math::graph::kNullId == canonicalLinkId)
    {
      if (canonicalLinkName.empty())
      {
        if (_model.models.size() == 0u)
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
          auto firstChildModelId =
              outModel.VertexIdByName(_model.models.front().name);
          outModel.AddEdge({modelFrameId, firstChildModelId}, true);
        }
      }
      else
      {
        errors.push_back({ErrorCode::MODEL_CANONICAL_LINK_INVALID,
            "canonical_link with name[" + canonicalLinkName +
            "] not found in model with name[" + _model.name + "]."});
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
    ScopedGraph<FrameAttachedToGraph> &_out, const Model *_model, bool _isRoot)
{
  if (!_model)
  {
    return Errors{{ErrorCode::ELEMENT_INVALID, "Invalid sdf::Model pointer."}};
  }
  return wrapperBuildFrameAttachedToGraph(_out, ModelWrapper(*_model), _isRoot);
}

/////////////////////////////////////////////////
Errors buildFrameAttachedToGraph(ScopedGraph<FrameAttachedToGraph> &_out,
                                 const InterfaceModel *_model)
{
  if (!_model)
  {
    return Errors{
        {ErrorCode::ELEMENT_INVALID, "Invalid sdf::InterfaceModel pointer."}};
  }
  return wrapperBuildFrameAttachedToGraph(_out, ModelWrapper(*_model), false);
}


/////////////////////////////////////////////////
Errors buildFrameAttachedToGraph(
            ScopedGraph<FrameAttachedToGraph> &_out, const WorldWrapper &_world)
{
  Errors errors;

  // add implicit world frame vertex first
  const std::string scopeContextName = "world";
  _out = _out.AddScopeVertex(
      "", scopeContextName, scopeContextName, sdf::FrameType::WORLD);

  // add model vertices
  addVerticesToGraph(_out, _world.models, _world, errors);

  // add joint vertices
  addVerticesToGraph(_out, _world.joints, _world, errors);

  // add frame vertices
  addVerticesToGraph(_out, _world.frames, _world, errors);

  // add edges from joint to child frames
  addEdgesToGraph(_out, _world.joints, _world, errors);

  // add frame edges
  addEdgesToGraph(_out, _world.frames, _world, errors);

  return errors;
}

/////////////////////////////////////////////////
Errors buildFrameAttachedToGraph(
            ScopedGraph<FrameAttachedToGraph> &_out, const World *_world)
{
  if (!_world)
  {
    return Errors{{ErrorCode::ELEMENT_INVALID, "Invalid sdf::World pointer."}};
  }

  return buildFrameAttachedToGraph(_out, WorldWrapper(*_world));
}

/////////////////////////////////////////////////
/// \brief Helper function that actually builds the PoseRelativeToGraph given a
/// wrapped Model or Interface model.
/// \param[out] _out Graph object to write.
/// \param[in] _model Wrapped Model or Interface model from which to build
/// relative_to graph.
/// \param[in] _isRoot True if the model is a standalone model, i.e,
/// //sdf/model. This is not relevant if the _model is a wrapped Interface
/// Model.
/// \return Errors.
Errors wrapperBuildPoseRelativeToGraph(ScopedGraph<PoseRelativeToGraph> &_out,
                                       const ModelWrapper &_model, bool _isRoot)
{
  Errors errors;

  const std::string scopeContextName = "__model__";
  auto rootId = gz::math::graph::kNullId;
  // add the model frame vertex first
  if (_isRoot)
  {
    _out = _out.AddScopeVertex(
        "", "__root__", scopeContextName, sdf::FrameType::MODEL);
    rootId = _out.ScopeVertexId();
  }
  auto modelId = _out.AddVertex(_model.name, sdf::FrameType::MODEL).Id();
  auto outModel = _out.AddScopeVertex(_model.name, scopeContextName,
      scopeContextName, sdf::FrameType::MODEL);
  auto modelFrameId = outModel.ScopeVertexId();

  // Add an aliasing edge from the model vertex to the
  // corresponding vertex in the PoseRelativeTo graph of the child model,
  // i.e, to the <model_name>::__model__ vertex.
  auto &edge = _out.AddEdge({modelId, modelFrameId}, {});
  // Set the edge weight to 0 to indicate that this is an aliasing edge.
  edge.SetWeight(0);

  // add link vertices
  addVerticesToGraph(outModel, _model.links, _model, errors);

  // add joint vertices
  addVerticesToGraph(outModel, _model.joints, _model, errors);

  // add frame vertices
  addVerticesToGraph(outModel, _model.frames, _model, errors);

  // add nested model vertices
  addVerticesToGraph(outModel, _model.models, _model, errors);

  // now that all vertices have been added to the graph,
  // add the edges that reference other vertices

  // add link edges
  addEdgesToGraph(outModel, _model.links, _model, errors);

  // add joint edges
  addEdgesToGraph(outModel, _model.joints, _model, errors);

  // add frame edges
  addEdgesToGraph(outModel, _model.frames, _model, errors);

  // add nested model edges
  addEdgesToGraph(outModel, _model.models, _model, errors);

  if (_isRoot)
  {
    // We have to add this edge now with an identity pose to be able to call
    // resolveModelPoseWithPlacementFrame, which in turn calls
    // sdf::resolvePoseRelativeToRoot. We will later update the edge after the
    // pose is calculated.
    auto rootToModel = outModel.AddEdge({rootId, modelId}, {});
    gz::math::Pose3d resolvedModelPose = _model.rawPose;
    sdf::Errors resolveErrors =
        resolveModelPoseWithPlacementFrame(_model.rawPose,
            _model.placementFrameName, outModel, resolvedModelPose);
    errors.insert(errors.end(), resolveErrors.begin(), resolveErrors.end());

    outModel.UpdateEdge(rootToModel, resolvedModelPose);
  }

  // For each merge model, update the edge between the parent model and the
  // proxy model frame to take into account the placement frame used when
  // nesting the merged model via //include.
  for (const auto &[proxyName, placementFrameName] :
       _model.mergedModelPlacements)
  {
    auto proxyId = outModel.VertexIdByName(proxyName);
    auto modelToProxy =
        outModel.Graph().EdgeFromVertices(modelFrameId, proxyId);
    const auto rawPose = modelToProxy.Data();

    // We have to first set the edge data to an identity pose to be able to call
    // resolveModelPoseWithPlacementFrame, which in turn calls
    // sdf::resolvePoseRelativeToRoot. We will later update the edge after the
    // pose is calculated.
    outModel.UpdateEdge(modelToProxy, gz::math::Pose3d::Zero);
    gz::math::Pose3d resolvedModelPose;
    sdf::Errors resolveErrors =
        resolveModelPoseWithPlacementFrame(rawPose,
            placementFrameName, outModel, resolvedModelPose);
    errors.insert(errors.end(), resolveErrors.begin(), resolveErrors.end());
    outModel.UpdateEdge(modelToProxy, resolvedModelPose);
  }
  return errors;
}
/////////////////////////////////////////////////
Errors buildPoseRelativeToGraph(
    ScopedGraph<PoseRelativeToGraph> &_out, const Model *_model, bool _isRoot)
{
  if (!_model)
  {
    return Errors{{ErrorCode::ELEMENT_INVALID, "Invalid sdf::Model pointer."}};
  }

  return wrapperBuildPoseRelativeToGraph(_out, ModelWrapper(*_model), _isRoot);
}

/////////////////////////////////////////////////
Errors buildPoseRelativeToGraph(ScopedGraph<PoseRelativeToGraph> &_out,
                                const InterfaceModel *_model)
{
  if (!_model)
  {
    return Errors{
        {ErrorCode::ELEMENT_INVALID, "Invalid sdf::InterfaceModel pointer."}};
  }
  return wrapperBuildPoseRelativeToGraph(_out, ModelWrapper(*_model), false);
}

/////////////////////////////////////////////////
Errors wrapperBuildPoseRelativeToGraph(
    ScopedGraph<PoseRelativeToGraph> &_out, const WorldWrapper &_world)
{
  Errors errors;

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
  addVerticesToGraph(_out, _world.models, _world, errors);

  // add joint vertices
  addVerticesToGraph(_out, _world.joints, _world, errors);

  // add frame vertices
  addVerticesToGraph(_out, _world.frames, _world, errors);

  // now that all vertices have been added to the graph,
  // add the edges that reference other vertices
  // add model edges
  addEdgesToGraph(_out, _world.models, _world, errors);

  // add joint edges
  addEdgesToGraph(_out, _world.joints, _world, errors);

  // add frame edges
  addEdgesToGraph(_out, _world.frames, _world, errors);

  return errors;
}

/////////////////////////////////////////////////
Errors buildPoseRelativeToGraph(
    ScopedGraph<PoseRelativeToGraph> &_out, const World *_world)
{
  if (!_world)
  {
    return Errors{{ErrorCode::ELEMENT_INVALID, "Invalid sdf::World pointer."}};
  }

  return wrapperBuildPoseRelativeToGraph(_out, WorldWrapper(*_world));
}

/////////////////////////////////////////////////
Errors validateFrameAttachedToGraph(
    const ScopedGraph<FrameAttachedToGraph> &_in)
{
  Errors errors;

  // Check if scope points to a valid graph
  if (!_in)
  {
    errors.push_back({ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR,
        "FrameAttachedToGraph error: scope does not point to a valid graph."});
    return errors;
  }

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

  // Check if scope points to a valid graph
  if (!_in)
  {
    errors.push_back({ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR,
        "PoseRelativeToGraph error: scope does not point to a valid graph."});
    return errors;
  }

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
    gz::math::Pose3d pose;
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
          sinkVertex.Name() + "] does not have FrameType LINK or STATIC_MODEL "
          "when starting from vertex with name [" + _vertexName + "]."});
      return errors;
    }
  }

  _attachedToBody = _in.VertexLocalName(sinkVertex);

  return errors;
}

/////////////////////////////////////////////////
Errors resolvePoseRelativeToRoot(
      gz::math::Pose3d &_pose,
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
      gz::math::Pose3d &_pose,
      const ScopedGraph<PoseRelativeToGraph> &_graph,
      const gz::math::graph::VertexId &_vertexId)
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

  gz::math::Pose3d pose;
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
Errors resolvePose(gz::math::Pose3d &_pose,
    const ScopedGraph<PoseRelativeToGraph> &_graph,
    const gz::math::graph::VertexId &_frameVertexId,
    const gz::math::graph::VertexId &_resolveToVertexId)
{
  Errors errors = resolvePoseRelativeToRoot(_pose, _graph, _frameVertexId);

  // If the resolveTo is empty, we're resolving to the Root, so we're done
  if (_resolveToVertexId != gz::math::graph::kNullId)
  {
    gz::math::Pose3d poseR;
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
    gz::math::Pose3d &_pose,
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
