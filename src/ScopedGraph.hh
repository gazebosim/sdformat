/*
 * Copyright 2020 Open Source Robotics Foundation
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

#ifndef SDF_SCOPED_GRAPH_HH
#define SDF_SCOPED_GRAPH_HH

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ignition/math/Pose3.hh>
#include <ignition/math/graph/Graph.hh>

#include "sdf/sdf_config.h"

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {

/// \brief Data structure that holds information associated with each scope
struct ScopedGraphData
{
  /// \brief The vertex ID of the designated scope vertex
  ignition::math::graph::VertexId scopeVertexId {
      ignition::math::graph::kNullId};

  /// \brief The prefix for all names under this scope
  std::string prefix {};

  /// \brief The name of this scope. Either world or __model__
  std::string scopeName {};
};

// Forward declarations for static_assert
class PoseRelativeToGraph;
class FrameAttachedToGraph;

/// \brief The ScopedGraph allows manipulating FrameAttachedTo and
/// PoseRelativeTo graphs within a smaller scope such as the scope of a model
/// inside a world graph.
///
/// Each valid instance of ScopedGraph has a pointer to either a
/// FrameAttachedToGraph or a PoseRelativeToGraph. In addition, each instance
/// contains the following:
/// - The prefix of the scope, which is empty for a world scope and the
/// name of a model for a model scope.
/// - The scope name of the scope, which can be either "world" or "__model__".
/// - ID of the vertex on which the scope is anchored. For
/// PoseRelativeToGraph's, this vertex is the root of the sub tree represented
/// by the ScopedGraph instance. For FrameAttachedToGraph, this vertex only
/// represents the scope boundary and does not imply that it's the root of the
/// sub tree because these graphs can have multiple disconnected trees.
///
/// When a vertex is added to the graph via ScopedGraph, the prefix is prepended
/// to the input name of the vertex. The name of the vertex with the prefix is
/// known as the absolute name of the vertex and the name of the vertex without
/// the prefix is known as the local name of the vertex.
///
/// \remark ScopedGraph has pointer semantic. i.e, a ScopedGraph object and its
/// copy point to the same underlying data and modifying one will affect the
/// other.
///
/// \tparam T Either FrameAttachedTo or PoseRelativeTo graph
template <typename T>
class ScopedGraph
{
  static_assert(
      std::is_same_v<T, sdf::PoseRelativeToGraph> ||
      std::is_same_v<T, sdf::FrameAttachedToGraph>,
      "Template parameter has to be either sdf::PoseRelativeToGraph or "
      "sdf::FrameAttachedToGraph");

  /// \brief template for extracting the graph type from the template parameter.
  /// \tparam G ignition::math::DirectedGraph type
  /// \return Vertex and Edge types of G
  public: template <typename G>
          struct GraphTypeExtracter
  {
    using Vertex = void;
    using Edge = void;
  };

  /// \brief Specialization for GraphTypeExtracter on
  /// ignition::math::DirectedGraph
  public: template <typename V, typename E>
          struct GraphTypeExtracter<ignition::math::graph::DirectedGraph<V, E>>
  {
    using Vertex = V;
    using Edge = E;
  };

  // Type aliases
  public: using MathGraphType = typename T::GraphType;
  public: using VertexId = ignition::math::graph::VertexId;
  public: using VertexType = typename GraphTypeExtracter<MathGraphType>::Vertex;
  public: using EdgeType = typename GraphTypeExtracter<MathGraphType>::Edge;
  public: using Vertex = ignition::math::graph::Vertex<VertexType>;
  public: using Edge = ignition::math::graph::DirectedEdge<EdgeType>;
  public: using MapType = typename T::MapType;

  /// \brief Default constructor. The constructed object is invalid as it
  /// doesn't point to any graph.
  public: ScopedGraph() = default;

  /// \brief Constructor. The constructed object holds a weak pointer to the
  /// passed in graph.
  /// \param[in] _graph A shared pointer to PoseRelativeTo or FrameAttachedTo
  /// graph.
  public: explicit ScopedGraph(const std::shared_ptr<T> &_graph);

  /// \brief Creates a scope anchored at an existing child model vertex.
  /// \param[in] _name Name of child model vertex. The prefix of the current
  /// scope will be prepended to this name before searching for the vertex that
  /// matches the name. The new scope will have a new prefix formed by appending
  /// _name to the existing prefix of the current scope.
  /// \return A new child scope.
  public: ScopedGraph<T> ChildModelScope(const std::string &_name);

  /// \brief Checks if the scope points to a valid graph.
  /// \return True if the scope points to a valid graph.
  public: explicit operator bool() const;

  /// \brief Immutable reference to the underlying PoseRelativeTo::graph or
  /// FrameAttachedTo::graph.
  public: const MathGraphType &Graph() const;

  /// \brief Immutable reference to the underlying PoseRelativeTo::map or
  /// FrameAttachedTo::map.
  public: const MapType &Map() const;

  /// \brief Adds a scope vertex to the graph. This creates a new
  /// scope by updating the prefix of the current scope, adding a new scope
  /// vertex to the graph, and setting a new scope name.
  /// \param[in] _prefix The new prefix of the scope.
  /// \param[in] _name The Name of the scope vertex to be added. The full name
  /// of the vertex will be newPrefix::_name, where newPrefix is the prefix
  /// obtained by adding _prefix to the existing prefix of the scope.
  public: Vertex &AddScopeVertex(const std::string &_prefix,
              const std::string &_name, const std::string &_scopeName,
              const VertexType &_data);

  /// \brief Adds a vertex to the graph.
  /// \param[in] _name The local name of the vertex. The absolute name of the
  /// vertex will be _prefix::_name.
  /// \param[in] _data Vertex data.
  /// \return The newly created vertex.
  public: Vertex &AddVertex(const std::string &_name, const VertexType &_data);

  /// \brief Adds an edge to the graph.
  /// \param[in] _vertexPair Pair of vertex IDs between which the edge will be
  /// created.
  /// \param[in] _data Edge data.
  /// \return The newly created edge.
  public: Edge &AddEdge(const ignition::math::graph::VertexId_P &_vertexPair,
              const EdgeType &_data);

  /// \brief Gets all the local names of the vertices in the current scope.
  /// \return A list of vertex names in the current scope.
  public: std::vector<std::string> VertexNames() const;

  /// \brief Get the local name of a vertex.
  /// \param[in] _id ID of the vertex.
  /// \return The local name of the vertex. If the vertex was not found in the
  /// graph, "__null__" will be returned.
  public: std::string VertexLocalName(const VertexId &_id) const;

  /// \brief Get the local name of a vertex.
  /// \param[in] _vertex Vertex object.
  /// \return The local name of the vertex. If the vertex was not found in the
  /// graph, "__null__" will be returned.
  public: std::string VertexLocalName(const Vertex &_vertex) const;

  /// \brief Update the information contained by an edge
  /// \remark Since this function has to remove the edge from the graph and add
  /// a new one, the original edge will have a new ID.
  /// \param[in] _edge The edge to update.
  /// \param[in] _data The new data.
  public: void UpdateEdge(Edge &_edge, const EdgeType &_data);

  /// \brief Count the number of vertices with a given local name in the scope.
  /// \param[in] _name Local name query
  /// \return Number of vertices that have the given local name in the scope.
  public: size_t Count(const std::string &_name) const;

  /// \brief Get the vertex ID of a vertex with a give local name.
  /// \param[in] _name Local name of the vertex
  /// \return Vertex ID of the vertex if found in this scope of the graph.
  /// Otherwise, kNullId is returned.
  public: VertexId VertexIdByName(const std::string &_name) const;

  /// \brief Get the scope vertex.
  /// \return Immutable reference to the scope vertex of this scope.
  public: const Vertex &ScopeVertex() const;

  /// \brief Get the scope vertex ID.
  /// \return Immutable reference to the scope vertex of this scope.
  public: VertexId ScopeVertexId() const;

  /// \brief Check if the graph on which this scope is based is the same as the
  /// input graph.
  /// \param[in] _graph Graph object to check.
  /// \return True if this scope points to the same graph as the input.
  public: bool PointsTo(const std::shared_ptr<T> &_graph) const;

  /// \brief Set the scope name.
  /// \param[in] _name New scope name.
  public: void SetScopeName(const std::string &_name);

  /// \brief Get the current scope name.
  /// \return The current scope name.
  public: const std::string &ScopeName() const;

  /// \brief Add the current prefix to the given name.
  /// \param[in] _name Input name.
  /// \return The name with the prefix prepended.
  public: std::string AddPrefix(const std::string &_name) const;

  /// \brief Find the current prefix in the given name. If the prefix is found,
  /// remove it.
  /// \param[in] _name Input name.
  /// \return The name with the prefix removed and true, if the prefix is found.
  /// Otherwise, the original name and false
  public: std::pair<std::string, bool> FindAndRemovePrefix(
              const std::string &_name) const;

  /// \brief Weak pointer to either a FrameAttachedToGraph or
  /// PoseRelativeToGraph.
  private: std::weak_ptr<T> graphWeak;

  /// \brief Shared pointer to the scope's data. A shared_ptr is used because
  /// this data has to be shared among many DOM objects.
  private: std::shared_ptr<ScopedGraphData> dataPtr;
};

/////////////////////////////////////////////////
template <typename T>
ScopedGraph<T>::ScopedGraph(const std::shared_ptr<T> &_graph)
    : graphWeak(_graph)
    , dataPtr(std::make_shared<ScopedGraphData>())
{
}

/////////////////////////////////////////////////
template <typename T>
ScopedGraph<T> ScopedGraph<T>::ChildModelScope(const std::string &_name)
{
  auto newScopedGraph = *this;
  newScopedGraph.dataPtr = std::make_shared<ScopedGraphData>();
  newScopedGraph.dataPtr->scopeVertexId = this->VertexIdByName(_name);
  newScopedGraph.dataPtr->prefix = this->AddPrefix(_name);
  newScopedGraph.dataPtr->scopeName = "__model__";
  return newScopedGraph;
}

/////////////////////////////////////////////////
template <typename T>
ScopedGraph<T>::operator bool() const
{
  return !this->graphWeak.expired() && (this->dataPtr != nullptr);
}

/////////////////////////////////////////////////
template <typename T>
auto ScopedGraph<T>::Graph() const -> const MathGraphType &
{
  return this->graphWeak.lock()->graph;
}

/////////////////////////////////////////////////
template <typename T>
auto ScopedGraph<T>::Map() const -> const MapType &
{
  return this->graphWeak.lock()->map;
}

/////////////////////////////////////////////////
template <typename T>
auto ScopedGraph<T>::AddScopeVertex(const std::string &_prefix,
    const std::string &_name, const std::string &_scopeName,
    const VertexType &_data) -> Vertex &
{
  this->dataPtr->prefix = this->AddPrefix(_prefix);
  Vertex &vert = this->AddVertex(_name, _data);
  this->dataPtr->scopeVertexId = vert.Id();
  this->SetScopeName(_scopeName);
  return vert;
}

/////////////////////////////////////////////////
template <typename T>
auto ScopedGraph<T>::AddVertex(
    const std::string &_name, const VertexType &_data) -> Vertex &
{
  auto graph = this->graphWeak.lock();
  const std::string newName = this->AddPrefix(_name);
  Vertex &vert = graph->graph.AddVertex(newName, _data);
  graph->map[newName] = vert.Id();
  return vert;
}

/////////////////////////////////////////////////
template <typename T>
auto ScopedGraph<T>::AddEdge(
    const ignition::math::graph::VertexId_P &_vertexPair, const EdgeType &_data)
    -> Edge &
{
  auto graph = graphWeak.lock();
  Edge &edge = graph->graph.AddEdge(_vertexPair, _data);
  return edge;
}

/////////////////////////////////////////////////
template <typename T>
std::vector<std::string> ScopedGraph<T>::VertexNames() const
{
  std::vector<std::string> out;
  for (const auto &namePair : this->Map())
  {
    const auto &idNamePair = this->FindAndRemovePrefix(namePair.first);
    if (idNamePair.second)
    {
      out.push_back(idNamePair.first);
    }
  }
  return out;
}

/////////////////////////////////////////////////
template <typename T>
std::string ScopedGraph<T>::VertexLocalName(const Vertex &_vert) const
{
  return this->FindAndRemovePrefix(_vert.Name()).first;
}

/////////////////////////////////////////////////
template <typename T>
std::string ScopedGraph<T>::VertexLocalName(const VertexId &_id) const
{
  return this->VertexLocalName(this->Graph().VertexFromId(_id));
}

/////////////////////////////////////////////////
template <typename T>
void ScopedGraph<T>::UpdateEdge(Edge &_edge, const EdgeType &_data)
{
  // There's no API to update the data of an edge, so we remove the edge and
  // insert a new one with the new pose.
  auto tailVertexId = _edge.Tail();
  auto headVertexId = _edge.Head();
  auto &graph = this->graphWeak.lock()->graph;
  graph.RemoveEdge(_edge.Id());
  _edge = graph.AddEdge({tailVertexId, headVertexId}, _data);
}

/////////////////////////////////////////////////
template <typename T>
const std::string &ScopedGraph<T>::ScopeName() const
{
  return this->dataPtr->scopeName;
}

/////////////////////////////////////////////////
template <typename T>
void ScopedGraph<T>::SetScopeName(const std::string &_name)
{
  this->dataPtr->scopeName = _name;
}

/////////////////////////////////////////////////
template <typename T>
std::size_t ScopedGraph<T>::Count(const std::string &_name) const
{
  return this->graphWeak.lock()->map.count(this->AddPrefix(_name));
}

/////////////////////////////////////////////////
template <typename T>
auto ScopedGraph<T>::VertexIdByName(const std::string &_name) const -> VertexId
{
  auto &map = this->Map();
  auto it = map.find(this->AddPrefix(_name));
  if (it != map.end())
    return it->second;
  else
    return ignition::math::graph::kNullId;
}

/////////////////////////////////////////////////
template <typename T>
auto ScopedGraph<T>::ScopeVertex() const -> const Vertex &
{
  return this->graphWeak.lock()->graph.VertexFromId(
      this->dataPtr->scopeVertexId);
}

/////////////////////////////////////////////////
template <typename T>
auto ScopedGraph<T>::ScopeVertexId() const -> VertexId
{
  return this->dataPtr->scopeVertexId;
}

/////////////////////////////////////////////////
template <typename T>
bool ScopedGraph<T>::PointsTo(const std::shared_ptr<T> &_graph) const
{
  return this->graphWeak.lock() == _graph;
}

/////////////////////////////////////////////////
template <typename T>
std::string ScopedGraph<T>::AddPrefix(const std::string &_name) const
{
  if (this->dataPtr->prefix.empty())
  {
    return _name;
  }
  else
  {
    return this->dataPtr->prefix + "::" + _name;
  }
}

/////////////////////////////////////////////////
template <typename T>
std::pair<std::string, bool>
ScopedGraph<T>::FindAndRemovePrefix(const std::string &_name) const
{
  if (this->dataPtr->prefix.empty())
  {
    return std::make_pair(_name, true);
  }

  if (0 ==
      _name.compare(0, this->dataPtr->prefix.size(), this->dataPtr->prefix))
  {
    return std::make_pair(_name.substr(this->dataPtr->prefix.size() + 2), true);
  }
  return std::make_pair(_name, false);
}
}
}

#endif /* SDF_SCOPED_GRAPH_HH */
