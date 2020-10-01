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
#include <sdf/sdf_config.h>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <ignition/math/Pose3.hh>
#include <ignition/math/graph/Graph.hh>

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
struct ScopedGraphData
{
  // std::unordered_set<VertexId> vertices;
  ignition::math::graph::VertexId scopeVertexId {
      ignition::math::graph::kNullId};

  std::string prefix {};

  std::string scopeName {};
};

template <typename T>
class ScopedGraph
{

  public: template <typename G>
          struct GraphTypeExtracter
  {
    using Vertex = void;
    using Edge = void;
  };

  public: template <typename V, typename E>
          struct GraphTypeExtracter<ignition::math::graph::DirectedGraph<V, E>>
  {
    using Vertex = V;
    using Edge = E;
  };

  public: using MathGraphType = typename T::GraphType;
  public: using VertexId = ignition::math::graph::VertexId;
  public: using VertexType = typename GraphTypeExtracter<MathGraphType>::Vertex;
  public: using EdgeType = typename GraphTypeExtracter<MathGraphType>::Edge;
  public: using Vertex = ignition::math::graph::Vertex<VertexType>;
  public: using Edge = ignition::math::graph::DirectedEdge<EdgeType>;
  public: using MapType = typename T::MapType;
  public: ScopedGraph() = default;
  public: ScopedGraph(const std::shared_ptr<T> _graph);
  public: ScopedGraph<T> ChildScope(
              const std::string &_name, const std::string &_scopeName);
  public: explicit operator bool() const;
  public: const MathGraphType &Graph() const;
  public: const MapType &Map() const;
  // May not be needed if they can be called via Graph()
  public: Vertex &AddScopeVertex(const std::string &_prefix,
              const std::string &_name, const VertexType &_data);
  public: Vertex &AddVertex(const std::string &_name, const VertexType &);
  public: Edge &AddEdge(
              const ignition::math::graph::VertexId_P &, const EdgeType &);
  public: const ignition::math::graph::VertexRef_M<VertexType> Vertices() const;
  public: const ignition::math::graph::VertexRef_M<VertexType> Vertices(
              const std::string &) const;
  public: std::vector<std::string> VertexNames() const;
  public: std::string VertexName(const VertexId &_id) const;
  public: std::string VertexName(const Vertex &_vertex) const;
  public: void UpdateEdge(const Edge &, const EdgeType &);
  public: size_t Count(const std::string &_name) const;
  public: VertexId VertexIdByName(const std::string &_name) const;
  public: Vertex ScopeVertex() const;
  public: VertexId ScopeVertexId() const;
  public: bool PointsTo(const std::shared_ptr<T> _graph) const;
  public: void SetScopeName(const std::string &_name);
  public: const std::string &ScopeName() const;
  public: std::string AddPrefix(const std::string &_name) const;
  public: std::string RemovePrefix(const std::string &_name) const;
  private: std::weak_ptr<T> graphWeak;
  private: std::shared_ptr<ScopedGraphData> dataPtr;
};

/////////////////////////////////////////////////
template <typename T>
ScopedGraph<T>::ScopedGraph(const std::shared_ptr<T> _graph)
    : graphWeak(_graph)
    , dataPtr(std::make_shared<ScopedGraphData>())
{
}

/////////////////////////////////////////////////
template <typename T>
ScopedGraph<T> ScopedGraph<T>::ChildScope(
    const std::string &_name, const std::string &_scopeName)
{
  auto newScopedGraph = *this;
  newScopedGraph.dataPtr = std::make_shared<ScopedGraphData>();
  newScopedGraph.dataPtr->scopeVertexId = this->VertexIdByName(_name);
  newScopedGraph.dataPtr->prefix = this->AddPrefix(_name);
  newScopedGraph.dataPtr->scopeName = _scopeName;
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
    const std::string &_name, const VertexType &_data) -> Vertex &
{
  this->dataPtr->prefix = this->AddPrefix(_prefix);
  Vertex &vert = this->AddVertex(_name, _data);
  this->dataPtr->scopeVertexId = vert.Id();
  return vert;
}

/////////////////////////////////////////////////
template <typename T>
auto ScopedGraph<T>::AddVertex(
    const std::string &_name, const VertexType &_data) -> Vertex &
{
  auto graph = graphWeak.lock();
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
auto ScopedGraph<T>::Vertices() const
    -> const ignition::math::graph::VertexRef_M<VertexType>
{
  auto graph = graphWeak.lock();
  return graph->graph.Vertices();
}

/////////////////////////////////////////////////
template <typename T>
auto ScopedGraph<T>::Vertices(const std::string &_name) const
    -> const ignition::math::graph::VertexRef_M<VertexType>
{
  auto graph = graphWeak.lock();
  return graph->graph.Vertices(this->AddPrefix(_name));
}

/////////////////////////////////////////////////
template <typename T>
std::vector<std::string> ScopedGraph<T>::VertexNames() const
{
  std::vector<std::string> out;
  for (const auto &namePair : this->Map())
  {
    out.push_back(this->RemovePrefix(namePair.first));
  }
  return out;
}

/////////////////////////////////////////////////
template <typename T>
std::string ScopedGraph<T>::VertexName(const Vertex &_vert) const
{
  return this->RemovePrefix(_vert.Name());
}

/////////////////////////////////////////////////
template <typename T>
std::string ScopedGraph<T>::VertexName(const VertexId &_id) const
{
  return this->VertexName(this->Graph().VertexFromId(_id));
}

/////////////////////////////////////////////////
template <typename T>
void ScopedGraph<T>::UpdateEdge(const Edge &_edge, const EdgeType &_data)
{
  // There's no API to update the data of an edge, so we remove the edge and
  // insert a new one with the new pose.
  auto tailVertexId = _edge.Tail();
  auto headVertexId = _edge.Head();
  auto &graph = this->graphWeak.lock()->graph;
  graph.RemoveEdge(_edge.Id());
  graph.AddEdge({tailVertexId, headVertexId}, _data);
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
std::size_t ScopedGraph<T>::VertexIdByName(const std::string &_name) const
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
auto ScopedGraph<T>::ScopeVertex() const -> Vertex
{
  return this->graphWeak.lock()->graph.VertexFromId(
      this->dataPtr->scopeVertexId);
}

/////////////////////////////////////////////////
template <typename T>
std::size_t ScopedGraph<T>::ScopeVertexId() const
{
  return this->dataPtr->scopeVertexId;
}

/////////////////////////////////////////////////
template <typename T>
bool ScopedGraph<T>::PointsTo(const std::shared_ptr<T> _graph) const
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
std::string ScopedGraph<T>::RemovePrefix(const std::string &_name) const
{
  if (!this->dataPtr->prefix.empty())
  {
    auto ind = _name.find(this->dataPtr->prefix + "::");
    if (ind == 0)
    {
      return _name.substr(this->dataPtr->prefix.size() + 2);
    }
  }
  return _name;
}

}
}

#endif /* SDF_SCOPED_GRAPH_HH */
