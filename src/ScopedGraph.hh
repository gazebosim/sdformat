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

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <sdf/sdf_config.h>

#include <ignition/math/graph/Graph.hh>
#include <ignition/math/Pose3.hh>

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
template <typename T>
class ScopedGraph
{

  public:
  template <typename G>
  struct GraphTypeExtracter
  {
    using Vertex = void;
    using Edge = void;
  };

  public:
  template <typename V, typename E>
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
  public: using MapType = std::unordered_map<std::string, VertexId>;

  public: ScopedGraph();
  // TODO: maybe _rootVertex can have a default value
  public: ScopedGraph(const std::shared_ptr<T> _graph, VertexId _rootVertex);
  public: ScopedGraph(const std::shared_ptr<T> _graph);
  public: explicit operator bool() const;

  public: const MathGraphType &Graph() const;
  public: const MapType &Map() const;

  // May not be needed if they can be called via Graph()
  public: Vertex &AddVertex(const std::string &_name, const VertexType &);
  public: Edge &AddEdge(const ignition::math::graph::VertexId_P &, const EdgeType &);
  public: size_t OutDegree(const VertexId &) const;
  public: size_t InDegree(const VertexId &) const;

  public: const ignition::math::graph::VertexRef_M<VertexType> Vertices() const;
  public: const ignition::math::graph::VertexRef_M<VertexType> Vertices(
      const std::string &) const;

  public: void UpdateEdge(std::reference_wrapper<const Edge> &, const EdgeType&);

  public: size_t Count(const std::string &_name) const;
  public: VertexId VertexIdByName(const std::string &_name) const;
  public: VertexId RootVertexId() const;

  public: bool IsTopLevel() const;
  public: bool PointsTo(const std::shared_ptr<T> _graph) const;
  public: std::string SetScopeName(const std::string &_name) const;
  public: std::string ScopeName() const;


  private: std::weak_ptr<T> graph;

  // private: std::unordered_set<VertexId> vertices;
  private: VertexId rootVertex;
  private: std::string prefix;
  private: std::string scopeName;
};

}
}

#endif /* SDF_SCOPED_GRAPH_HH */
