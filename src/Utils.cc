/*
 * Copyright 2017 Open Source Robotics Foundation
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
#include <utility>
#include "Utils.hh"

/////////////////////////////////////////////////
bool sdf::loadName(sdf::ElementPtr _sdf, std::string &_name)
{
  // Read the name
  std::pair<std::string, bool> namePair = _sdf->Get<std::string>("name", "");

  _name = namePair.first;
  return namePair.second;
}

/////////////////////////////////////////////////
bool sdf::loadPose(sdf::ElementPtr _sdf, ignition::math::Pose3d &_pose,
              std::string &_frame)
{
  sdf::ElementPtr sdf = _sdf;
  if (_sdf->GetName() != "pose")
  {
    if (_sdf->HasElement("pose"))
      sdf = _sdf->GetElement("pose");
    else
      return false;
  }

  // Read the frame. An empty frame implies the parent frame.
  std::pair<std::string, bool> framePair = sdf->Get<std::string>("frame", "");

  // Read the pose value.
  using Pose3d = ignition::math::Pose3d;
  std::pair<Pose3d, bool> posePair =
    sdf->Get<Pose3d>("", Pose3d::Zero);

  // Set output, but only if the return value is true.
  if (posePair.second)
  {
    _pose = posePair.first;
    _frame = framePair.first;
  }

  // The frame attribute is optional, so only return true or false based
  // on the pose element value.
  return posePair.second;
}

//////////////////////////////////////////////////
ignition::math::Pose3d sdf::poseInFrame(const std::string &_src,
    const std::string &_dst, FrameGraph &_graph)
{
  using Matrix4d = ignition::math::Matrix4d;
  using Pose3d = ignition::math::Pose3d;

  using CostInfo = ignition::math::graph::CostInfo;
  using VertexId = ignition::math::graph::VertexId;

  const Pose3d poseInf(
    ignition::math::INF_D, ignition::math::INF_D, ignition::math::INF_D,
    ignition::math::INF_D, ignition::math::INF_D, ignition::math::INF_D);

  if (_src.empty())
    return poseInf;

  // Handle the case where the source and destination are the same.
  if (_src == _dst)
    return Pose3d::Zero;

  // Get the source vertex.
  const ignition::math::graph::VertexRef_M<PoseWithFrameName> srcVertices =
      _graph.Vertices(_src);

  // Get all the vertices in the frame graph that match the provided frame.
  // If _dst is empty, then the result of this function (poseInFrame) will
  // be the pose of the _src frame.
  const ignition::math::graph::VertexRef_M<PoseWithFrameName> dstVertices =
      _graph.Vertices(_dst);

  // There should be only one vertex for the source vertex, and 1 or
  // 0 vertices for the destination vertex.
  if (srcVertices.size() != 1 || dstVertices.size() > 1)
    return poseInf;

  const VertexId srcId = srcVertices.begin()->first;

  // Short circuit if the destination is empty.
  if (dstVertices.empty())
    return _graph.VertexFromId(srcId).Data().first;

  const VertexId dstId = dstVertices.begin()->first;

  // Run Dijkstra to find a path from _src to _dst
  std::map<VertexId, CostInfo> costMap =
    ignition::math::graph::Dijkstra(_graph, srcId, dstId);

  // // Dijkstra debug output
  // std::cerr << "src[" << srcId << "]"
  //           << ", dst[" << dstId << "]"
  //           << std::endl;
  // for (auto vv : costMap)
  // {
  //   std::cout << "DestId[" << vv.first << "] Cost["
  //     << vv.second.first << "] PrevId[" << vv.second.second
  //     << "]" << std::endl;
  // }

  Matrix4d poseResult = Matrix4d::Identity;

  // Start from destination vertex and work back to the source.
  for (VertexId current = dstId, next; current != srcId; current = next)
  {
    // Get next vertex in path from destination to source.
    next = costMap.find(current)->second.second;

    // Get the edge between current and next.
    const ignition::math::graph::DirectedEdge<Matrix4d> &edge =
        _graph.EdgeFromVertices(current, next);

    // std::cerr << "  current " << current
    //           << ", next " << next
    //           << ", edge " << edge.Id()
    //           << std::endl;

    // Make sure the edge is valid.
    if (edge.Id() !=
        ignition::math::graph::DirectedEdge<Matrix4d>::NullEdge.Id())
    {
      poseResult *= edge.Data();
    }
    else
    {
      // Invalid edge, set inifinite pose to indicate error case.
      return poseInf;
    }
  }

  return poseResult.Pose();
}
