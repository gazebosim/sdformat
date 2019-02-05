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
    return Matrix4d::Identity.Pose();

  // Get the source vertex.
  const ignition::math::graph::VertexRef_M<Matrix4d> srcVertices =
      _graph.Vertices(_src);

  // Get all the vertices in the frame graph that match the provided frame.
  // If _dst is empty, then the result of this function (poseInFrame) will
  // be the pose of the _src frame.
  const ignition::math::graph::VertexRef_M<Matrix4d> dstVertices =
      _graph.Vertices(_dst);

  // There should be only one vertex for the source vertex, and 1 or
  // 0 vertices for the destination vertex.
  if (srcVertices.size() != 1 || dstVertices.size() > 1)
    return poseInf;

  // Short circuit if the destination is empty.
  if (dstVertices.empty())
    return _graph.VertexFromId(srcVertices.begin()->first).Data().Pose();

  // Run Dijkstra to find a path from _src to _dst
  std::map<VertexId, CostInfo> result =
    ignition::math::graph::Dijkstra(_graph,
        srcVertices.begin()->first, dstVertices.begin()->first);

  // // Dijkstra debug output
  // for (auto vv : result)
  // {
  //   std::cout << "DestId[" << vv.first << "] Cost["
  //     << vv.second.first << "] PrevId[" << vv.second.second
  //     << "]" << std::endl;
  // }

  Matrix4d finalPose = Matrix4d::Identity;
  for (VertexId nextVertex, key = dstVertices.begin()->first;;
       key = nextVertex)
  {
    // Get the next vertex in the path from the destination vertex to the
    // source vertex.
    nextVertex = result.find(key)->second.second;

    // Are we at the source vertex, which is the end of the line.
    if (nextVertex == key)
    {
      // Compute the final pose and break
      finalPose *= _graph.VertexFromId(key).Data();
      break;
    }

    // Get the edge between nextVertex and key.
    const ignition::math::graph::DirectedEdge<int> &edge =
      _graph.EdgeFromVertices(nextVertex, key);

    // Make sure the edge is valid.
    if (edge.Id() != ignition::math::graph::DirectedEdge<int>::NullEdge.Id())
    {
      // // Debug output:
      // std::cout << "Key[" << key << "] Edge From[" << edge.Head()
      //   << "] To[" << edge.Tail() << "] Data[" << edge.Data() << "]\n";

      // Get the direction of the edge.
      // \todo I think we look at just the Head() and Tail() of the edge and
      // compare those values to key and nextVertex.
      bool inverse = edge.Data() < 0;

      if (inverse)
        finalPose *= _graph.VertexFromId(key).Data().Inverse();
      else if (key != dstVertices.begin()->first)
        finalPose *=  _graph.VertexFromId(key).Data();
    }
    else
    {
      /// \todo(nkoenig) This is an error case. Inform the caller somehow.
      break;
    }
  }

  return finalPose.Pose();
}
