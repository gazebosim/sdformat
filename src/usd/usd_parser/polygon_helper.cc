/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "polygon_helper.hh"

#include <vector>
#include <cassert>

namespace usd
{
  std::vector<unsigned int> PolygonToTriangles(
      pxr::VtIntArray &_faceVertexIndices,
      pxr::VtIntArray &_faceVertexCounts,
      pxr::VtArray<pxr::GfVec3f> &_points)
  {
    // TODO: Use more robust algorithms.
    // For reference, blender supports "ear-clipping", and "Beauty".
    // ref: https://blender.stackexchange.com/questions/215553/what-algorithm-is-used-for-beauty-triangulation
    // ref: https://en.wikipedia.org/wiki/Polygon_triangulation
    size_t count = 0;
    for (const auto &vCount : _faceVertexCounts)
    {
      count += vCount - 2;
    }
    std::vector<unsigned int> triangles;
    triangles.reserve(count * 3);

    size_t cur = 0;
    for (const auto &vCount : _faceVertexCounts)
    {
      for (size_t i = cur + 2; i < cur + vCount; i++)
      {
        triangles.emplace_back(_faceVertexIndices[cur]);
        triangles.emplace_back(_faceVertexIndices[i - 1]);
        triangles.emplace_back(_faceVertexIndices[i]);
      }
      cur += vCount;
    }
    assert(triangles.size() == count * 3);

    return triangles;
  }
}
